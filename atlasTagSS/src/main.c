/*
 * Copyright (c) 2025 Your Name
 * SPDX-License-License-Identifier: Apache-2.0
 */
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <dwm3000.h>

LOG_MODULE_REGISTER(main, CONFIG_LOG_DEFAULT_LEVEL);

BUILD_ASSERT(DT_NODE_HAS_COMPAT(DT_CHOSEN(zephyr_console), zephyr_cdc_acm_uart),
             "Console device is not ACM CDC UART device");

static struct dwm3000_context dwm3000_ctx;
static struct dwm3000_config dwm3000_cfg = {
    .spi_dev = NULL,
    .gpio_dev = NULL,
    .cs_pin = 5,
    .reset_pin = 28,
    .wakeup_pin = 29,
    .irq_pin = 3,
    .spi_cfg = {
        .frequency = 2000000,
        .operation = SPI_WORD_SET(8) | SPI_OP_MODE_MASTER,
        .slave = 0,
        .cs = NULL,
    },
};

dwt_txconfig_t txconfig_options = {
    0x34,       /* PG delay. */
    0xfdfdfdfd, /* TX power. */
    0x0         /*PG count*/
};

/* Default communication configuration. We use default non-STS DW mode. */
static dwt_config_t config = {
    .chan            = 5,               /* Channel number. */
    .txPreambLength  = DWT_PLEN_128,    /* Preamble length. Used in TX only. */
    .rxPAC           = DWT_PAC8,        /* Preamble acquisition chunk size. Used in RX only. */
    .txCode          = 9,               /* TX preamble code. Used in TX only. */
    .rxCode          = 9,               /* RX preamble code. Used in RX only. */
    .sfdType         = 1,    /* 0 to use standard 8 symbol SFD */
    .dataRate        = DWT_BR_6M8,      /* Data rate. */
    .phrMode         = DWT_PHRMODE_STD, /* PHY header mode. */
    .phrRate         = DWT_PHRRATE_STD, /* PHY header rate. */
    .sfdTO           = (129 + 8 - 8),   /* SFD timeout */
    .stsMode         = DWT_STS_MODE_OFF,
    .stsLength       = DWT_STS_LEN_64,  /* STS length, see allowed values in Enum dwt_sts_lengths_e */
    .pdoaMode        = DWT_PDOA_M0      /* PDOA mode off */
};

/* Inter-ranging delay period, in milliseconds. */
#define RNG_DELAY_MS 1000

/* Default antenna delay values for 64 MHz PRF. See NOTE 1 below. */
#define TX_ANT_DLY 16385
#define RX_ANT_DLY 16385


/* Frames used in the ranging process. See NOTE 3 below. */
static uint8_t tx_poll_msg[] = { 0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0xE0, 0, 0 };
static uint8_t rx_resp_msg[] = { 0x41, 0x88, 0, 0xCA, 0xDE, 'V', 'E', 'W', 'A', 0xE1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

/* Length of the common part of the message (up to and including the function code, see NOTE 2 below). */
#define ALL_MSG_COMMON_LEN 10
/* Indexes to access some of the fields in the frames defined above. */
#define ALL_MSG_SN_IDX          2
#define RESP_MSG_POLL_RX_TS_IDX 10
#define RESP_MSG_RESP_TX_TS_IDX 14
#define RESP_MSG_TS_LEN         4

/* Frame sequence number, incremented after each transmission. */
static uint8_t frame_seq_nb = 0;

/* Buffer to store received response message.
 * Its size is adjusted to longest frame that this example code is supposed to handle. */
#define RX_BUF_LEN 20
static uint8_t rx_buffer[RX_BUF_LEN];

/* Hold copy of status register state here for reference so that it can be examined at a debug breakpoint. */
static uint32_t status_reg = 0;


/* Delay between frames, in UWB microseconds. See NOTE 1 below. */
#define POLL_TX_TO_RESP_RX_DLY_UUS 240
/* Receive response timeout. See NOTE 5 below. */
#define RESP_RX_TIMEOUT_UUS 400


/* Hold copies of computed time of flight and distance here for reference so that it can be examined at a debug breakpoint. */
static double tof;
static double distance;

/* BLE Connection Tracking */
static struct bt_conn *conn_connected;

/* GATT Service for Distance */
#define DISTANCE_SERVICE_UUID BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x567812345678)
#define DISTANCE_CHAR_UUID    BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x567812345679)

static void distance_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    LOG_INF("Distance CCCD changed: %s (value=0x%04x)", value == BT_GATT_CCC_NOTIFY ? "Subscribed" : "Unsubscribed", value);
}

static ssize_t gatt_write(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                          const void *buf, uint16_t len, uint16_t offset, uint8_t flags)
{
    LOG_INF("GATT write: attr=%p, len=%d, offset=%d, flags=0x%02x", attr, len, offset, flags);
    if (attr->uuid->type == BT_UUID_TYPE_128) {
        char uuid_str[37];
        bt_uuid_to_str(attr->uuid, uuid_str, sizeof(uuid_str));
        LOG_INF("Write to UUID: %s", uuid_str);
    }
    if (len >= 2) {
        uint16_t value = ((uint8_t *)buf)[0] | (((uint8_t *)buf)[1] << 8);
        LOG_INF("Written value: 0x%04x", value);
    }
    return bt_gatt_attr_write(conn, attr, buf, len, offset, flags);
}

BT_GATT_SERVICE_DEFINE(distance_svc,
    BT_GATT_PRIMARY_SERVICE(BT_UUID_DECLARE_128(DISTANCE_SERVICE_UUID)),
    BT_GATT_CHARACTERISTIC(BT_UUID_DECLARE_128(DISTANCE_CHAR_UUID),
                           BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                           BT_GATT_PERM_READ,
                           NULL, NULL, NULL),
    BT_GATT_CCC(distance_ccc_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
);

/* Advertising Data */
static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA_BYTES(BT_DATA_UUID128_ALL, DISTANCE_SERVICE_UUID),
};

static void connected(struct bt_conn *conn, uint8_t err)
{
    if (err) {
        LOG_ERR("Connection failed (err 0x%02x)", err);
    } else {
        conn_connected = bt_conn_ref(conn);
        LOG_INF("Connected");
    }
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
    LOG_INF("Disconnected (reason 0x%02x)", reason);
    if (conn_connected) {
        bt_conn_unref(conn_connected);
        conn_connected = NULL;
    }
}

static void mtu_updated(struct bt_conn *conn, uint16_t tx, uint16_t rx)
{
    LOG_INF("MTU updated: TX: %d RX: %d bytes", tx, rx);
}

static struct bt_gatt_cb gatt_callbacks = {
    .att_mtu_updated = mtu_updated,
};

BT_CONN_CB_DEFINE(conn_callbacks) = {
    .connected = connected,
    .disconnected = disconnected,
};

static void bt_ready(void)
{
    int err;
    struct bt_le_adv_param adv_param = {
        .id = BT_ID_DEFAULT,
        .interval_min = BT_GAP_ADV_FAST_INT_MIN_2,
        .interval_max = BT_GAP_ADV_FAST_INT_MAX_2,
        .options = BT_LE_ADV_OPT_CONNECTABLE | BT_LE_ADV_OPT_USE_NAME,
    };

    err = bt_le_adv_start(&adv_param, ad, ARRAY_SIZE(ad), NULL, 0);
    if (err) {
        LOG_ERR("Advertising failed to start (err %d)", err);
        return;
    }
    LOG_INF("Advertising started");
}


/* SPI Thread: Handles DWM3000 SPI reads and logging */
K_THREAD_STACK_DEFINE(spi_stack, 1024);
static struct k_thread spi_thread_data;

void spi_thread(void *arg1, void *arg2, void *arg3)
{
    struct dwm3000_context *ctx = (struct dwm3000_context *)arg1;
    int err;
    uint32_t dev_id;

    if (!ctx) {
        LOG_ERR("[AtlasTag] Invalid context, ctx is NULL");
        return;
    }

    LOG_INF("[AtlasTag] Initializing: RX_ANT_DLY=%u, TX_ANT_DLY=%u, POLL_TX_TO_RESP_RX_DLY_UUS=%u, RESP_RX_TIMEOUT_UUS=%u",
            RX_ANT_DLY, TX_ANT_DLY, POLL_TX_TO_RESP_RX_DLY_UUS, RESP_RX_TIMEOUT_UUS);

    dwt_setrxantennadelay(ctx, RX_ANT_DLY);
    dwt_settxantennadelay(ctx, TX_ANT_DLY);

    dwt_setrxaftertxdelay(ctx, POLL_TX_TO_RESP_RX_DLY_UUS);
    dwt_setrxtimeout(ctx, RESP_RX_TIMEOUT_UUS);

    dwt_setlnapamode(ctx, DWT_LNA_ENABLE | DWT_PA_ENABLE);

    while (1) {

        LOG_INF("[AtlasTag] Sending poll, frame_seq_nb=%u", frame_seq_nb);
        tx_poll_msg[ALL_MSG_SN_IDX] = frame_seq_nb;

        dwt_writesysstatuslo(ctx, SYS_STATUS_TXFRS_BIT_MASK);
        dwt_writetxdata(ctx, sizeof(tx_poll_msg), tx_poll_msg, 0);
        dwt_writetxfctrl(ctx, sizeof(tx_poll_msg), 0, 1); 

        LOG_INF("[AtlasTag] Starting TX (immediate, response expected)");
        dwt_starttx(ctx, DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);

        LOG_INF("[AtlasTag] Waiting for SYS_STATUS");
        waitforsysstatus(ctx, &status_reg, NULL, (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR), 0);

        frame_seq_nb++;
        LOG_INF("[Atlas] SYS_STATUS after wait: 0x%08x", status_reg);
        if (status_reg & SYS_STATUS_RXFCG_BIT_MASK) {
            uint16_t frame_len;

            LOG_INF("[AtlasTag] RX frame received, clearing SYS_STATUS");
            dwt_writesysstatuslo(ctx, SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_TXFRS_BIT_MASK);

            frame_len = dwt_getframelength(ctx);
            LOG_INF("[AtlasTag] Frame length=%u, RX_BUF_LEN=%u", frame_len, RX_BUF_LEN);
            if (frame_len <= RX_BUF_LEN) {
                dwt_readrxdata(ctx, rx_buffer, frame_len, 0);

                rx_buffer[ALL_MSG_SN_IDX] = 0;

                if (memcmp(rx_buffer, rx_resp_msg, ALL_MSG_COMMON_LEN) == 0) {
                    uint32_t poll_tx_ts, resp_rx_ts, poll_rx_ts, resp_tx_ts;
                    int32_t rtd_init, rtd_resp;
                    float clockOffsetRatio;
    
    
                    LOG_INF("[AtlasTag] Valid response message received");
                    poll_tx_ts = dwt_readtxtimestamplo32(ctx);
                    resp_rx_ts = dwt_readrxtimestamplo32(ctx);
                   
                    clockOffsetRatio = ((float)dwt_readclockoffset(ctx)) / (uint32_t)(1 << 26);

                    /* Get timestamps embedded in response message. */
                    resp_msg_get_ts(&rx_buffer[RESP_MSG_POLL_RX_TS_IDX], &poll_rx_ts);
                    resp_msg_get_ts(&rx_buffer[RESP_MSG_RESP_TX_TS_IDX], &resp_tx_ts);

                    /* Compute time of flight and distance, using clock offset ratio to correct for differing local and remote clock rates */
                    rtd_init = resp_rx_ts - poll_tx_ts;
                    rtd_resp = resp_tx_ts - poll_rx_ts;

                    tof = ((rtd_init - rtd_resp * (1 - clockOffsetRatio)) / 2.0) * DWT_TIME_UNITS;
                    distance = tof * SPEED_OF_LIGHT;
                    /* Display computed distance on LCD. */
                    LOG_INF("DIST: %3.2f m", distance);

                }
            }
        } else {
            LOG_ERR("[AtlasTag] Emmm, damn");
            dwt_writesysstatuslo(ctx, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
        }

        /* Execute a delay between ranging exchanges. */
        k_msleep(RNG_DELAY_MS);
    }
}

/* BLE/USB Thread: Handles BLE notifications and USB logging */
K_THREAD_STACK_DEFINE(ble_usb_stack, 1024);
static struct k_thread ble_usb_thread_data;

void ble_usb_thread(void *arg1, void *arg2, void *arg3)
{
    struct bt_conn *conn = NULL;
    static float distance = 1.0f;
    int err;

    while (1) {
        if (conn_connected) {
            conn = bt_conn_ref(conn_connected);

            LOG_INF("Current distance: %.2f meters", (double)distance);
            LOG_HEXDUMP_INF(&distance, sizeof(distance), "Distance bytes");
            err = bt_gatt_notify(conn, &distance_svc.attrs[1], &distance, sizeof(distance));
            if (err) {
                LOG_ERR("Notify failed (err %d)", err);
            } else {
                LOG_INF("Notified distance: %.2f meters", (double)distance);
            }

            distance += 1.0f;
            if (distance > 100.0f) {
                distance = 1.0f;
            }

            bt_conn_unref(conn);
            conn = NULL;
        } else {
            LOG_INF("No BLE connection");
        }

        k_sleep(K_SECONDS(5));
    }
}

void main(void)
{
    int err;
    LOG_INF("Starting MDBT50-DB-33...");

    const struct device *console_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
    uint32_t dtr = 0;
    err = usb_enable(NULL);
    if (err) {
        LOG_ERR("USB init failed (err %d)", err);
        return;
    }
    while (!dtr) {
        uart_line_ctrl_get(console_dev, UART_LINE_CTRL_DTR, &dtr);
        k_sleep(K_MSEC(100));
    }

    err = bt_enable(NULL);
    if (err) {
        LOG_ERR("Bluetooth init failed (err %d)", err);
        return;
    }
    LOG_INF("Bluetooth initialized");

    bt_gatt_cb_register(&gatt_callbacks);
    bt_ready();

    dwm3000_cfg.spi_dev = device_get_binding("SPI_0");
    if (!dwm3000_cfg.spi_dev) {
        LOG_ERR("SPI device SPI_0 not found");
        return;
    }

    dwm3000_cfg.gpio_dev = DEVICE_DT_GET(DT_NODELABEL(gpio0));
    if (!device_is_ready(dwm3000_cfg.gpio_dev)) {
        LOG_ERR("GPIO0 device not ready");
        return;
    }

    err = dwm3000_init(&dwm3000_ctx, &dwm3000_cfg);
    if (err) {
        LOG_ERR("DWM3000 init failed: %d", err);
        return;
    }

    err = port_set_dw_ic_spi_fastrate(&dwm3000_ctx);
    if (err) {
        LOG_ERR("Retry hardware reset failed: %d", err);
    }


    LOG_INF("Retrying hardware reset");
    err = true_reset_DWIC(&dwm3000_ctx);
    if (err) {
        LOG_ERR("Retry hardware reset failed: %d", err);
    }

    k_sleep(K_MSEC(2));

    
    while (!new_dwt_checkidlerc(&dwm3000_ctx)) { };
    LOG_INF("CHEQUEADOOOOO");
    
    if (dwt_initialise(&dwm3000_ctx, DWT_DW_INIT) == DWT_ERROR) {
        LOG_ERR("INIT FAILED");
        while (1) {  };
    }
    LOG_INF("INICIALIZADOOOO");

    
    if (dwt_configure(&dwm3000_ctx, &config))  {
        LOG_ERR("CONFIG FAILED");
        while (1) {
        LOG_ERR("Fallo migente");
        k_sleep(K_SECONDS(1));
        };
    }

    LOG_INF("CONFIGURADO");
    
    dwt_configuretxrf(&dwm3000_ctx, &txconfig_options);
    
    LOG_INF("CONFIGURADO 2!");

    uint32_t dev_id;
    err = dwm3000_read_dev_id(&dwm3000_ctx, &dev_id);
    if (err) {
        LOG_ERR("DWM3000 read device ID failed: %d", err);
    } else {
        LOG_INF("Device ID: 0x%08x", dev_id);
    }

    LOG_INF("Sending started");

    LOG_INF("Starting BLE/USB thread");
    k_thread_create(&ble_usb_thread_data, ble_usb_stack,
                    K_THREAD_STACK_SIZEOF(ble_usb_stack),
                    ble_usb_thread, NULL, NULL, NULL,
                    5, 0, K_NO_WAIT);

    LOG_INF("Starting SPI thread");
    k_thread_create(&spi_thread_data, spi_stack,
                    K_THREAD_STACK_SIZEOF(spi_stack),
                    spi_thread, &dwm3000_ctx, NULL, NULL,
                    3, 0, K_NO_WAIT);

    while (1) {
        k_sleep(K_FOREVER);
    }
}