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

static uint8_t tx_msg[] = {0xC5, 0, 'D', 'E', 'C', 'A', 'W', 'A', 'V', 'E'};

/* Index to access to sequence number of the blink frame in the tx_msg array. */
#define BLINK_FRAME_SN_IDX 1

#define FRAME_LENGTH    (sizeof(tx_msg)+FCS_LEN) //The real length that is going to be transmitted


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


/* Frames used in the ranging process. See NOTE 2 below. */
static uint8_t tx_poll_msg[] = { 0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0x21 };
static uint8_t rx_resp_msg[] = { 0x41, 0x88, 0, 0xCA, 0xDE, 'V', 'E', 'W', 'A', 0x10, 0x02, 0, 0 };
static uint8_t tx_final_msg[] = { 0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0x23, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

/* Length of the common part of the message (up to and including the function code, see NOTE 2 below). */
#define ALL_MSG_COMMON_LEN 10
/* Indexes to access some of the fields in the frames defined above. */
#define ALL_MSG_SN_IDX            2
#define FINAL_MSG_POLL_TX_TS_IDX  10
#define FINAL_MSG_RESP_RX_TS_IDX  14
#define FINAL_MSG_FINAL_TX_TS_IDX 18

/* Frame sequence number, incremented after each transmission. */
static uint8_t frame_seq_nb = 0;

/* Buffer to store received response message.
 * Its size is adjusted to longest frame that this example code is supposed to handle. */
#define RX_BUF_LEN 24
static uint8_t rx_buffer[RX_BUF_LEN];

/* Hold copy of status register state here for reference so that it can be examined at a debug breakpoint. */
static uint32_t status_reg = 0;


/* Delay between frames, in UWB microseconds. See NOTE 4 below. */
/* This is the delay from the end of the frame transmission to the enable of the receiver, as programmed for the DW IC's wait for response feature. */
#define CPU_PROCESSING_TIME 600
#define POLL_TX_TO_RESP_RX_DLY_UUS (300 + CPU_PROCESSING_TIME)
/* This is the delay from Frame RX timestamp to TX reply timestamp used for calculating/setting the DW IC's delayed TX function.
 * This value is required to be larger than POLL_TX_TO_RESP_RX_DLY_UUS. Please see NOTE 4 for more details. */
#define RESP_RX_TO_FINAL_TX_DLY_UUS (300 + CPU_PROCESSING_TIME)
/* Receive response timeout. See NOTE 5 below. */
#define RESP_RX_TIMEOUT_UUS 900
/* Preamble timeout, in multiple of PAC size. See NOTE 7 below. */
#define PRE_TIMEOUT 5
/* Time-stamps of frames transmission/reception, expressed in device time units. */
static uint64_t poll_tx_ts;
static uint64_t resp_rx_ts;
static uint64_t final_tx_ts;


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
    dwt_setpreambledetecttimeout(ctx, PRE_TIMEOUT);

    dwt_setlnapamode(ctx, DWT_LNA_ENABLE | DWT_PA_ENABLE);

    while (1) {
        LOG_INF("[AtlasTag] Sending poll, frame_seq_nb=%u", frame_seq_nb);
        tx_poll_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
        dwt_writetxdata(ctx, sizeof(tx_poll_msg), tx_poll_msg, 0);
        dwt_writetxfctrl(ctx, sizeof(tx_poll_msg) + FCS_LEN, 0, 1);

        LOG_INF("[AtlasTag] Starting TX (immediate, response expected)");
        dwt_starttx(ctx, DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);

        LOG_INF("[AtlasTag] Waiting for SYS_STATUS");
        waitforsysstatus(ctx, &status_reg, NULL, (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR), 0);

        frame_seq_nb++;

        if (status_reg & SYS_STATUS_RXFCG_BIT_MASK) {
            uint16_t frame_len;

            LOG_INF("[AtlasTag] RX frame received, clearing SYS_STATUS");
            dwt_writesysstatuslo(ctx, SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_TXFRS_BIT_MASK);

            frame_len = dwt_getframelength(ctx);
            LOG_INF("[AtlasTag] Frame length=%u, RX_BUF_LEN=%u", frame_len, RX_BUF_LEN);
            if (frame_len <= RX_BUF_LEN) {
                dwt_readrxdata(ctx, rx_buffer, frame_len, 0);
            } else {
                LOG_ERR("[AtlasTag] Frame length %u exceeds RX_BUF_LEN %u", frame_len, RX_BUF_LEN);
            }

            rx_buffer[ALL_MSG_SN_IDX] = 0;
            if (memcmp(rx_buffer, rx_resp_msg, ALL_MSG_COMMON_LEN) == 0) {
                uint32_t final_tx_time;
                int ret;

                LOG_INF("[AtlasTag] Valid response message received");
                poll_tx_ts = get_tx_timestamp_u64(ctx);
                resp_rx_ts = get_rx_timestamp_u64(ctx);
                LOG_INF("[AtlasTag] poll_tx_ts=0x%016llx, resp_rx_ts=0x%016llx", poll_tx_ts, resp_rx_ts);

                final_tx_time = (resp_rx_ts + (RESP_RX_TO_FINAL_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;
                LOG_INF("[AtlasTag] final_tx_time=0x%08x", final_tx_time);
                dwt_setdelayedtrxtime(ctx, final_tx_time);

                final_tx_ts = (((uint64_t)(final_tx_time & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY;

                final_msg_set_ts(&tx_final_msg[FINAL_MSG_POLL_TX_TS_IDX], poll_tx_ts);
                final_msg_set_ts(&tx_final_msg[FINAL_MSG_RESP_RX_TS_IDX], resp_rx_ts);
                final_msg_set_ts(&tx_final_msg[FINAL_MSG_FINAL_TX_TS_IDX], final_tx_ts);

                tx_final_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
                dwt_writetxdata(ctx, sizeof(tx_final_msg), tx_final_msg, 0);
                dwt_writetxfctrl(ctx, sizeof(tx_final_msg) + FCS_LEN, 0, 1);

                LOG_INF("[AtlasTag] Starting TX (delayed) for final message");
                ret = dwt_starttx(ctx, DWT_START_TX_DELAYED);
                if (ret == DWT_SUCCESS) {
                    LOG_INF("[AtlasTag] Waiting for TXFRS");
                    waitforsysstatus(ctx, NULL, NULL, SYS_STATUS_TXFRS_BIT_MASK, 0);

                    LOG_INF("[AtlasTag] Final TX complete, clearing SYS_STATUS");
                    dwt_writesysstatuslo(ctx, SYS_STATUS_TXFRS_BIT_MASK);

                    frame_seq_nb++;
                } else {
                    LOG_ERR("[AtlasTag] dwt_starttx failed for final message, ret=%d", ret);
                }
            } else {
                LOG_ERR("[AtlasTag] Invalid response message, validation failed, rx_buffer=%02x %02x %02x %02x",
                        rx_buffer[0], rx_buffer[1], rx_buffer[2], rx_buffer[3]);
            }
        } else {
            LOG_ERR("[AtlasTag] No good RX frame, SYS_STATUS=0x%08x, RXPRD=%d, RXSFDD=%d, RXPTO=%d, RXSFDTO=%d",
                    status_reg,
                    (status_reg & 0x00008000) ? 1 : 0, // RXPRD (bit 15)
                    (status_reg & 0x00020000) ? 1 : 0, // RXSFDD (bit 17)
                    (status_reg & 0x01000000) ? 1 : 0, // RXPTO (bit 24)
                    (status_reg & 0x02000000) ? 1 : 0); // RXSFDTO (bit 25)
            dwt_writesysstatuslo(ctx, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR | SYS_STATUS_TXFRS_BIT_MASK);
        }

#if SLEEP_EN
        dwt_entersleep(ctx, DWT_DW_IDLE);
        k_sleep(K_SECONDS(1));
        dwt_wakeup_ic();
        k_sleep(K_SECONDS(2));
        while (!dwt_checkidlerc(ctx)) { };
        dwt_restoreconfig(ctx);
#else
        k_sleep(K_SECONDS(1));
#endif
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