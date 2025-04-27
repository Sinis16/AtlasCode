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

/* Inter-frame delay period, in milliseconds. */
#define TX_DELAY_MS 500


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
    .sfdType         = DWT_SFD_DW_8,    /* 0 to use standard 8 symbol SFD */
    .dataRate        = DWT_BR_6M8,      /* Data rate. */
    .phrMode         = DWT_PHRMODE_STD, /* PHY header mode. */
    .phrRate         = DWT_PHRRATE_STD, /* PHY header rate. */
    .sfdTO           = (129 + 8 - 8),   /* SFD timeout */
    .stsMode         = DWT_STS_MODE_OFF,
    .stsLength       = DWT_STS_LEN_64,  /* STS length, see allowed values in Enum dwt_sts_lengths_e */
    .pdoaMode        = DWT_PDOA_M0      /* PDOA mode off */
};


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

    while (1) {
        err = dwm3000_read_dev_id(ctx, &dev_id);
        if (err) {
            LOG_ERR("Device ID read failed: %d", err);
        } else {
            LOG_INF("Device ID: 0x%08x", dev_id);
        }

        LOG_INF("SPI thread looping...");
        k_sleep(K_SECONDS(1));
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
    k_sleep(K_MSEC(10));

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
    

    uint32_t dev_id;
    err = dwm3000_read_dev_id(&dwm3000_ctx, &dev_id);
    if (err) {
        LOG_ERR("DWM3000 read device ID failed: %d", err);
    } else {
        LOG_INF("Device ID: 0x%08x", dev_id);
    }

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