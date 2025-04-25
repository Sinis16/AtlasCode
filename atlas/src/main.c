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
    .spi_dev = NULL, // Set at runtime
    .gpio_dev = NULL, // Set at runtime
    .cs_pin = 5, // P0.05
    .reset_pin = 28, // P0.28
    .wakeup_pin = 29, // P0.29
    .irq_pin = 3, // P0.03
    .spi_cfg = {
        .frequency = 2000000,
        .operation = SPI_WORD_SET(8) | SPI_OP_MODE_MASTER,
        .slave = 0,
        .cs = NULL,
    },
};

/* BLE Connection Tracking */
static struct bt_conn *conn_connected;

/* GATT Service for Distance */
#define DISTANCE_SERVICE_UUID BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x567812345678)
#define DISTANCE_CHAR_UUID    BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x567812345679)

static void distance_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    LOG_INF("Distance CCCD changed: %s", value == BT_GATT_CCC_NOTIFY ? "Subscribed" : "Unsubscribed");
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

    err = bt_le_adv_start(BT_LE_ADV_CONN_NAME, ad, ARRAY_SIZE(ad), NULL, 0);
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
    int irq_state;

    /* Configure SPI to 8 MHz */
    LOG_INF("Configuring SPI to 8 MHz");
    err = port_set_dw_ic_spi_fastrate(ctx);
    if (err) {
        LOG_ERR("SPI 8 MHz config failed: %d", err);
        return;
    }

    while (1) {
        /* Read Device ID (placeholder for DS-TWR) */
        err = dwm3000_read_dev_id(ctx, &dev_id);
        if (err) {
            LOG_ERR("Device ID read failed: %d", err);
        } else {
            LOG_INF("Device ID: 0x%08x", dev_id);
        }

        /* Read IRQ state */
        err = dwm3000_get_irq_state(ctx, &irq_state);
        if (err) {
            LOG_ERR("IRQ state read failed: %d", err);
        } else {
            LOG_INF("IRQ state: %d", irq_state);
        }

        LOG_INF("SPI thread looping...");
        k_sleep(K_SECONDS(1)); // Adjust for DS-TWR (e.g., K_MSEC(10))
    }
}

/* BLE/USB Thread: Handles BLE notifications and USB logging */
K_THREAD_STACK_DEFINE(ble_usb_stack, 1024);
static struct k_thread ble_usb_thread_data;

void ble_usb_thread(void *arg1, void *arg2, void *arg3)
{
    struct bt_conn *conn = NULL;
    static float distance = 1.0f; // Start at 1.0 meters
    int err;

    while (1) {
        if (conn_connected) {
            /* Get connection reference */
            conn = bt_conn_ref(conn_connected);

            /* Send notification */
            err = bt_gatt_notify(conn, &distance_svc.attrs[1], &distance, sizeof(distance));
            if (err) {
                LOG_ERR("Notify failed (err %d)", err);
            } else {
                LOG_INF("Notified distance: %.2f meters", distance);
            }

            /* Increment distance, reset to 1.0 after 100.0 */
            distance += 1.0f;
            if (distance > 100.0f) {
                distance = 1.0f;
            }

            bt_conn_unref(conn);
            conn = NULL;

            LOG_INF("Current distance: %.2f meters", distance);
        } else {
            LOG_INF("No BLE connection");
        }

        

        k_sleep(K_SECONDS(5)); // Notify every 5 seconds
    }
}

void main(void)
{
    int err;
    LOG_INF("Starting MDBT50-DB-33...");

    // Initialize USB
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

    // Initialize Bluetooth
    err = bt_enable(NULL);
    if (err) {
        LOG_ERR("Bluetooth init failed (err %d)", err);
        return;
    }
    LOG_INF("Bluetooth initialized");

    bt_gatt_cb_register(&gatt_callbacks);
    bt_ready();

    // Initialize DWM3000
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
    LOG_INF("Performing hardware reset");
    err = reset_DWIC(&dwm3000_ctx);
    if (err) {
        LOG_ERR("Hardware reset failed: %d", err);
        return;
    }
    k_sleep(K_MSEC(10));

    LOG_INF("Configuring SPI to 2 MHz");
    err = port_set_dw_ic_spi_slowrate(&dwm3000_ctx);
    if (err) {
        LOG_ERR("SPI 2 MHz config failed: %d", err);
        return;
    }

    LOG_INF("Clearing AON config");
    err = dwt_clearaonconfig(&dwm3000_ctx);
    if (err) {
        LOG_ERR("Clear AON config failed: %d", err);
    }
    k_sleep(K_MSEC(10));

    LOG_INF("Performing soft reset");
    err = dwt_softreset(&dwm3000_ctx);
    if (err) {
        LOG_ERR("Soft reset failed: %d", err);
    }
    k_sleep(K_MSEC(10));

    LOG_INF("Retrying hardware reset");
    err = reset_DWIC(&dwm3000_ctx);
    if (err) {
        LOG_ERR("Retry hardware reset failed: %d", err);
    }
    k_sleep(K_MSEC(10));

    LOG_INF("Reading CLK_CTRL");
    uint8_t tx_buf[5] = {CLK_CTRL_ID, 0x00, 0x00, 0x00, 0x00};
    uint8_t rx_buf[5] = {0};
    err = dwm3000_spi_transceive(&dwm3000_ctx, tx_buf, rx_buf, sizeof(tx_buf));
    if (err) {
        LOG_ERR("CLK_CTRL read failed: %d", err);
    } else {
        uint32_t clk_ctrl = (rx_buf[4] << 24) | (rx_buf[3] << 16) | (rx_buf[2] << 8) | rx_buf[1];
        LOG_INF("CLK_CTRL: 0x%08x", clk_ctrl);
    }

    LOG_INF("Reading SYS_STATE");
    tx_buf[0] = 0x01; // SYS_STATE
    err = dwm3000_spi_transceive(&dwm3000_ctx, tx_buf, rx_buf, sizeof(tx_buf));
    if (err) {
        LOG_ERR("SYS_STATE read failed: %d", err);
    } else {
        uint32_t sys_state = (rx_buf[4] << 24) | (rx_buf[3] << 16) | (rx_buf[2] << 8) | rx_buf[1];
        LOG_INF("SYS_STATE: 0x%08x", sys_state);
    }

    LOG_INF("Reading SYS_STATUS before IDLE_RC check");
    tx_buf[0] = SYS_STATUS_ID;
    err = dwm3000_spi_transceive(&dwm3000_ctx, tx_buf, rx_buf, sizeof(tx_buf));
    if (err) {
        LOG_ERR("SYS_STATUS read failed: %d", err);
    } else {
        uint32_t status = (rx_buf[4] << 24) | (rx_buf[3] << 16) | (rx_buf[2] << 8) | rx_buf[1];
        LOG_INF("SYS_STATUS: 0x%08x", status);
    }

    LOG_INF("Checking IDLE_RC state");
    err = dwt_checkidlerc(&dwm3000_ctx);
    if (err) {
        LOG_ERR("DWM3000 not in IDLE_RC state: %d, SYS_STATUS: 0x%08x", err, dwm3000_ctx.last_sys_status);
    } else {
        LOG_INF("DWM3000 in IDLE_RC state");
    }

    LOG_INF("Reading SYS_STATUS after IDLE_RC check");
    err = dwm3000_spi_transceive(&dwm3000_ctx, tx_buf, rx_buf, sizeof(tx_buf));
    if (err) {
        LOG_ERR("SYS_STATUS read failed: %d", err);
    } else {
        uint32_t status = (rx_buf[4] << 24) | (rx_buf[3] << 16) | (rx_buf[2] << 8) | rx_buf[1];
        LOG_INF("SYS_STATUS: 0x%08x", status);
    }

    uint32_t dev_id;
    err = dwm3000_read_dev_id(&dwm3000_ctx, &dev_id);
    if (err) {
        LOG_ERR("DWM3000 read device ID failed: %d", err);
    } else {
        LOG_INF("Device ID: 0x%08x", dev_id);
    }

    int irq_state;
    err = dwm3000_get_irq_state(&dwm3000_ctx, &irq_state);
    if (err) {
        LOG_ERR("IRQ state read failed: %d", err);
    } else {
        LOG_INF("IRQ state: %d", irq_state);
    }

    /* Start threads */
    LOG_INF("Starting BLE/USB thread");
    k_thread_create(&ble_usb_thread_data, ble_usb_stack,
                    K_THREAD_STACK_SIZEOF(ble_usb_stack),
                    ble_usb_thread, NULL, NULL, NULL,
                    5, 0, K_NO_WAIT); // Priority 5

    LOG_INF("Starting SPI thread");
    k_thread_create(&spi_thread_data, spi_stack,
                    K_THREAD_STACK_SIZEOF(spi_stack),
                    spi_thread, &dwm3000_ctx, NULL, NULL,
                    3, 0, K_NO_WAIT); // Priority 3

    /* Main thread becomes idle */
    while (1) {
        k_sleep(K_FOREVER); // Idle, threads handle tasks
    }
}