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

static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA_BYTES(BT_DATA_UUID16_ALL, BT_UUID_16_ENCODE(BT_UUID_DIS_VAL)),
};

static void connected(struct bt_conn *conn, uint8_t err)
{
    if (err) {
        LOG_ERR("Connection failed (err 0x%02x)", err);
    } else {
        LOG_INF("Connected");
    }
}

static void disconnected(struct bt_conn *conn, uint8_t reason)

{
    LOG_INF("Disconnected (reason 0x%02x)", reason);
}

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

    LOG_INF("Configuring SPI to 8 MHz");
    err = port_set_dw_ic_spi_fastrate(&dwm3000_ctx);
    if (err) {
        LOG_ERR("SPI 8 MHz config failed: %d", err);
    }

    while (1) {
        err = dwm3000_read_dev_id(&dwm3000_ctx, &dev_id);
        if (err) {
            LOG_ERR("Device ID read failed: %d", err);
        } else {
            LOG_INF("Device ID: 0x%08x", dev_id);
        }

        LOG_INF("Looping, still alive...");
        k_sleep(K_SECONDS(1));
    }
}
