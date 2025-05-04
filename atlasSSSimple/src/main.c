/*
 * Copyright (c) 2025 Your Name
 * SPDX-License-Identifier: Apache-2.0
 */
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/drivers/uart.h>
#include <dwm3000.h>
#include <stdint.h>
#include <stddef.h>

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
    0x0         /* PG count */
};

/* Default communication configuration. We use default non-STS DW mode. */
static dwt_config_t config = {
    .chan            = 5,               /* Channel number. */
    .txPreambLength  = DWT_PLEN_128,    /* Preamble length. Used in TX only. */
    .rxPAC           = DWT_PAC8,        /* Preamble acquisition chunk size. Used in RX only. */
    .txCode          = 9,               /* TX preamble code. Used in TX only. */
    .rxCode          = 9,               /* RX preamble code. Used in RX only. */
    .sfdType         = 0,               /* 0 to use standard 8 symbol SFD */
    .dataRate        = DWT_BR_6M8,      /* Data rate. */
    .phrMode         = DWT_PHRMODE_STD, /* PHY header mode. */
    .phrRate         = DWT_PHRRATE_STD, /* PHY header rate. */
    .sfdTO           = (129 + 8 - 8),   /* SFD timeout */
    .stsMode         = DWT_STS_MODE_OFF,
    .stsLength       = DWT_STS_LEN_64,  /* STS length */
    .pdoaMode        = DWT_PDOA_M0      /* PDOA mode off */
};

/* Default antenna delay values for 64 MHz PRF. */
#define TX_ANT_DLY 16385
#define RX_ANT_DLY 16385

/* Frames used in the ranging process. */
static uint8_t rx_poll_msg[] = { 0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0xE0, 0, 0 };
static uint8_t tx_resp_msg[] = { 0x41, 0x88, 0, 0xCA, 0xDE, 'V', 'E', 'W', 'A', 0xE1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

/* Length of the common part of the message (up to and including the function code). */
#define ALL_MSG_COMMON_LEN 10
/* Index to access some of the fields in the frames involved in the process. */
#define ALL_MSG_SN_IDX          2
#define RESP_MSG_POLL_RX_TS_IDX 10
#define RESP_MSG_RESP_TX_TS_IDX 14
#define RESP_MSG_TS_LEN         4

/* Frame sequence number, incremented after each transmission. */
static uint8_t frame_seq_nb = 0;

/* Buffer to store received response message. */
#define RX_BUF_LEN 12
static uint8_t rx_buffer[RX_BUF_LEN];

/* Hold copy of status register state for debugging. */
static uint32_t status_reg = 0;

/* Delay between frames, in UWB microseconds. */
#define POLL_RX_TO_RESP_TX_DLY_UUS 240

/* Timestamps of frames transmission/reception. */
static uint64_t poll_rx_ts;
static uint64_t resp_tx_ts;

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
        LOG_ERR("Set SPI fast rate failed: %d", err);
    }

    LOG_INF("Retrying hardware reset");
    err = true_reset_DWIC(&dwm3000_ctx);
    if (err) {
        LOG_ERR("Retry hardware reset failed: %d", err);
    }

    k_sleep(K_MSEC(2));

    while (!new_dwt_checkidlerc(&dwm3000_ctx)) { }
    LOG_INF("Idle check passed");

    if (dwt_initialise(&dwm3000_ctx, DWT_DW_INIT) == DWT_ERROR) {
        LOG_ERR("DWM3000 init failed");
        while (1) { }
    }
    LOG_INF("DWM3000 initialized");

    int retries = 3;
    while (retries--) {
        if (!dwt_configure(&dwm3000_ctx, &config)) {
            break;
        }
        LOG_ERR("CONFIG FAILED");
    }
    if (retries <= 0) {
        LOG_ERR("Configuration failed after retries");
        while (1) { k_sleep(K_SECONDS(1)); }
    }
    LOG_INF("DWM3000 configured");

    dwt_configuretxrf(&dwm3000_ctx, &txconfig_options);
    LOG_INF("TX RF configured");

    uint32_t dev_id;
    err = dwm3000_read_dev_id(&dwm3000_ctx, &dev_id);
    if (err) {
        LOG_ERR("DWM3000 read device ID failed: %d", err);
    } else {
        LOG_INF("Device ID: 0x%08x", dev_id);
    }

    dwt_setrxantennadelay(&dwm3000_ctx, RX_ANT_DLY);
    dwt_settxantennadelay(&dwm3000_ctx, TX_ANT_DLY);
    dwt_setlnapamode(&dwm3000_ctx, DWT_LNA_ENABLE | DWT_PA_ENABLE);
    LOG_INF("Starting UWB ranging");

    while (1) {
        //LOG_INF("Enabling RX immediate");
        dwt_rxenable(&dwm3000_ctx, DWT_START_RX_IMMEDIATE);

        //LOG_INF("Waiting for SYS_STATUS: expecting RXFCG or RX_ERR");
        waitforsysstatus(&dwm3000_ctx, &status_reg, NULL, (DWT_INT_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_ERR), 0);

        //LOG_INF("SYS_STATUS: 0x%08x", status_reg);
        if (status_reg & DWT_INT_RXFCG_BIT_MASK) {
            uint16_t frame_len;

            //LOG_INF("Good RX frame received");
            dwt_writesysstatuslo(&dwm3000_ctx, DWT_INT_RXFCG_BIT_MASK);
            //LOG_INF("Cleared SYS_STATUS_RXFCG_BIT_MASK");

            frame_len = dwt_getframelength(&dwm3000_ctx);
            //LOG_INF("Frame length: %u, RX buffer size: %u", frame_len, sizeof(rx_buffer));
            if (frame_len <= sizeof(rx_buffer)) {
                dwt_readrxdata(&dwm3000_ctx, rx_buffer, frame_len, 0);
                //LOG_HEXDUMP_INF(rx_buffer, frame_len, "Received frame");

                rx_buffer[ALL_MSG_SN_IDX] = 0;
                //LOG_INF("Checking if frame matches poll message");
                if (memcmp(rx_buffer, rx_poll_msg, ALL_MSG_COMMON_LEN) == 0) {
                    uint32_t resp_tx_time;
                    int ret;

                    //LOG_INF("Valid poll message received");
                    poll_rx_ts = get_rx_timestamp_u64(&dwm3000_ctx);
                    //LOG_INF("Poll RX timestamp: 0x%016llx", poll_rx_ts);

                    resp_tx_time = (poll_rx_ts + (POLL_RX_TO_RESP_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;
                    dwt_setdelayedtrxtime(&dwm3000_ctx, resp_tx_time);
                    //LOG_INF("Response TX time set: 0x%08x (delay=%u us)", resp_tx_time, POLL_RX_TO_RESP_TX_DLY_UUS);

                    resp_tx_ts = (((uint64_t)(resp_tx_time & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY;
                    //LOG_INF("Response TX timestamp: 0x%016llx", resp_tx_ts);

                    resp_msg_set_ts(&tx_resp_msg[RESP_MSG_POLL_RX_TS_IDX], poll_rx_ts);
                    resp_msg_set_ts(&tx_resp_msg[RESP_MSG_RESP_TX_TS_IDX], resp_tx_ts);
                    //LOG_INF("Timestamps written to response message");

                    tx_resp_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
                    dwt_writetxdata(&dwm3000_ctx, sizeof(tx_resp_msg), tx_resp_msg, 0);
                    dwt_writetxfctrl(&dwm3000_ctx, sizeof(tx_resp_msg), 0, 1);
                    //LOG_HEXDUMP_INF(tx_resp_msg, sizeof(tx_resp_msg), "Response message prepared");

                    //LOG_INF("Starting TX delayed");
                    ret = dwt_starttx(&dwm3000_ctx, DWT_START_TX_IMMEDIATE);
                    //LOG_INF("dwt_starttx result: %d", ret);

                    if (ret == DWT_SUCCESS) {
                        //LOG_INF("Waiting for TX frame sent");
                        waitforsysstatus(&dwm3000_ctx, NULL, NULL, DWT_INT_TXFRS_BIT_MASK, 0);

                        dwt_writesysstatuslo(&dwm3000_ctx, DWT_INT_TXFRS_BIT_MASK);
                        //LOG_INF("Cleared DWT_INT_TXFRS_BIT_MASK");

                        frame_seq_nb++;
                        //LOG_INF("Frame sequence number incremented: %u", frame_seq_nb);
                    } else {
                        //LOG_ERR("dwt_starttx failed, abandoning exchange");
                    }
                } else {
                    //LOG_ERR("Frame does not match expected poll message");
                }
            } else {
                //LOG_ERR("Frame length %u exceeds buffer size %u", frame_len, sizeof(rx_buffer));
            }
        } else {
            LOG_ERR("No good RX frame, clearing RX error events");
            dwt_writesysstatuslo(&dwm3000_ctx, SYS_STATUS_ALL_RX_ERR);
        }
    }
}