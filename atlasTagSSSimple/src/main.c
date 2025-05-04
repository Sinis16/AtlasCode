/*
 * Copyright (c) 2025 Your Name
 * SPDX-License-Identifier: Apache-2.0
 */
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/drivers/uart.h>
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

/* Inter-ranging delay period, in milliseconds. */
#define RNG_DELAY_MS 1000

/* Default antenna delay values for 64 MHz PRF. */
#define TX_ANT_DLY 16385
#define RX_ANT_DLY 16385

/* Frames used in the ranging process. */
static uint8_t tx_poll_msg[] = { 0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0xE0, 0, 0 };
static uint8_t rx_resp_msg[] = { 0x41, 0x88, 0, 0xCA, 0xDE, 'V', 'E', 'W', 'A', 0xE1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

/* Length of the common part of the message (up to and including the function code). */
#define ALL_MSG_COMMON_LEN 10
/* Indexes to access some of the fields in the frames. */
#define ALL_MSG_SN_IDX          2
#define RESP_MSG_POLL_RX_TS_IDX 10
#define RESP_MSG_RESP_TX_TS_IDX 14
#define RESP_MSG_TS_LEN         4

/* Frame sequence number, incremented after each transmission. */
static uint8_t frame_seq_nb = 0;

/* Buffer to store received response message. */
#define RX_BUF_LEN 20
static uint8_t rx_buffer[RX_BUF_LEN];

/* Hold copy of status register state for debugging. */
static uint32_t status_reg = 0;

/* Delay between frames, in UWB microseconds. */
#define POLL_TX_TO_RESP_RX_DLY_UUS 240
/* Receive response timeout. */
#define RESP_RX_TIMEOUT_UUS 300000

/* Hold copies of computed time of flight and distance for debugging. */
static double tof;
static double distance;

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

    if (dwt_configure(&dwm3000_ctx, &config)) {
        LOG_ERR("DWM3000 config failed");
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
    dwt_setrxaftertxdelay(&dwm3000_ctx, POLL_TX_TO_RESP_RX_DLY_UUS);
    dwt_setrxtimeout(&dwm3000_ctx, RESP_RX_TIMEOUT_UUS);
    dwt_setlnapamode(&dwm3000_ctx, DWT_LNA_ENABLE | DWT_PA_ENABLE);

    LOG_INF("Starting UWB ranging");

    while (1) {
        //LOG_INF("Sending poll, frame_seq_nb=%u", frame_seq_nb);
        tx_poll_msg[ALL_MSG_SN_IDX] = frame_seq_nb;

        dwt_writesysstatuslo(&dwm3000_ctx, SYS_STATUS_TXFRS_BIT_MASK);
        dwt_writetxdata(&dwm3000_ctx, sizeof(tx_poll_msg), tx_poll_msg, 0);
        dwt_writetxfctrl(&dwm3000_ctx, sizeof(tx_poll_msg), 0, 1);

        //LOG_INF("Starting TX (immediate, response expected)");
        dwt_starttx(&dwm3000_ctx, DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);

        //LOG_INF("Waiting for SYS_STATUS");
        waitforsysstatus(&dwm3000_ctx, &status_reg, NULL, (DWT_INT_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR), 0);

        frame_seq_nb++;
        //LOG_INF("SYS_STATUS: 0x%08x", status_reg);
        if (status_reg & DWT_INT_RXFCG_BIT_MASK) {
            uint16_t frame_len;

            //LOG_INF("RX frame received, clearing SYS_STATUS");
            dwt_writesysstatuslo(&dwm3000_ctx, DWT_INT_RXFCG_BIT_MASK);

            frame_len = dwt_getframelength(&dwm3000_ctx);
            //LOG_INF("Frame length=%u, RX_BUF_LEN=%u", frame_len, RX_BUF_LEN);
            if (frame_len <= sizeof(rx_buffer)) {
                dwt_readrxdata(&dwm3000_ctx, rx_buffer, frame_len, 0);

                rx_buffer[ALL_MSG_SN_IDX] = 0;

                if (memcmp(rx_buffer, rx_resp_msg, ALL_MSG_COMMON_LEN) == 0) {
                    uint32_t poll_tx_ts, resp_rx_ts, poll_rx_ts, resp_tx_ts;
                    int32_t rtd_init, rtd_resp;
                    float clockOffsetRatio;

                    //LOG_INF("Valid response message received");
                    poll_tx_ts = dwt_readtxtimestamplo32(&dwm3000_ctx);
                    resp_rx_ts = dwt_readrxtimestamplo32(&dwm3000_ctx);

                    clockOffsetRatio = ((float)dwt_readclockoffset(&dwm3000_ctx)) / (uint32_t)(1 << 26);

                    resp_msg_get_ts(&rx_buffer[RESP_MSG_POLL_RX_TS_IDX], &poll_rx_ts);
                    resp_msg_get_ts(&rx_buffer[RESP_MSG_RESP_TX_TS_IDX], &resp_tx_ts);

                    rtd_init = resp_rx_ts - poll_tx_ts;
                    rtd_resp = resp_tx_ts - poll_rx_ts;

                    tof = ((rtd_init - rtd_resp * (1 - clockOffsetRatio)) / 2.0) * DWT_TIME_UNITS;
                    distance = tof * SPEED_OF_LIGHT;

                    LOG_INF("ToF: %3.6f s", tof);
                    //LOG_INF("Distance: %3.4f m", distance);
                }
            }
        } else {
            //LOG_ERR("RX error or timeout");
            dwt_writesysstatuslo(&dwm3000_ctx, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
        }

        k_msleep(RNG_DELAY_MS);
    }
}