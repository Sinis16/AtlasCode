/*
 * Copyright (c) 2025 Your Name
 * SPDX-License-License-Identifier: Apache-2.0
 */

 #include <zephyr/kernel.h>
 #include <zephyr/drivers/spi.h>
 #include <zephyr/drivers/gpio.h>
 #include "dwm3000.h"
 
 
 static int dwm3000_spi_transceive(struct dwm3000_context *ctx, uint8_t *tx_buf, uint8_t *rx_buf, size_t len)
 {
     const struct dwm3000_config *cfg = ctx->config;
     struct spi_buf tx = {.buf = tx_buf, .len = len};
     struct spi_buf rx = {.buf = rx_buf, .len = len};
     struct spi_buf_set tx_set = {.buffers = &tx, .count = 1};
     struct spi_buf_set rx_set = {.buffers = &rx, .count = 1};
 
     int ret = gpio_pin_set(cfg->gpio_dev, cfg->cs_pin, 0); // CS low
     if (ret) {
         return ret;
     }
     k_sleep(K_MSEC(2)); // CS delay
 
     ret = spi_transceive(cfg->spi_dev, &cfg->spi_cfg, &tx_set, &rx_set);
     if (ret) {
         gpio_pin_set(cfg->gpio_dev, cfg->cs_pin, 1); // CS high
         return ret;
     }
 
     ret = gpio_pin_set(cfg->gpio_dev, cfg->cs_pin, 1); // CS high
     return ret;
 }

 /* Write 8-bit value to register with offset */
static int dwt_write8bitoffsetreg(struct dwm3000_context *ctx, uint16_t reg, uint16_t offset, uint8_t value)
{
    uint8_t tx_buf[3] = { (uint8_t)(reg & 0x7F) | 0x80, (uint8_t)offset, value };
    uint8_t rx_buf[3] = {0};

    return dwm3000_spi_transceive(ctx, tx_buf, rx_buf, 3);
}

/* Write 16-bit value to register with offset */
static int dwt_write16bitoffsetreg(struct dwm3000_context *ctx, uint16_t reg, uint16_t offset, uint16_t value)
{
    uint8_t tx_buf[4] = { (uint8_t)(reg & 0x7F) | 0x80, (uint8_t)offset, (uint8_t)value, (uint8_t)(value >> 8) };
    uint8_t rx_buf[4] = {0};

    return dwm3000_spi_transceive(ctx, tx_buf, rx_buf, 4);
}
 /* OR 8-bit value with register at offset */
static int dwt_or8bitoffsetreg(struct dwm3000_context *ctx, uint16_t reg, uint16_t offset, uint8_t value)
{
    uint8_t tx_buf[2] = { (uint8_t)(reg & 0x7F), (uint8_t)offset };
    uint8_t rx_buf[2] = {0};
    int ret = dwm3000_spi_transceive(ctx, tx_buf, rx_buf, 2);
    if (ret) {
        return ret;
    }

    tx_buf[0] |= 0x80; // Write mode
    tx_buf[1] = rx_buf[1] | value;
    return dwm3000_spi_transceive(ctx, tx_buf, rx_buf, 2);
}

 int dwm3000_init(struct dwm3000_context *ctx, const struct dwm3000_config *cfg)
 {
     if (!ctx || !cfg) {
         return -EINVAL;
     }
 
    ctx->config = cfg;
    ctx->dblbuffon = DBL_BUFF_ACCESS_BUFFER_0;
    ctx->sleep_mode = 0;
 
     if (!device_is_ready(cfg->spi_dev)) {
         return -ENODEV;
     }
 
     if (!device_is_ready(cfg->gpio_dev)) {
         return -ENODEV;
     }
 
     int ret;
     ret = gpio_pin_configure(cfg->gpio_dev, cfg->cs_pin, GPIO_OUTPUT_HIGH);
     if (ret) {
         return ret;
     }
     ret = gpio_pin_configure(cfg->gpio_dev, cfg->reset_pin, GPIO_OUTPUT_HIGH);
     if (ret) {
         return ret;
     }
     ret = gpio_pin_configure(cfg->gpio_dev, cfg->wakeup_pin, GPIO_OUTPUT_HIGH);
     if (ret) {
         return ret;
     }
     ret = gpio_pin_configure(cfg->gpio_dev, cfg->irq_pin, GPIO_INPUT);
     if (ret) {
         return ret;
     }
 
     return dwm3000_reset(ctx);
 }
 
 int dwm3000_reset(struct dwm3000_context *ctx)
 {
     const struct dwm3000_config *cfg = ctx->config;
 
     int ret;
     ret = gpio_pin_set(cfg->gpio_dev, cfg->wakeup_pin, 0); // Wake low
     if (ret) return ret;
     k_sleep(K_MSEC(100));
     ret = gpio_pin_set(cfg->gpio_dev, cfg->wakeup_pin, 1); // Wake high
     if (ret) return ret;
     k_sleep(K_MSEC(100));
     ret = gpio_pin_set(cfg->gpio_dev, cfg->reset_pin, 0); // Reset low
     if (ret) return ret;
     k_sleep(K_MSEC(20));
     ret = gpio_pin_set(cfg->gpio_dev, cfg->reset_pin, 1); // Reset high
     if (ret) return ret;
     k_sleep(K_MSEC(100));
 
     return 0;
 }
 
 int dwm3000_read_dev_id(struct dwm3000_context *ctx, uint32_t *dev_id)
 {
     if (!ctx || !dev_id) {
         return -EINVAL;
     }
 
     uint8_t tx_buf[5] = {DWM3000_REG_DEV_ID, 0x00, 0x00, 0x00, 0x00};
     uint8_t rx_buf[5] = {0};
 
     int ret = dwm3000_spi_transceive(ctx, tx_buf, rx_buf, sizeof(tx_buf));
     if (ret) {
         return ret;
     }
 
     *dev_id = (rx_buf[4] << 24) | (rx_buf[3] << 16) | (rx_buf[2] << 8) | rx_buf[1];
     return 0;
 }
 
 int dwm3000_get_irq_state(struct dwm3000_context *ctx, int *state)
 {
     if (!ctx || !state) {
         return -EINVAL;
     }
 
     const struct dwm3000_config *cfg = ctx->config;
     *state = gpio_pin_get(cfg->gpio_dev, cfg->irq_pin);
     if (*state < 0) {
         return *state;
     }
 
     return 0;
 }

/* DS TWR function implementations */

/* Reference: ds_twr_initiator_sts.c - port_set_dw_ic_spi_fastrate() sets SPI to high speed (e.g., 8 MHz) */
int port_set_dw_ic_spi_fastrate(struct dwm3000_context *ctx)
{
    if (!ctx || !ctx->config || !device_is_ready(ctx->config->spi_dev)) {
        return -EINVAL;
    }

    struct dwm3000_config *cfg = (struct dwm3000_config *)ctx->config;
    cfg->spi_cfg.frequency = 8000000;
    cfg->spi_cfg.operation = SPI_WORD_SET(8) | SPI_OP_MODE_MASTER;

    return 0;
}

int port_set_dw_ic_spi_slowrate(struct dwm3000_context *ctx)
{
    if (!ctx || !ctx->config || !device_is_ready(ctx->config->spi_dev)) {
        return -EINVAL;
    }

    struct dwm3000_config *cfg = (struct dwm3000_config *)ctx->config;
    cfg->spi_cfg.frequency = 2000000;
    cfg->spi_cfg.operation = SPI_WORD_SET(8) | SPI_OP_MODE_MASTER;

    return 0;
}


/* Reference: ds_twr_initiator_sts.c - reset_DWIC() performs hardware or soft reset of DW IC */
int reset_DWIC(struct dwm3000_context *ctx)
{
    if (!ctx || !ctx->config || !device_is_ready(ctx->config->gpio_dev)) {
        return -EINVAL;
    }

    const struct dwm3000_config *cfg = ctx->config;
    int ret;

#if 1
    /* Hardware reset using RSTn pin */
    ret = gpio_pin_configure(cfg->gpio_dev, cfg->reset_pin, GPIO_OUTPUT | GPIO_OPEN_DRAIN);
    if (ret) {
        return ret;
    }

    ret = gpio_pin_set(cfg->gpio_dev, cfg->reset_pin, 0); // RSTn low
    if (ret) {
        return ret;
    }
    k_sleep(K_USEC(10)); // 10 us

    ret = gpio_pin_set(cfg->gpio_dev, cfg->reset_pin, 1); // RSTn high
    if (ret) {
        return ret;
    }

    ret = gpio_pin_configure(cfg->gpio_dev, cfg->reset_pin, GPIO_OUTPUT_HIGH); // Back to default
    if (ret) {
        return ret;
    }

    k_sleep(K_MSEC(2)); // 2 ms
#else
    /* Soft reset via SPI */
    ret = port_set_dw_ic_spi_slowrate(ctx);
    if (ret) {
        return ret;
    }

    ret = dwt_softreset(ctx);
    if (ret) {
        return ret;
    }

    ret = port_set_dw_ic_spi_fastrate(ctx);
    if (ret) {
        return ret;
    }
#endif

    return 0;
}
/* Reference: ds_twr_initiator_sts.c - dwt_softreset() performs soft reset */
int dwt_softreset(struct dwm3000_context *ctx)
{
    if (!ctx || !ctx->config || !device_is_ready(ctx->config->spi_dev)) {
        return -EINVAL;
    }

    int ret = dwt_clearaonconfig(ctx);
    if (ret) {
        return ret;
    }

    k_sleep(K_MSEC(1)); // 1 ms for AON config

    ret = dwt_or8bitoffsetreg(ctx, CLK_CTRL_ID, 0, FORCE_SYSCLK_FOSC);
    if (ret) {
        return ret;
    }

    ret = dwt_write8bitoffsetreg(ctx, SOFT_RST_ID, 0, DWT_RESET_ALL);
    if (ret) {
        return ret;
    }

    k_sleep(K_MSEC(1)); // 1 ms for PLL lock

    ctx->dblbuffon = DBL_BUFF_ACCESS_BUFFER_0;
    ctx->sleep_mode = 0;

    return 0;
}

/* Reference: ds_twr_initiator_sts.c - dwt_clearaonconfig() clears AON configuration */
int dwt_clearaonconfig(struct dwm3000_context *ctx)
{
    if (!ctx || !ctx->config || !device_is_ready(ctx->config->spi_dev)) {
        return -EINVAL;
    }

    int ret = dwt_write16bitoffsetreg(ctx, AON_DIG_CFG_ID, 0, 0x00);
    if (ret) {
        return ret;
    }

    ret = dwt_write8bitoffsetreg(ctx, ANA_CFG_ID, 0, 0x00);
    if (ret) {
        return ret;
    }

    ret = dwt_write8bitoffsetreg(ctx, AON_CTRL_ID, 0, 0x00);
    if (ret) {
        return ret;
    }

    ret = dwt_write8bitoffsetreg(ctx, AON_CTRL_ID, 0, AON_CTRL_ARRAY_SAVE_BIT_MASK);
    if (ret) {
        return ret;
    }

    return 0;
}

/* Reference: ds_twr_initiator_sts.c - dwt_checkidlerc() checks if DW IC is in IDLE_RC state */
int dwt_checkidlerc(void)
{
    // TODO: Poll status register for IDLE_RC state
    return DWT_SUCCESS;
}

/* Reference: ds_twr_initiator_sts.c - dwt_initialise() initializes DW IC */
int dwt_initialise(int mode)
{
    // TODO: Initialize DWM3000, set default state
    return DWT_SUCCESS;
}

/* Reference: ds_twr_initiator_sts.c - dwt_configure() sets radio parameters */
int dwt_configure(struct dwt_config_t *config)
{
    // TODO: Configure channel, preamble, data rate, etc.
    return DWT_SUCCESS;
}

/* Reference: ds_twr_initiator_sts.c - dwt_configuretxrf() sets TX RF parameters */
int dwt_configuretxrf(struct dwt_txconfig_t *txconfig)
{
    // TODO: Configure TX power and bandwidth
    return DWT_SUCCESS;
}

/* Reference: ds_twr_initiator_sts.c - dwt_setrxantennadelay() sets RX antenna delay */
void dwt_setrxantennadelay(uint16_t delay)
{
    // TODO: Set RX antenna delay in device time units
}

/* Reference: ds_twr_initiator_sts.c - dwt_settxantennadelay() sets TX antenna delay */
void dwt_settxantennadelay(uint16_t delay)
{
    // TODO: Set TX antenna delay in device time units
}

/* Reference: ds_twr_initiator_sts.c - dwt_setrxaftertxdelay() sets RX after TX delay */
void dwt_setrxaftertxdelay(uint32_t delay)
{
    // TODO: Set delay between TX and RX enable
}

/* Reference: ds_twr_initiator_sts.c - dwt_setrxtimeout() sets RX timeout */
void dwt_setrxtimeout(uint16_t timeout)
{
    // TODO: Set RX timeout in UWB microseconds
}

/* Reference: ds_twr_initiator_sts.c - dwt_setpreambledetecttimeout() sets preamble detection timeout */
void dwt_setpreambledetecttimeout(uint16_t timeout)
{
    // TODO: Set preamble detection timeout
}

/* Reference: ds_twr_initiator_sts.c - dwt_setlnapamode() enables LNA/PA */
void dwt_setlnapamode(uint8_t mode)
{
    // TODO: Enable/disable LNA and PA modes
}

/* Reference: ds_twr_initiator_sts.c - dwt_setleds() configures diagnostic LEDs */
void dwt_setleds(uint8_t mode)
{
    // TODO: Configure LEDs for debugging
}

/* Reference: ds_twr_initiator_sts.c - dwt_writetxdata() writes TX frame data */
void dwt_writetxdata(uint16_t length, uint8_t *data, uint16_t offset)
{
    // TODO: Write data to TX buffer
}

/* Reference: ds_twr_initiator_sts.c - dwt_writetxfctrl() configures TX frame control */
void dwt_writetxfctrl(uint16_t length, uint16_t offset, uint8_t ranging)
{
    // TODO: Set TX frame control parameters
}

/* Reference: ds_twr_initiator_sts.c - dwt_starttx() starts frame transmission */
int dwt_starttx(uint8_t mode)
{
    // TODO: Start TX with immediate or delayed mode
    return DWT_SUCCESS;
}

/* Reference: ds_twr_initiator_sts.c - dwt_read32bitreg() reads 32-bit register */
uint32_t dwt_read32bitreg(uint32_t reg)
{
    // TODO: Read 32-bit register via SPI
    return 0;
}

/* Reference: ds_twr_initiator_sts.c - dwt_write32bitreg() writes 32-bit register */
void dwt_write32bitreg(uint32_t reg, uint32_t value)
{
    // TODO: Write 32-bit register via SPI
}

/* Reference: ds_twr_initiator_sts.c - dwt_readrxdata() reads RX frame data */
void dwt_readrxdata(uint8_t *buffer, uint16_t length, uint16_t offset)
{
    // TODO: Read data from RX buffer
}

/* Reference: ds_twr_initiator_sts.c - get_tx_timestamp_u64() gets TX timestamp */
uint64_t get_tx_timestamp_u64(void)
{
    // TODO: Read 64-bit TX timestamp
    return 0;
}

/* Reference: ds_twr_initiator_sts.c - get_rx_timestamp_u64() gets RX timestamp */
uint64_t get_rx_timestamp_u64(void)
{
    // TODO: Read 64-bit RX timestamp
    return 0;
}

/* Reference: ds_twr_initiator_sts.c - final_msg_set_ts() embeds timestamp in final message */
void final_msg_set_ts(uint8_t *ts_field, uint64_t ts)
{
    // TODO: Set timestamp in message payload
}

/* Reference: ds_twr_initiator_sts.c - dwt_setdelayedtrxtime() sets delayed TX time */
void dwt_setdelayedtrxtime(uint32_t time)
{
    // TODO: Set delayed TX time in device time units
}