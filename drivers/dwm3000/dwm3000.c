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
 
 int dwm3000_init(struct dwm3000_context *ctx, const struct dwm3000_config *cfg)
 {
     if (!ctx || !cfg) {
         return -EINVAL;
     }
 
     ctx->config = cfg;
 
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