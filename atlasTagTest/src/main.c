/*
 * Copyright (c) 2025 Your Name
 * SPDX-License-License-Identifier: Apache-2.0
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
     .spi_dev = NULL, // Set at runtime
     .gpio_dev = NULL, // Set at runtime
     .cs_pin = 5, // P0.05
     .reset_pin = 18, // P0.18
     .wakeup_pin = 17, // P0.17
     .irq_pin = 3, // P0.03
     .spi_cfg = {
         .frequency = 2000000,
         .operation = SPI_WORD_SET(8) | SPI_OP_MODE_MASTER,
         .slave = 0,
         .cs = NULL,
     },
 };
 
 void main(void)
 {
     int err;
 
     LOG_INF("Starting Receiver...");
 
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
 
     uint32_t dev_id;
     err = dwm3000_read_dev_id(&dwm3000_ctx, &dev_id);
     if (err) {
         LOG_ERR("DWM3000 read device ID failed: %d", err);
         return;
     }
     LOG_INF("Device ID: 0x%08x", dev_id);
 
     while (1) {
         LOG_INF("Receiver looping...");
         k_sleep(K_SECONDS(1));
     }
 }