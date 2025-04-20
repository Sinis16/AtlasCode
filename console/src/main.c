/*
 * Copyright (c) 2016 Intel Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/drivers/uart.h>

#define SPI_DEV "SPI_0"

BUILD_ASSERT(DT_NODE_HAS_COMPAT(DT_CHOSEN(zephyr_console), zephyr_cdc_acm_uart),
	     "Console device is not ACM CDC UART device");

int main(void)
{	
	const struct device *spi = device_get_binding(SPI_DEV);
	if (!spi) {
        printk("SPI device %s not found\n", SPI_DEV);
        return 0;
    }
    if (!device_is_ready(spi)) {
        printk("SPI device not ready\n");
        return 0;
    }

	const struct device *const dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
	uint32_t dtr = 0;

	if (usb_enable(NULL)) {
		return 0;
	}

 
	uint8_t tx_buf[] = {0x00};
    uint8_t rx_buf[1] = {0};
    struct spi_buf tx = {.buf = tx_buf, .len = sizeof(tx_buf)};
    struct spi_buf rx = {.buf = rx_buf, .len = sizeof(rx_buf)};
    struct spi_buf_set tx_set = {.buffers = &tx, .count = 1};
    struct spi_buf_set rx_set = {.buffers = &rx, .count = 1};


	struct spi_config config = {
        .frequency = 8000000,
        .operation = SPI_WORD_SET(8) | SPI_OP_MODE_MASTER | SPI_MODE_CPOL,
        .slave = 0,
        .cs = NULL,
    };

	/* Poll if the DTR flag was set */
	while (!dtr) {
		uart_line_ctrl_get(dev, UART_LINE_CTRL_DTR, &dtr);
		/* Give CPU resources to low priority threads. */
		k_sleep(K_MSEC(100));
	}

	while (1) {
        int ret = spi_transceive(spi, &config, &tx_set, &rx_set);
        if (ret) {
            printk("SPI error: %s\n", CONFIG_ARCH);
			printk("amognt", CONFIG_ARCH);
        } else {
            printk("SPI TX: 0x%02x, RX: 0x%02x\n", tx_buf[0], rx_buf[0], CONFIG_ARCH);
			printk("amog", CONFIG_ARCH);
        }
        k_sleep(K_MSEC(1000));
    }
}
