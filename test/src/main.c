#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>

// Device Tree nodes
#define SPI_NODE DT_NODELABEL(spi0)
#define DWM_NODE DT_PATH(soc, spi_40003000, dwm3000_0) // Use DT_PATH for dwm3000@0

// Verify console is CDC ACM UART
BUILD_ASSERT(DT_NODE_HAS_COMPAT(DT_CHOSEN(zephyr_console), zephyr_cdc_acm_uart),
             "Console device is not ACM CDC UART device");

// DWM3000 register for Device ID
#define REG_DEV_ID 0x00

// Device and GPIO bindings
static const struct device *spi_dev = DEVICE_DT_GET(SPI_NODE);
static const struct device *gpio_dev = DEVICE_DT_GET(DT_NODELABEL(gpio0));

// GPIO specifications from Device Tree
static const struct gpio_dt_spec cs_gpio = GPIO_DT_SPEC_GET(SPI_NODE, cs_gpios); // P0.05
static const struct gpio_dt_spec reset_gpio = GPIO_DT_SPEC_GET(DWM_NODE, reset_gpios); // P0.28
static const struct gpio_dt_spec wake_gpio = GPIO_DT_SPEC_GET(DWM_NODE, wakeup_gpios); // P0.29
static const struct gpio_dt_spec irq_gpio = GPIO_DT_SPEC_GET(DWM_NODE, irq_gpios); // P0.03

int main(void)
{
    int ret;

    // Initialize SPI
    if (!device_is_ready(spi_dev)) {
        printk("SPI device not ready\n");
        return 0;
    }

    // Initialize GPIO device
    if (!device_is_ready(gpio_dev)) {
        printk("GPIO device not ready\n");
        return 0;
    }

    // Configure GPIO pins
    ret = gpio_pin_configure_dt(&cs_gpio, GPIO_OUTPUT_HIGH);
    if (ret) {
        printk("CS (P0.05) config failed: %d\n", ret);
        return 0;
    }
    ret = gpio_pin_configure_dt(&reset_gpio, GPIO_OUTPUT_HIGH);
    if (ret) {
        printk("Reset (P0.28) config failed: %d\n", ret);
        return 0;
    }
    ret = gpio_pin_configure_dt(&wake_gpio, GPIO_OUTPUT_HIGH);
    if (ret) {
        printk("Wake (P0.29) config failed: %d\n", ret);
        return 0;
    }
    ret = gpio_pin_configure_dt(&irq_gpio, GPIO_INPUT);
    if (ret) {
        printk("IRQ (P0.03) config failed: %d\n", ret);
        return 0;
    }

    // USB CDC setup
    const struct device *console_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
    if (!device_is_ready(console_dev)) {
        printk("Console device not ready\n");
        return 0;
    }
    uint32_t dtr = 0;
    if (usb_enable(NULL)) {
        printk("USB enable failed\n");
        return 0;
    }

    while (!dtr) {
        uart_line_ctrl_get(console_dev, UART_LINE_CTRL_DTR, &dtr);
        k_sleep(K_MSEC(100));
    }

    // Spi configuration: 2 MHz, Mode 0 (CPOL=0, CPHA=0)
    struct spi_config config = {
        .frequency = DT_PROP(DWM_NODE, spi_max_frequency), // 2 MHz from dwm3000
        .operation = SPI_WORD_SET(8) | SPI_OP_MODE_MASTER, // Mode 0
        .slave = 0,
        .cs = NULL, // Manual CS control
    };

    // Wake and reset DWM3000
    printk("Initializing DWM3000...\n");
    gpio_pin_set_dt(&wake_gpio, 0); // Wake low (P0.29)
    k_sleep(K_MSEC(100));
    gpio_pin_set_dt(&wake_gpio, 1); // Wake high
    k_sleep(K_MSEC(100));
    gpio_pin_set_dt(&reset_gpio, 0); // Reset low (P0.28)
    k_sleep(K_MSEC(20));
    gpio_pin_set_dt(&reset_gpio, 1); // Reset high
    k_sleep(K_MSEC(100));

    // Check IRQ
    int irq_state = gpio_pin_get_dt(&irq_gpio);
    printk("IRQ state (P0.03): %d\n", irq_state);

    // Read Device ID
    uint8_t tx_buf[5] = {REG_DEV_ID, 0x00, 0x00, 0x00, 0x00};
    uint8_t rx_buf[5] = {0};
    struct spi_buf tx = {.buf = tx_buf, .len = sizeof(tx_buf)};
    struct spi_buf rx = {.buf = rx_buf, .len = sizeof(rx_buf)};
    struct spi_buf_set tx_set = {.buffers = &tx, .count = 1};
    struct spi_buf_set rx_set = {.buffers = &rx, .count = 1};

    printk("Reading Device ID...\n");
    gpio_pin_set_dt(&cs_gpio, 0); // CS low (P0.05)
    k_sleep(K_MSEC(2));
    ret = spi_transceive(spi_dev, &config, &tx_set, &rx_set);
    gpio_pin_set_dt(&cs_gpio, 1); // CS high

    if (ret) {
        printk("SPI error: %d\n", ret);
    } else {
        uint32_t dev_id = (rx_buf[4] << 24) | (rx_buf[3] << 16) | (rx_buf[2] << 8) | rx_buf[1];
        printk("Device ID: 0x%08x\n", dev_id);
    }

    while (1) {
        printk("Looping, still alive...\n");

        // Re-read Device ID
        gpio_pin_set_dt(&cs_gpio, 0); // CS low
        k_sleep(K_MSEC(2));
        ret = spi_transceive(spi_dev, &config, &tx_set, &rx_set);
        gpio_pin_set_dt(&cs_gpio, 1); // CS high
        if (ret) {
            printk("SPI error: %d\n", ret);
        } else {
            uint32_t dev_id = (rx_buf[4] << 24) | (rx_buf[3] << 16) | (rx_buf[2] << 8) | rx_buf[1];
            printk("Device ID: 0x%08x\n", dev_id);
        }

        k_sleep(K_SECONDS(1));
    }
    return 0;
}