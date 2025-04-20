#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/drivers/uart.h>

// Definitions
#define SPI_DEV "SPI_0"
#define CS_PIN   5   // P0.05
#define RST_PIN  18  // P0.18
#define WAKE_PIN 17  // P0.17
#define IRQ_PIN  3   // P0.03
#define REG_DEV_ID 0x00

BUILD_ASSERT(DT_NODE_HAS_COMPAT(DT_CHOSEN(zephyr_console), zephyr_cdc_acm_uart),
             "Console device is not ACM CDC UART device");

static const struct device *spi_dev;
static const struct device *gpio0_dev;

int app_main(void) {
    // Initialize SPI
    spi_dev = device_get_binding(SPI_DEV);
    if (!spi_dev) {
        printk("SPI device %s not found\n", SPI_DEV);
        return 0;
    }
    if (!device_is_ready(spi_dev)) {
        printk("SPI device not ready\n");
        return 0;
    }

    // Initialize GPIO0
    gpio0_dev = DEVICE_DT_GET(DT_NODELABEL(gpio0));
    if (!device_is_ready(gpio0_dev)) {
        printk("GPIO0 device not ready\n");
        return 0;
    }

    // Configure GPIO pins
    int ret;
    ret = gpio_pin_configure(gpio0_dev, CS_PIN, GPIO_OUTPUT_HIGH);
    if (ret) printk("CS config failed: %d\n", ret);
    ret = gpio_pin_configure(gpio0_dev, RST_PIN, GPIO_OUTPUT_HIGH);
    if (ret) printk("Reset config failed: %d\n", ret);
    ret = gpio_pin_configure(gpio0_dev, WAKE_PIN, GPIO_OUTPUT_HIGH);
    if (ret) printk("Wake config failed: %d\n", ret);
    ret = gpio_pin_configure(gpio0_dev, IRQ_PIN, GPIO_INPUT);
    if (ret) printk("IRQ config failed: %d\n", ret);

    // USB CDC setup
    const struct device *console_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
    uint32_t dtr = 0;
    if (usb_enable(NULL)) {
        printk("USB enable failed\n");
        return 0;
    }
    
    while (!dtr) {
        uart_line_ctrl_get(console_dev, UART_LINE_CTRL_DTR, &dtr);
        k_sleep(K_MSEC(100));
    }

    // SPI configuration: 2 MHz, Mode 0 (CPOL=0, CPHA=0)
    struct spi_config config = {
        .frequency = 2000000,
        .operation = SPI_WORD_SET(8) | SPI_OP_MODE_MASTER,  // Mode 0
        .slave = 0,
        .cs = NULL, // Manual CS control
    };

    // Wake and reset DWM3000
    printk("Initializing DWM3000...\n");
    gpio_pin_set(gpio0_dev, WAKE_PIN, 0); // Wake low
    k_sleep(K_MSEC(100));
    gpio_pin_set(gpio0_dev, WAKE_PIN, 1); // Wake high
    k_sleep(K_MSEC(100));
    gpio_pin_set(gpio0_dev, RST_PIN, 0);  // Reset low
    k_sleep(K_MSEC(20));
    gpio_pin_set(gpio0_dev, RST_PIN, 1);  // Reset high
    k_sleep(K_MSEC(100));

    // Check IRQ
    int irq_state = gpio_pin_get(gpio0_dev, IRQ_PIN);
    printk("IRQ state: %d\n", irq_state);

    // Read Device ID
    uint8_t tx_buf[5] = {REG_DEV_ID, 0x00, 0x00, 0x00, 0x00};
    uint8_t rx_buf[5] = {0};
    struct spi_buf tx = {.buf = tx_buf, .len = sizeof(tx_buf)};
    struct spi_buf rx = {.buf = rx_buf, .len = sizeof(rx_buf)};
    struct spi_buf_set tx_set = {.buffers = &tx, .count = 1};
    struct spi_buf_set rx_set = {.buffers = &rx, .count = 1};

    printk("Reading Device ID...\n");
    gpio_pin_set(gpio0_dev, CS_PIN, 0); // CS low
    k_sleep(K_MSEC(2));
    ret = spi_transceive(spi_dev, &config, &tx_set, &rx_set);
    gpio_pin_set(gpio0_dev, CS_PIN, 1); // CS high

    if (ret) {
        printk("SPI error: %d\n", ret);
    } else {
        uint32_t dev_id = (rx_buf[4] << 24) | (rx_buf[3] << 16) | (rx_buf[2] << 8) | rx_buf[1];
        printk("Device ID: 0x%08x\n", dev_id);
    }

    while (1) {
        printk("Looping, still alive...\n");
        
        // Re-read Device ID
        gpio_pin_set(gpio0_dev, CS_PIN, 0); // CS low
        k_sleep(K_MSEC(2));
        ret = spi_transceive(spi_dev, &config, &tx_set, &rx_set);
        gpio_pin_set(gpio0_dev, CS_PIN, 1); // CS high
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