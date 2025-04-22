#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/drivers/uart.h>

// Definitions
#define CS_PIN   5   // P0.05
#define RST_PIN  28  // P0.28
#define WAKE_PIN 29  // P0.29
#define IRQ_PIN  3   // P0.03

BUILD_ASSERT(DT_NODE_HAS_COMPAT(DT_CHOSEN(zephyr_console), zephyr_cdc_acm_uart),
             "Console device is not ACM CDC UART device");

static const struct device *gpio0_dev;

int main(void) {
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

    printk("Starting GPIO test...\n");

    while (1) {
        // Set all output pins low
        ret = gpio_pin_set(gpio0_dev, CS_PIN, 0);
        if (ret) printk("CS set low failed: %d\n", ret);
        ret = gpio_pin_set(gpio0_dev, RST_PIN, 0);
        if (ret) printk("RST set low failed: %d\n", ret);
        ret = gpio_pin_set(gpio0_dev, WAKE_PIN, 0);
        if (ret) printk("WAKE set low failed: %d\n", ret);

        // Read IRQ state
        int irq_state = gpio_pin_get(gpio0_dev, IRQ_PIN);
        if (irq_state < 0) {
            printk("IRQ read failed: %d\n", irq_state);
        } else {
            printk("Pins LOW - CS: 0, RST: 0, WAKE: 0, IRQ: %d\n", irq_state);
        }

        k_sleep(K_SECONDS(2));

        // Set all output pins high
        ret = gpio_pin_set(gpio0_dev, CS_PIN, 1);
        if (ret) printk("CS set high failed: %d\n", ret);
        ret = gpio_pin_set(gpio0_dev, RST_PIN, 1);
        if (ret) printk("RST set high failed: %d\n", ret);
        ret = gpio_pin_set(gpio0_dev, WAKE_PIN, 1);
        if (ret) printk("WAKE set high failed: %d\n", ret);

        // Read IRQ state
        irq_state = gpio_pin_get(gpio0_dev, IRQ_PIN);
        if (irq_state < 0) {
            printk("IRQ read failed: %d\n", irq_state);
        } else {
            printk("Pins HIGH - CS: 1, RST: 1, WAKE: 1, IRQ: %d\n", irq_state);
        }

        k_sleep(K_SECONDS(2));
    }

    return 0;
}