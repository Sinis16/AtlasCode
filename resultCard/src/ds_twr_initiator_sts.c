#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/usb/usb_device.h>
#include <deca_device_api.h>
#include <deca_regs.h>
#include <port.h>

#define SPI_DEV "SPI_0"
#define CS_PIN   5   // P0.05
#define RST_PIN  28  // P0.28 (was 19)
#define WAKE_PIN 29  // P0.29 (was 20)
#define IRQ_PIN  3   // P0.03
#define REG_DEV_ID 0x00

BUILD_ASSERT(DT_NODE_HAS_COMPAT(DT_CHOSEN(zephyr_console), zephyr_cdc_acm_uart),
             "Console device is not ACM CDC UART device");

static const struct device *spi_dev;
static const struct device *gpio0_dev;

static struct spi_config spi_cfg = {
    .frequency = 2000000,
    .operation = SPI_WORD_SET(8) | SPI_OP_MODE_MASTER,
    .slave = 0,
    .cs = NULL,
};


static dwt_config_t config = { 5, DWT_PLEN_128, DWT_PAC8, 9, 9, 1, DWT_BR_6M8, DWT_PHRMODE_STD, DWT_PHRRATE_STD, (129 + 8 - 8), DWT_STS_MODE_OFF, DWT_STS_LEN_64, DWT_PDOA_M0 };
static uint8_t tx_poll_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0x21};


int app_main(void) {
    spi_dev = device_get_binding(SPI_DEV);
    if (!spi_dev || !device_is_ready(spi_dev)) {
        printk("SPI device %s not ready\n", SPI_DEV);
        return 0;
    }
    else {
        printk("SPI device %s is ready\n", SPI_DEV);
    }

    gpio0_dev = DEVICE_DT_GET(DT_NODELABEL(gpio0));
    if (!device_is_ready(gpio0_dev)) {
        printk("GPIO0 device not ready\n");
        return 0;
    } else {
        printk("GPIO0 device %s is ready\n");
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
    k_sleep(K_MSEC(100));
    

    //reset_DWIC();

    printk("Initializing DWM3000...\n");
    gpio_pin_set(gpio0_dev, WAKE_PIN, 0);
    k_sleep(K_MSEC(100));
    gpio_pin_set(gpio0_dev, WAKE_PIN, 1);
    k_sleep(K_MSEC(100));

    port_set_dw_ic_spi_fastrate();
    printk("SPI fast\n");

    gpio_pin_set(gpio0_dev, RST_PIN, 0);
    k_usleep(10);
    gpio_pin_set(gpio0_dev, RST_PIN, 1);
    setup_DW3000RSTnIRQPro(0);

    k_sleep(K_MSEC(2));
    printk("DWM3000 reset via SPI\n");

    //while (!dwt_checkidlerc()) {
    //    printk("Waiting IDLE_RC...\n");
    //    k_sleep(K_MSEC(1));
    //}

    //if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR) {
    //    printk("Init failed\n");
    //    while (1) { k_sleep(K_SECONDS(1)); printk("Init dead\n"); }
    //}
    //printk("DWM3000 init done\n");

    //if (dwt_configure(&config)) {
    //    printk("Config failed\n");
    //    while (1) { k_sleep(K_SECONDS(1)); printk("Config dead\n"); }
    //}
    //printk("DWM3000 configured\n");

    int irq_state = gpio_pin_get(gpio0_dev, IRQ_PIN);
    printk("IRQ state: %d\n", irq_state);

    uint8_t tx_buf[5] = {REG_DEV_ID, 0x00, 0x00, 0x00, 0x00};
    uint8_t rx_buf[5] = {0};
    struct spi_buf tx = {.buf = tx_buf, .len = sizeof(tx_buf)};
    struct spi_buf rx = {.buf = rx_buf, .len = sizeof(rx_buf)};
    struct spi_buf_set tx_set = {.buffers = &tx, .count = 1};
    struct spi_buf_set rx_set = {.buffers = &rx, .count = 1};

    printk("Reading Device ID...\n");
    gpio_pin_set(gpio0_dev, CS_PIN, 0);
    k_sleep(K_MSEC(2));
    ret = spi_transceive(spi_dev, &spi_cfg, &tx_set, &rx_set);
    gpio_pin_set(gpio0_dev, CS_PIN, 1);

    if (ret) {
        printk("SPI error: %d\n", ret);
    } else {
        uint32_t dev_id = (rx_buf[4] << 24) | (rx_buf[3] << 16) | (rx_buf[2] << 8) | rx_buf[1];
        printk("Device ID: 0x%08x\n", dev_id);
    }

    /*
    uint8_t frame_seq_nb = 0;
    printk("Polling start...\n");
    while (1) {
        tx_poll_msg[2] = frame_seq_nb++;
        dwt_writetxdata(sizeof(tx_poll_msg), tx_poll_msg, 0);
        dwt_writetxfctrl(sizeof(tx_poll_msg) + FCS_LEN, 0, 1);
        if (dwt_starttx(DWT_START_TX_IMMEDIATE) == DWT_ERROR) {
            printk("TX failed\n");
        } else {
            printk("Poll %d sent\n", frame_seq_nb);
        }
        k_sleep(K_MSEC(1000));
    }
    */

    while (1) {
        printk("Looping, still alive...\n");
        k_sleep(K_SECONDS(1));
    }
    return 0;
}

void setup_DW3000RSTnIRQPro(int enable)
{
    if (enable) {
        /* Configure as active output for reset */
        gpio_pin_configure(gpio0_dev, RST_PIN, GPIO_OUTPUT);
    } else {
        /* Set back to high-impedance output (inactive) */
        gpio_pin_configure(gpio0_dev, RST_PIN, GPIO_OUTPUT);
        gpio_pin_set(gpio0_dev, RST_PIN, 1);  // Ensure high (inactive) state
    }
}