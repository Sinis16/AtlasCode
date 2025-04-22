/*
 * Copyright (c) 2025 Your Name
 * SPDX-License-Identifier: Apache-2.0
*/

#ifndef DWM3000_H
#define DWM3000_H
 
#include <zephyr/kernel.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
 
#define DWM3000_REG_DEV_ID 0x00

/* Constants from Decawave example */
#define FCS_LEN 2 /* Frame Check Sequence length */
#define FRAME_LEN_MAX_EX 0x000007FF /* Maximum frame length */
#define SYS_STATUS_ID 0x0F /* System Status register (placeholder) */
#define RX_FINFO_ID 0x10 /* RX Frame Information register (placeholder) */
#define SYS_STATUS_RXFCG_BIT_MASK 0x00004000 /* RX Frame Check Good (placeholder) */
#define SYS_STATUS_ALL_RX_TO 0x00008000 /* RX Timeout (placeholder) */
#define SYS_STATUS_ALL_RX_ERR 0x00020000 /* RX Error (placeholder) */
#define SYS_STATUS_TXFRS_BIT_MASK 0x00000080 /* TX Frame Sent (placeholder) */

/* DWT constants from deca_device_api.h */
#define DWT_SUCCESS 0
#define DWT_ERROR -1
#define DWT_DW_INIT 0 /* Initialization mode (placeholder) */

/* Channel number */
#define DWT_CHANNEL_5 5

/* Preamble length */
#define DWT_PLEN_128 128

/* Preamble acquisition chunk size */
#define DWT_PAC8 8

/* TX/RX preamble code */
#define DWT_PRF_64M 9

/* SFD mode */
#define DWT_SFD_NON_STD_8 1

/* Data rate */
#define DWT_BR_6M8 0 /* 6.8 Mbps */

/* PHY header mode */
#define DWT_PHRMODE_STD 0

/* PHY header rate */
#define DWT_PHRRATE_STD 0

/* STS mode */
#define DWT_STS_MODE_OFF 0
#define DWT_STS_LEN_64 64

/* PDOA mode */
#define DWT_PDOA_M0 0

/* LNA/PA and LED modes */
#define DWT_LNA_ENABLE 0x01
#define DWT_PA_ENABLE 0x02
#define DWT_LEDS_ENABLE 0x01
#define DWT_LEDS_INIT_BLINK 0x02


/* Structure definitions from Decawave example */
struct dwt_config_t {
    uint8_t chan; /* Channel number */
    uint16_t preamble_len; /* Preamble length */
    uint8_t pac; /* Preamble acquisition chunk size */
    uint8_t tx_code; /* TX preamble code */
    uint8_t rx_code; /* RX preamble code */
    uint8_t sfd_mode; /* SFD mode */
    uint8_t data_rate; /* Data rate */
    uint8_t phr_mode; /* PHY header mode */
    uint8_t phr_rate; /* PHY header rate */
    uint16_t sfd_timeout; /* SFD timeout */
    uint8_t sts_mode; /* STS mode */
    uint16_t sts_len; /* STS length */
    uint8_t pdoa_mode; /* PDOA mode */
};

struct dwt_txconfig_t {
    uint32_t power; /* TX power */
    uint8_t pg_delay; /* Pulse generator delay */
    uint8_t pg_count; /* Pulse generator count (placeholder) */
};

struct dwm3000_config {
    const struct device *spi_dev;
    const struct device *gpio_dev;
    uint8_t cs_pin;
    uint8_t reset_pin;
    uint8_t wakeup_pin;
    uint8_t irq_pin;
    struct spi_config spi_cfg;
};
 
struct dwm3000_context {
    const struct dwm3000_config *config;
};

int dwm3000_init(struct dwm3000_context *ctx, const struct dwm3000_config *cfg);
int dwm3000_reset(struct dwm3000_context *ctx);
int dwm3000_read_dev_id(struct dwm3000_context *ctx, uint32_t *dev_id);
int dwm3000_get_irq_state(struct dwm3000_context *ctx, int *state);

/* DS TWR function declarations */
int port_set_dw_ic_spi_fastrate(struct dwm3000_context *ctx);
void reset_DWIC(void);
int dwt_checkidlerc(void);
int dwt_initialise(int mode);
int dwt_configure(struct dwt_config_t *config);
int dwt_configuretxrf(struct dwt_txconfig_t *txconfig);
void dwt_setrxantennadelay(uint16_t delay);
void dwt_settxantennadelay(uint16_t delay);
void dwt_setrxaftertxdelay(uint32_t delay);
void dwt_setrxtimeout(uint16_t timeout);
void dwt_setpreambledetecttimeout(uint16_t timeout);
void dwt_setlnapamode(uint8_t mode);
void dwt_setleds(uint8_t mode);
void dwt_writetxdata(uint16_t length, uint8_t *data, uint16_t offset);
void dwt_writetxfctrl(uint16_t length, uint16_t offset, uint8_t ranging);
int dwt_starttx(uint8_t mode);
uint32_t dwt_read32bitreg(uint32_t reg);
void dwt_write32bitreg(uint32_t reg, uint32_t value);
void dwt_readrxdata(uint8_t *buffer, uint16_t length, uint16_t offset);
uint64_t get_tx_timestamp_u64(void);
uint64_t get_rx_timestamp_u64(void);
void final_msg_set_ts(uint8_t *ts_field, uint64_t ts);
void dwt_setdelayedtrxtime(uint32_t time);

#endif /* DWM3000_H */