/*
 * Copyright (c) 2025 Your Name
 * SPDX-License-Identifier: Apache-2.0
*/

#ifndef DWM3000_H
#define DWM3000_H
 
#include <zephyr/kernel.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>

#include <stdint.h>
#include <stddef.h>
 
#define DWM3000_REG_DEV_ID 0x00

/* Constants from Decawave example */
#define FCS_LEN 2 /* Frame Check Sequence length */
#define FRAME_LEN_MAX_EX 0x000007FF /* Maximum frame length */
#define SYS_STATUS_ID 0x44 /* System Status register (placeholder) */
#define SYS_STATUS_IDLE_BIT 0x00000001 /* IDLE_RC state bit */
#define SYS_STATUS_RCINIT_BIT_MASK           0x01000000UL

#define RX_FINFO_ID 0x10 /* RX Frame Information register (placeholder) */
#define SYS_STATUS_RXFCG_BIT_MASK 0x00004000 /* RX Frame Check Good (placeholder) */
#define SYS_STATUS_ALL_RX_TO 0x00008000 /* RX Timeout (placeholder) */
#define SYS_STATUS_ALL_RX_ERR 0x00020000 /* RX Error (placeholder) */
#define SYS_STATUS_TXFRS_BIT_MASK 0x00000080 /* TX Frame Sent (placeholder) */

/* DWT constants from deca_device_api.h */
#define DWT_SUCCESS 0
#define DWT_ERROR -1

//DW3000 IDLE/INIT mode definitions
#define DWT_DW_INIT      0x0
#define DWT_DW_IDLE      0x1
#define DWT_DW_IDLE_RC   0x2

#define DWT_READ_OTP_PID  0x10    //read part ID from OTP
#define DWT_READ_OTP_LID  0x20    //read lot ID from OTP
#define DWT_READ_OTP_BAT  0x40    //read ref voltage from OTP
#define DWT_READ_OTP_TMP  0x80    //read ref temperature from OTP

#define DBL_BUFF_OFF             0x0
#define DBL_BUFF_ACCESS_BUFFER_0 0x1
#define DBL_BUFF_ACCESS_BUFFER_1 0x3
/******************************************************************************
* @brief Bit definitions for register OTP_CFG
**/
#define OTP_CFG_ID                           0xb0008
#define OTP_CFG_LEN                          (4U)
#define OTP_CFG_MASK                         0xFFFFFFFFUL
#define OTP_CFG_DGC_SEL_BIT_OFFSET           (13U)
#define OTP_CFG_DGC_SEL_BIT_LEN              (1U)
#define OTP_CFG_DGC_SEL_BIT_MASK             0x2000U
#define OTP_CFG_OPS_ID_BIT_OFFSET           (11U)
#define OTP_CFG_OPS_ID_BIT_LEN              (2U)
#define OTP_CFG_OPS_ID_BIT_MASK             0x1800U
#define OTP_CFG_OPS_KICK_BIT_OFFSET         (10U)
#define OTP_CFG_OPS_KICK_BIT_LEN            (1U)
#define OTP_CFG_OPS_KICK_BIT_MASK           0x400U
#define OTP_CFG_BIAS_KICK_BIT_OFFSET         (8U)
#define OTP_CFG_BIAS_KICK_BIT_LEN            (1U)
#define OTP_CFG_BIAS_KICK_BIT_MASK           0x100U
#define OTP_CFG_LDO_KICK_BIT_OFFSET          (7U)
#define OTP_CFG_LDO_KICK_BIT_LEN             (1U)
#define OTP_CFG_LDO_KICK_BIT_MASK            0x80U
#define OTP_CFG_DGC_KICK_BIT_OFFSET          (6U)
#define OTP_CFG_DGC_KICK_BIT_LEN             (1U)
#define OTP_CFG_DGC_KICK_BIT_MASK            0x40U
#define OTP_CFG_OTP_WRITE_MR_BIT_OFFSET      (3U)
#define OTP_CFG_OTP_WRITE_MR_BIT_LEN         (1U)
#define OTP_CFG_OTP_WRITE_MR_BIT_MASK        0x8U
#define OTP_CFG_OTP_WRITE_BIT_OFFSET         (2U)
#define OTP_CFG_OTP_WRITE_BIT_LEN            (1U)
#define OTP_CFG_OTP_WRITE_BIT_MASK           0x4U
#define OTP_CFG_OTP_READ_BIT_OFFSET          (1U)
#define OTP_CFG_OTP_READ_BIT_LEN             (1U)
#define OTP_CFG_OTP_READ_BIT_MASK            0x2U
#define OTP_CFG_OTP_MAN_CTR_EN_BIT_OFFSET    (0U)
#define OTP_CFG_OTP_MAN_CTR_EN_BIT_LEN       (1U)
#define OTP_CFG_OTP_MAN_CTR_EN_BIT_MASK      0x1U

/******************************************************************************
* @brief Bit definitions for register OTP_ADDR
**/
#define OTP_ADDR_ID                          0xb0004
#define OTP_ADDR_LEN                         (4U)
#define OTP_ADDR_MASK                        0xFFFFFFFFUL
#define OTP_ADDR_OTP_ADDR_BIT_OFFSET         (0U)
#define OTP_ADDR_OTP_ADDR_BIT_LEN            (11U)
#define OTP_ADDR_OTP_ADDR_BIT_MASK           0x7ffU

/******************************************************************************
* @brief Bit definitions for register OTP_RDATA
**/
#define OTP_RDATA_ID                         0xb0010
#define OTP_RDATA_LEN                        (4U)
#define OTP_RDATA_MASK                       0xFFFFFFFFUL
#define OTP_RDATA_OTP_RDATA_BIT_OFFSET       (0U)
#define OTP_RDATA_OTP_RDATA_BIT_LEN          (32U)
#define OTP_RDATA_OTP_RDATA_BIT_MASK         0xffffffffUL


/******************************************************************************
* @brief Bit definitions for register XTAL
**/
#define XTAL_ID                              0x90014
#define XTAL_LEN                             (4U)
#define XTAL_MASK                            0xFFFFFFFFUL
#define XTAL_XTAL_TRIM_BIT_OFFSET            (0U)
#define XTAL_XTAL_TRIM_BIT_LEN               (7U)
#define XTAL_XTAL_TRIM_BIT_MASK              0x7fU


/******************************************************************************
* @brief Bit definitions for register BIAS_CTRL
**/
#define BIAS_CTRL_ID                         0x11001f             /*  */
#define BIAS_CTRL_LEN                        (4U)
#define BIAS_CTRL_MASK                       0xFFFFFFFFUL
#define BIAS_CTRL_BIAS_OFFSET (0U)
#define BIAS_CTRL_BIAS_LEN   (5U)
#define BIAS_CTRL_BIAS_MASK  0x1fU

typedef enum
{
    DWT_DGC_LOAD_FROM_SW=0,
    DWT_DGC_LOAD_FROM_OTP
} dwt_dgc_load_location;

#define DWT_DGC_CFG             0x32
#define DWT_DGC_CFG0            0x10000240
#define DWT_DGC_CFG1            0x1b6da489
#define PD_THRESH_NO_DATA       0xAF5F35CC      /* PD threshold for no data STS mode*/
#define PD_THRESH_DEFAULT       0xAF5F584C

// OTP addresses definitions
#define LDOTUNELO_ADDRESS (0x04)
#define LDOTUNEHI_ADDRESS (0x05)
#define PARTID_ADDRESS  (0x06)
#define LOTID_ADDRESS   (0x07)
#define VBAT_ADDRESS    (0x08)
#define VTEMP_ADDRESS   (0x09)
#define XTRIM_ADDRESS   (0x1E)
#define OTPREV_ADDRESS  (0x1F)
#define BIAS_TUNE_ADDRESS (0xA)
#define DGC_TUNE_ADDRESS (0x20)

//DW3000 SLEEP and WAKEUP configuration parameters
#define DWT_PGFCAL       0x0800
#define DWT_GOTORX       0x0200
#define DWT_GOTOIDLE     0x0100
#define DWT_SEL_OPS3     0x00C0
#define DWT_SEL_OPS2     0x0080                     // Short OPS table
#define DWT_SEL_OPS1     0x0040                     // SCP
#define DWT_SEL_OPS0     0x0000                     // Long OPS table
#define DWT_ALT_OPS      0x0020
#define DWT_LOADLDO      0x0010
#define DWT_LOADDGC      0x0008
#define DWT_LOADBIAS     0x0004
#define DWT_RUNSAR       0x0002
#define DWT_CONFIG       0x0001                     // download the AON array into the HIF (configuration download)

#define DWT_PRES_SLEEP   0x20                       // allows for SLEEP_EN bit to be "preserved", although it will self - clear on wake up
#define DWT_WAKE_WUP     0x10                       // wake up on WAKEUP PIN
#define DWT_WAKE_CSN     0x8                        // wake up on chip select
#define DWT_BROUT_EN     0x4                        // enable brownout detector during sleep/deep sleep
#define DWT_SLEEP        0x2                        // enable sleep (if this bit is clear the device will enter deep sleep)
#define DWT_SLP_EN       0x1     


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

/* Soft reset constants */
#define CLK_CTRL_ID 0x2B /* Clock Control register */
#define FORCE_SYSCLK_FOSC 0x01 /* Force FOSC clock */
#define SOFT_RST_ID 0x3F /* Soft Reset register */
#define DWT_RESET_ALL 0x0F /* Reset HIF, TX, RX, PMSC */
#define AON_DIG_CFG_ID 0x2C /* AON Digital Config register */
#define ANA_CFG_ID 0x2E /* Analog Config register */
#define AON_CTRL_ID 0x2D /* AON Control register */
#define AON_CTRL_ARRAY_SAVE_BIT_MASK 0x01 /* Save AON array */

/* Double buffer constants */
#define DBL_BUFF_ACCESS_BUFFER_0 0x00 /* Access RX_BUFFER_0 */

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

typedef struct
{
    uint32_t status;      //initial value of register as ISR is entered
    uint16_t status_hi;   //initial value of register as ISR is entered, if relevant for that event type
    uint16_t datalength;  //length of frame
    uint8_t  rx_flags;    //RX frame flags, see above
} dwt_cb_data_t;

typedef void (*dwt_cb_t)(const dwt_cb_data_t *);

#define DWM3000_SPI_BUF_SIZE 255

struct dwm3000_context {
    const struct dwm3000_config *config;
    uint8_t dblbuffon; /* Double buffer state */
    uint8_t sleep_mode; /* Sleep mode state */
    uint32_t last_sys_status; /* Last SYS_STATUS on dwt_checkidlerc failure */
    uint8_t tx_buf[DWM3000_SPI_BUF_SIZE]; /* Transmit buffer */
    uint8_t rx_buf[DWM3000_SPI_BUF_SIZE]; /* Receive buffer */
    size_t buf_size; /* Size of the buffers */
};

typedef enum {
    DW3000_SPI_RD_BIT    = 0x0000U,
    DW3000_SPI_WR_BIT    = 0x8000U,
    DW3000_SPI_AND_OR_8  = 0x8001U,
    DW3000_SPI_AND_OR_16 = 0x8002U,
    DW3000_SPI_AND_OR_32 = 0x8003U,
} spi_modes_e;

/******************************************************************************
* @brief Bit definitions for register SPICRC_CFG
**/
#define SPICRC_CFG_ID                        0x18
#define SPICRC_CFG_LEN                       (4U)
#define SPICRC_CFG_MASK                      0xFFFFFFFFUL
#define SPICRC_CFG_SPI_RD_CRC_BIT_OFFSET     (0U)
#define SPICRC_CFG_SPI_RD_CRC_BIT_LEN        (8U)
#define SPICRC_CFG_SPI_RD_CRC_BIT_MASK       0xffU

// LDO and BIAS tune kick
#define LDO_BIAS_KICK (0x180)  // Writing to bit 7 and 8

#define dwt_or16bitoffsetreg(ctx, addr, offset, or_val) dwt_modify16bitoffsetreg(ctx, addr, offset, -1, or_val)
#define dwt_and16bitoffsetreg(ctx, addr, offset, and_val) dwt_modify16bitoffsetreg(ctx, addr, offset, and_val, 0)
#define dwt_and_or16bitoffsetreg(ctx, addr,offset, and_val, or_val) dwt_modify16bitoffsetreg(ctx, addr,offset,and_val,or_val)

typedef enum
{
    DWT_SPI_CRC_MODE_NO = 0,    /* No CRC */
    DWT_SPI_CRC_MODE_WR,        /* This is used to enable SPI CRC check (the SPI CRC check will be enabled on DW3000 and CRC-8 added for SPI write transactions) */
    DWT_SPI_CRC_MODE_WRRD       /* This is used to optionally enable additional CRC check on the SPI read operations, while the CRC check on the SPI write operations is also enabled */
}dwt_spi_crc_mode_e;

// Macros and Enumerations for SPI & CLock blocks
//
#define DW3000_SPI_FAC      (0<<6 | 1<<0)
#define DW3000_SPI_FARW     (0<<6 | 0<<0)
#define DW3000_SPI_EAMRW    (1<<6)

#ifndef DWT_NUM_DW_DEV
#define DWT_NUM_DW_DEV (1)
#endif

typedef void(*dwt_spierrcb_t)(void);


typedef __uint8_t uint8_t;
typedef __uint16_t uint16_t;
typedef __uint32_t uint32_t;
typedef __uint64_t uint64_t;

typedef __int8_t int8_t;
typedef __int16_t int16_t;
typedef __int32_t int32_t;
typedef __int64_t int64_t;

int dwm3000_spi_transceive(struct dwm3000_context *ctx, uint8_t *tx_buf, uint8_t *rx_buf, size_t len);
int dwm3000_init(struct dwm3000_context *ctx, const struct dwm3000_config *cfg);
int dwm3000_reset(struct dwm3000_context *ctx);
int dwm3000_read_dev_id(struct dwm3000_context *ctx, uint32_t *dev_id);
int dwm3000_get_irq_state(struct dwm3000_context *ctx, int *state);

/* DS TWR function declarations */
int port_set_dw_ic_spi_fastrate(struct dwm3000_context *ctx);
int port_set_dw_ic_spi_slowrate(struct dwm3000_context *ctx);
int true_reset_DWIC(struct dwm3000_context *ctx);
int reset_DWIC(struct dwm3000_context *ctx);

//int new_dwt_checkidlerc(struct dwm3000_context *ctx);
uint32_t dwt_read32bitoffsetreg(struct dwm3000_context *ctx, int regFileID, int regOffset);
uint16_t dwt_read16bitoffsetreg(struct dwm3000_context *ctx, int reg, int offset);
uint8_t dwt_generatecrc8(const uint8_t* byteArray, int len, uint8_t crcRemainderInit);
void dwt_init_crc_table(void);
int dwt_initialise(struct dwm3000_context *ctx, int mode);

int readfromspi(
    struct dwm3000_context *ctx,
    uint16_t        headerLength,
    const uint8_t * headerBuffer,
    uint16_t        readLength,
    uint8_t       * readBuffer);

int writetospiwithcrc(
    struct dwm3000_context *ctx,
    uint16_t           headerLength,
    const    uint8_t * headerBuffer,
    uint16_t           bodyLength,
    const    uint8_t * bodyBuffer,
    uint8_t            crc8);

int writetospi(
    struct dwm3000_context *ctx,
    uint16_t           headerLength,
    const    uint8_t * headerBuffer,
    uint16_t           bodyLength,
    const    uint8_t * bodyBuffer);

    uint8_t dwt_read8bitoffsetreg(
        struct dwm3000_context *ctx,
        uint32_t regFileID,
        uint16_t regOffset);
    
    void dwt_readfromdevice(
        struct dwm3000_context *ctx,
        uint32_t regFileID,
        uint16_t index,
        uint16_t length,
        uint8_t *buffer);
    
    void dwt_xfer3000(
        struct dwm3000_context *ctx,
        const uint32_t regFileID,
        const uint16_t indx,
        const uint16_t length,
        uint8_t *buffer,
        const spi_modes_e mode);

int dwt_softreset(struct dwm3000_context *ctx);
int dwt_clearaonconfig(struct dwm3000_context *ctx);
int dwt_checkidlerc(struct dwm3000_context *ctx);
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