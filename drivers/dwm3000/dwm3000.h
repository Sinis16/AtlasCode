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
#define FCS_LEN                 (2)
#define FRAME_LEN_MAX_EX 0x000007FF /* Maximum frame length */
#define SYS_STATUS_IDLE_BIT 0x00000001 /* IDLE_RC state bit */

#define RX_FINFO_ID 0x10 /* RX Frame Information register (placeholder) */
#define SYS_STATUS_ALL_RX_TO 0x00008000 /* RX Timeout (placeholder) */
#define SYS_STATUS_ALL_RX_ERR 0x00020000 /* RX Error (placeholder) */

#define DELAY_20uUSec           (20)   
#define MAX_RETRIES_FOR_PLL     (6)
#define MAX_RETRIES_FOR_PGF     (3)

/******************************************************************************
* @brief Bit definitions for register SYS_STATUS
**/
#define SYS_STATUS_ID                        0x44
#define SYS_STATUS_LEN                       (4U)
#define SYS_STATUS_MASK                      0xFFFFFFFFUL
#define SYS_STATUS_ARFE_BIT_OFFSET           (29U)
#define SYS_STATUS_ARFE_BIT_LEN              (1U)
#define SYS_STATUS_ARFE_BIT_MASK             0x20000000UL
#define SYS_STATUS_CPERR_BIT_OFFSET          (28U)
#define SYS_STATUS_CPERR_BIT_LEN             (1U)
#define SYS_STATUS_CPERR_BIT_MASK            0x10000000UL
#define SYS_STATUS_HPDWARN_BIT_OFFSET        (27U)
#define SYS_STATUS_HPDWARN_BIT_LEN           (1U)
#define SYS_STATUS_HPDWARN_BIT_MASK          0x08000000UL
#define SYS_STATUS_RXSTO_BIT_OFFSET          (26U)
#define SYS_STATUS_RXSTO_BIT_LEN             (1U)
#define SYS_STATUS_RXSTO_BIT_MASK            0x04000000UL
#define SYS_STATUS_PLL_HILO_BIT_OFFSET       (25U)
#define SYS_STATUS_PLL_HILO_BIT_LEN          (1U)
#define SYS_STATUS_PLL_HILO_BIT_MASK         0x02000000UL
#define SYS_STATUS_RCINIT_BIT_OFFSET         (24U)
#define SYS_STATUS_RCINIT_BIT_LEN            (1U)
#define SYS_STATUS_RCINIT_BIT_MASK           0x01000000UL
#define SYS_STATUS_SPIRDY_BIT_OFFSET         (23U)
#define SYS_STATUS_SPIRDY_BIT_LEN            (1U)
#define SYS_STATUS_SPIRDY_BIT_MASK           0x00800000UL
#define SYS_STATUS_RXPTO_BIT_OFFSET          (21U)
#define SYS_STATUS_RXPTO_BIT_LEN             (1U)
#define SYS_STATUS_RXPTO_BIT_MASK            0x00200000UL
#define SYS_STATUS_RXOVRR_BIT_OFFSET         (20U)
#define SYS_STATUS_RXOVRR_BIT_LEN            (1U)
#define SYS_STATUS_RXOVRR_BIT_MASK           0x00100000UL
#define SYS_STATUS_VWARN_BIT_OFFSET          (19U)
#define SYS_STATUS_VWARN_BIT_LEN             (1U)
#define SYS_STATUS_VWARN_BIT_MASK            0x00080000UL
#define SYS_STATUS_CIAERR_BIT_OFFSET         (18U)
#define SYS_STATUS_CIAERR_BIT_LEN            (1U)
#define SYS_STATUS_CIAERR_BIT_MASK           0x00040000UL
#define SYS_STATUS_RXFTO_BIT_OFFSET          (17U)
#define SYS_STATUS_RXFTO_BIT_LEN             (1U)
#define SYS_STATUS_RXFTO_BIT_MASK            0x00020000UL
#define SYS_STATUS_RXFSL_BIT_OFFSET          (16U)
#define SYS_STATUS_RXFSL_BIT_LEN             (1U)
#define SYS_STATUS_RXFSL_BIT_MASK            0x00010000UL
#define SYS_STATUS_RXFCE_BIT_OFFSET          (15U)
#define SYS_STATUS_RXFCE_BIT_LEN             (1U)
#define SYS_STATUS_RXFCE_BIT_MASK            0x00008000U
#define SYS_STATUS_RXFCG_BIT_OFFSET          (14U)
#define SYS_STATUS_RXFCG_BIT_LEN             (1U)
#define SYS_STATUS_RXFCG_BIT_MASK            0x00004000U
#define SYS_STATUS_RXFR_BIT_OFFSET           (13U)
#define SYS_STATUS_RXFR_BIT_LEN              (1U)
#define SYS_STATUS_RXFR_BIT_MASK             0x00002000U
#define SYS_STATUS_RXPHE_BIT_OFFSET          (12U)
#define SYS_STATUS_RXPHE_BIT_LEN             (1U)
#define SYS_STATUS_RXPHE_BIT_MASK            0x00001000U
#define SYS_STATUS_RXPHD_BIT_OFFSET          (11U)
#define SYS_STATUS_RXPHD_BIT_LEN             (1U)
#define SYS_STATUS_RXPHD_BIT_MASK            0x00000800U
#define SYS_STATUS_CIADONE_BIT_OFFSET        (10U)
#define SYS_STATUS_CIADONE_BIT_LEN           (1U)
#define SYS_STATUS_CIADONE_BIT_MASK          0x00000400U
#define SYS_STATUS_RXSFDD_BIT_OFFSET         (9U)
#define SYS_STATUS_RXSFDD_BIT_LEN            (1U)
#define SYS_STATUS_RXSFDD_BIT_MASK           0x00000200U
#define SYS_STATUS_RXPRD_BIT_OFFSET          (8U)
#define SYS_STATUS_RXPRD_BIT_LEN             (1U)
#define SYS_STATUS_RXPRD_BIT_MASK            0x00000100U
#define SYS_STATUS_TXFRS_BIT_OFFSET          (7U)
#define SYS_STATUS_TXFRS_BIT_LEN             (1U)
#define SYS_STATUS_TXFRS_BIT_MASK            0x00000080U
#define SYS_STATUS_TXPHS_BIT_OFFSET          (6U)
#define SYS_STATUS_TXPHS_BIT_LEN             (1U)
#define SYS_STATUS_TXPHS_BIT_MASK            0x00000040U
#define SYS_STATUS_TXPRS_BIT_OFFSET          (5U)
#define SYS_STATUS_TXPRS_BIT_LEN             (1U)
#define SYS_STATUS_TXPRS_BIT_MASK            0x00000020U
#define SYS_STATUS_TXFRB_BIT_OFFSET          (4U)
#define SYS_STATUS_TXFRB_BIT_LEN             (1U)
#define SYS_STATUS_TXFRB_BIT_MASK            0x00000010U
#define SYS_STATUS_AAT_BIT_OFFSET            (3U)
#define SYS_STATUS_AAT_BIT_LEN               (1U)
#define SYS_STATUS_AAT_BIT_MASK              0x00000008U
#define SYS_STATUS_SPICRCE_BIT_OFFSET        (2U)
#define SYS_STATUS_SPICRCE_BIT_LEN           (1U)
#define SYS_STATUS_SPICRCE_BIT_MASK          0x00000004U
#define SYS_STATUS_CP_LOCK_BIT_OFFSET        (1U)
#define SYS_STATUS_CP_LOCK_BIT_LEN           (1U)
#define SYS_STATUS_CP_LOCK_BIT_MASK          0x00000002U
#define SYS_STATUS_IRQS_BIT_OFFSET           (0U)
#define SYS_STATUS_IRQS_BIT_LEN              (1U)
#define SYS_STATUS_IRQS_BIT_MASK             0x00000001U


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

/******************************************************************************
* @brief Bit definitions for register DGC_CFG
**/
#define DGC_CFG_ID                           0x30018
#define DGC_CFG_LEN                          (4U)
#define DGC_CFG_MASK                         0xFFFFFFFFUL
#define DGC_CFG_THR_64_BIT_OFFSET            (9U)
#define DGC_CFG_THR_64_BIT_LEN               (6U)
#define DGC_CFG_THR_64_BIT_MASK              0x7e00U
#define DGC_CFG_RX_TUNE_EN_BIT_OFFSET        (0U)
#define DGC_CFG_RX_TUNE_EN_BIT_LEN           (1U)
#define DGC_CFG_RX_TUNE_EN_BIT_MASK          0x1U

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

/******************************************************************************
* @brief Bit definitions for register SYS_CFG
**/
#define SYS_CFG_ID                           0x10
#define SYS_CFG_LEN                          (4U)
#define SYS_CFG_MASK                         0xFFFFFFFFUL
#define SYS_CFG_FAST_AAT_EN_BIT_OFFSET       (18U)
#define SYS_CFG_FAST_AAT_EN_BIT_LEN          (1U)
#define SYS_CFG_FAST_AAT_EN_BIT_MASK         0x40000UL
#define SYS_CFG_PDOA_MODE_BIT_OFFSET         (16U)
#define SYS_CFG_PDOA_MODE_BIT_LEN            (2U)
#define SYS_CFG_PDOA_MODE_BIT_MASK           0x30000UL
#define SYS_CFG_CP_SDC_BIT_OFFSET            (15U)
#define SYS_CFG_CP_SDC_BIT_LEN               (1U)
#define SYS_CFG_CP_SDC_BIT_MASK              0x8000U
#define SYS_CFG_CP_SPC_BIT_OFFSET            (12U)
#define SYS_CFG_CP_SPC_BIT_LEN               (2U)
#define SYS_CFG_CP_SPC_BIT_MASK              0x3000U
#define SYS_CFG_AUTO_ACK_BIT_OFFSET          (11U)
#define SYS_CFG_AUTO_ACK_BIT_LEN             (1U)
#define SYS_CFG_AUTO_ACK_BIT_MASK            0x800U
#define SYS_CFG_RXAUTR_BIT_OFFSET            (10U)
#define SYS_CFG_RXAUTR_BIT_LEN               (1U)
#define SYS_CFG_RXAUTR_BIT_MASK              0x400U
#define SYS_CFG_RXWTOE_BIT_OFFSET            (9U)
#define SYS_CFG_RXWTOE_BIT_LEN               (1U)
#define SYS_CFG_RXWTOE_BIT_MASK              0x200U
#define SYS_CFG_CIA_STS_BIT_OFFSET           (8U)
#define SYS_CFG_CIA_STS_BIT_LEN              (1U)
#define SYS_CFG_CIA_STS_BIT_MASK             0x100U
#define SYS_CFG_CIA_IPATOV_BIT_OFFSET        (7U)
#define SYS_CFG_CIA_IPATOV_BIT_LEN           (1U)
#define SYS_CFG_CIA_IPATOV_BIT_MASK          0x80U
#define SYS_CFG_SPI_CRC_BIT_OFFSET           (6U)
#define SYS_CFG_SPI_CRC_BIT_LEN              (1U)
#define SYS_CFG_SPI_CRC_BIT_MASK             0x40U
#define SYS_CFG_PHR_6M8_BIT_OFFSET           (5U)
#define SYS_CFG_PHR_6M8_BIT_LEN              (1U)
#define SYS_CFG_PHR_6M8_BIT_MASK             0x20U
#define SYS_CFG_PHR_MODE_BIT_OFFSET          (4U)
#define SYS_CFG_PHR_MODE_BIT_LEN             (1U)
#define SYS_CFG_PHR_MODE_BIT_MASK            0x10U
#define SYS_CFG_DIS_DRXB_BIT_OFFSET          (3U)
#define SYS_CFG_DIS_DRXB_BIT_LEN             (1U)
#define SYS_CFG_DIS_DRXB_BIT_MASK            0x8U
#define SYS_CFG_DIS_FCE_BIT_OFFSET           (2U)
#define SYS_CFG_DIS_FCE_BIT_LEN              (1U)
#define SYS_CFG_DIS_FCE_BIT_MASK             0x4U
#define SYS_CFG_DIS_FCS_TX_BIT_OFFSET        (1U)
#define SYS_CFG_DIS_FCS_TX_BIT_LEN           (1U)
#define SYS_CFG_DIS_FCS_TX_BIT_MASK          0x2U
#define SYS_CFG_FFEN_BIT_OFFSET              (0U)
#define SYS_CFG_FFEN_BIT_LEN                 (1U)
#define SYS_CFG_FFEN_BIT_MASK                0x1U

/******************************************************************************
* @brief Bit definitions for register RX_CAL_RESI
**/
#define RX_CAL_RESI_ID                       0x40014
#define RX_CAL_RESI_LEN                      (4U)
#define RX_CAL_RESI_MASK                     0xFFFFFFFFUL


/******************************************************************************
* @brief Bit definitions for register RX_CAL_RESQ
**/
#define RX_CAL_RESQ_ID                       0x4001c
#define RX_CAL_RESQ_LEN                      (4U)
#define RX_CAL_RESQ_MASK                     0xFFFFFFFFUL

/******************************************************************************
* @brief Bit definitions for register RX_CAL_STS
**/
#define RX_CAL_STS_ID                       0x40020
#define RX_CAL_STS_LEN                      (4U)
#define RX_CAL_STS_MASK                     0xFFFFFFFFUL

/******************************************************************************
* @brief Bit definitions for register RX_CAL_CFG
**/
#define RX_CAL_CFG_ID                       0x4000c
#define RX_CAL_CFG_LEN                      (4U)
#define RX_CAL_CFG_MASK                     0xFFFFFFFFUL
#define RX_CAL_CFG_COMP_DLY_BIT_OFFSET      (16U)
#define RX_CAL_CFG_COMP_DLY_BIT_LEN         (4U)
#define RX_CAL_CFG_COMP_DLY_BIT_MASK        0xf0000UL
#define RX_CAL_CFG_CAL_EN_BIT_OFFSET        (4U)
#define RX_CAL_CFG_CAL_EN_BIT_LEN           (1U)
#define RX_CAL_CFG_CAL_EN_BIT_MASK          0x10U
#define RX_CAL_CFG_CAL_MODE_BIT_OFFSET      (0U)
#define RX_CAL_CFG_CAL_MODE_BIT_LEN         (2U)
#define RX_CAL_CFG_CAL_MODE_BIT_MASK        0x3U

// dwt_readcarrierintegrator defines
#define B20_SIGN_EXTEND_TEST (0x00100000UL)
#define B20_SIGN_EXTEND_MASK (0xFFF00000UL)
#define DRX_CARRIER_INT_LEN  (3)

#define CIA_MANUALLOWERBOUND_TH_64  (0x10) //cia lower bound threshold values for 64 MHz PRF
#define STSQUAL_THRESH_64 (0.90f)

//! constants for specifying TX Preamble length in symbols
//! These are defined to allow them be directly written into byte 2 of the TX_FCTRL register
//! (i.e. a four bit value destined for bits 20..18 but shifted left by 2 for byte alignment)
#define DWT_PLEN_4096   0x03    //! Standard preamble length 4096 symbols
#define DWT_PLEN_2048   0x0A    //! Non-standard preamble length 2048 symbols
#define DWT_PLEN_1536   0x06    //! Non-standard preamble length 1536 symbols
#define DWT_PLEN_1024   0x02    //! Standard preamble length 1024 symbols
#define DWT_PLEN_512    0x0d    //! Non-standard preamble length 512 symbols
#define DWT_PLEN_256    0x09    //! Non-standard preamble length 256 symbols
#define DWT_PLEN_128    0x05    //! Non-standard preamble length 128 symbols
#define DWT_PLEN_64     0x01    //! Standard preamble length 64 symbols
#define DWT_PLEN_32     0x04    //! Non-standard length 32
#define DWT_PLEN_72     0x07    //! Non-standard length 72

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

#define DWT_PHRMODE_STD         0x0     // standard PHR mode
#define DWT_PHRMODE_EXT         0x1     // DW proprietary extended frames PHR mode

#define ERR_RX_CAL_FAIL        0x1fffffff

/* Channel number */
#define DWT_CHANNEL_5 5

/******************************************************************************
* @brief Bit definitions for register CHAN_CTRL
**/
#define CHAN_CTRL_ID                         0x10014
#define CHAN_CTRL_LEN                        (4U)
#define CHAN_CTRL_MASK                       0xFFFFFFFFUL
#define CHAN_CTRL_RX_PCODE_BIT_OFFSET        (8U)
#define CHAN_CTRL_RX_PCODE_BIT_LEN           (5U)
#define CHAN_CTRL_RX_PCODE_BIT_MASK          0x1f00U
#define CHAN_CTRL_TX_PCODE_BIT_OFFSET        (3U)
#define CHAN_CTRL_TX_PCODE_BIT_LEN           (5U)
#define CHAN_CTRL_TX_PCODE_BIT_MASK          0xf8U
#define CHAN_CTRL_SFD_TYPE_BIT_OFFSET        (1U)
#define CHAN_CTRL_SFD_TYPE_BIT_LEN           (2U)
#define CHAN_CTRL_SFD_TYPE_BIT_MASK          0x6U
#define CHAN_CTRL_RF_CHAN_BIT_OFFSET         (0U)
#define CHAN_CTRL_RF_CHAN_BIT_LEN            (1U)
#define CHAN_CTRL_RF_CHAN_BIT_MASK           0x1U

#define SQRT_FACTOR             181 /*Factor of sqrt(2) for calculation*/
#define STS_LEN_SUPPORTED       7   /*The supported STS length options*/
#define SQRT_SHIFT_VAL          7
#define SHIFT_VALUE             11
#define MOD_VALUE               2048
#define HALF_MOD                (MOD_VALUE>>1)

/******************************************************************************
* @brief Bit definitions for register TX_FCTRL
**/
#define TX_FCTRL_ID                          0x24
#define TX_FCTRL_LEN                         (4U)
#define TX_FCTRL_MASK                        0xFFFFFFFFUL
#define TX_FCTRL_TXB_OFFSET_BIT_OFFSET       (16U)
#define TX_FCTRL_TXB_OFFSET_BIT_LEN          (10U)
#define TX_FCTRL_TXB_OFFSET_BIT_MASK         0x3ff0000UL
#define TX_FCTRL_TXPSR_BIT_OFFSET         (12U)
#define TX_FCTRL_TXPSR_BIT_LEN            (4U)
#define TX_FCTRL_TXPSR_BIT_MASK           0xf000U
#define TX_FCTRL_TR_BIT_OFFSET               (11U)
#define TX_FCTRL_TR_BIT_LEN                  (1U)
#define TX_FCTRL_TR_BIT_MASK                 0x800U
#define TX_FCTRL_TXBR_BIT_OFFSET             (10U)
#define TX_FCTRL_TXBR_BIT_LEN                (1U)
#define TX_FCTRL_TXBR_BIT_MASK               0x400U
#define TX_FCTRL_TXFLEN_BIT_OFFSET           (0U)
#define TX_FCTRL_TXFLEN_BIT_LEN              (10U)
#define TX_FCTRL_TXFLEN_BIT_MASK             0x3ffU


#define RF_TXCTRL_CH5           0x1C071134UL    /* */
#define RF_TXCTRL_CH9           0x1C010034UL    /* */
#define RF_TXCTRL_LO_B2         0x0E            /* */
#define RF_RXCTRL_CH9           0x08B5A833UL    /* */
#define RF_PLL_CFG_CH5          0x1F3C
#define RF_PLL_CFG_CH9          0x0F3C
#define RF_PLL_CFG_LD           0x81
#define LDO_RLOAD_VAL_B1        0x14

/******************************************************************************
* @brief Bit definitions for register PLL_CFG
**/
#define PLL_CFG_ID                           0x90000
#define PLL_CFG_LEN                          (4U)
#define PLL_CFG_MASK                         0xFFFFFFFFUL

/******************************************************************************
* @brief Bit definitions for register LDO_RLOAD
**/
#define LDO_RLOAD_ID                         0x70050
#define LDO_RLOAD_LEN                        (4U)
#define LDO_RLOAD_MASK                       0xFFFFFFFFUL


#define TX_CTRL_LO_ID                        0x70018
#define TX_CTRL_LO_LEN                       (4U)
#define TX_CTRL_LO_MASK                      0xFFFFFFFFUL

/******************************************************************************
* @brief Bit definitions for register PLL_CAL
**/
#define PLL_CAL_ID                           0x90008
#define PLL_CAL_LEN                          (4U)
#define PLL_CAL_MASK                         0xFFFFFFFFUL



#define DWT_SFDTOC_DEF          129  // default SFD timeout value

/******************************************************************************
* @brief Bit definitions for register TX_CTRL_HI
**/
#define TX_CTRL_HI_ID                        0x7001c
#define TX_CTRL_HI_LEN                       (4U)
#define TX_CTRL_HI_MASK                      0xFFFFFFFFUL
#define TX_CTRL_HI_TX_PG_DELAY_BIT_OFFSET    (0U)
#define TX_CTRL_HI_TX_PG_DELAY_BIT_LEN       (6U)
#define TX_CTRL_HI_TX_PG_DELAY_BIT_MASK      0x3fU


//! constants for specifying Preamble Acquisition Chunk (PAC) Size in symbols
#define DWT_PAC8        0   //!< PAC  8 (recommended for RX of preamble length  128 and below
#define DWT_PAC16       1   //!< PAC 16 (recommended for RX of preamble length  256
#define DWT_PAC32       2   //!< PAC 32 (recommended for RX of preamble length  512
#define DWT_PAC4        3   //!< PAC  4 (recommended for RX of preamble length  < 127

//! constants for specifying SFD Types and size
#define DWT_SFD_IEEE_4A 0   //!< IEEE 8-bit ternary
#define DWT_SFD_DW_8    1   //!< DW 8-bit
#define DWT_SFD_DW_16   2   //!< DW 16-bit
#define DWT_SFD_IEEE_4Z 3   //!< IEEE 8-bit binary (4z)
#define DWT_SFD_LEN8    (8) //!< IEEE, and DW 8-bit are length 8
#define DWT_SFD_LEN16   (16)//!< DW 16-bit is length 16

typedef enum
{
    DWT_STS_LEN_32  =0,
    DWT_STS_LEN_64  =1,
    DWT_STS_LEN_128 =2,
    DWT_STS_LEN_256 =3,
    DWT_STS_LEN_512 =4,
    DWT_STS_LEN_1024=5,
    DWT_STS_LEN_2048=6
} dwt_sts_lengths_e;

/* TX/RX preamble code */
#define DWT_PRF_64M 9

/* SFD mode */
#define DWT_SFD_NON_STD_8 1

//! constants for selecting the bit rate for data TX (and RX)
//! These are defined for write (with just a shift) the TX_FCTRL register
#define DWT_BR_850K     0   //!< UWB bit rate 850 kbits/s
#define DWT_BR_6M8      1   //!< UWB bit rate 6.8 Mbits/s
#define DWT_BR_NODATA   2   //!< No data (SP3 packet format)


#define DWT_PHRRATE_STD         0x0     // standard PHR rate
#define DWT_PHRRATE_DTA         0x1     // PHR at data rate (6M81)

// Define DW3000 STS modes
#define DWT_STS_MODE_OFF         0x0     // STS is off
#define DWT_STS_MODE_1           0x1     // STS mode 1
#define DWT_STS_MODE_2           0x2     // STS mode 2
#define DWT_STS_MODE_ND          0x3     // STS with no data
#define DWT_STS_MODE_SDC         0x8     // Enable Super Deterministic Codes
#define DWT_STS_CONFIG_MASK      0xB

/******************************************************************************
* @brief Bit definitions for register DTUNE3
**/
#define DTUNE3_ID                            0x6000c
#define DTUNE3_LEN                           (4U)
#define DTUNE3_MASK                          0xFFFFFFFFUL



/******************************************************************************
* @brief Bit definitions for register STS_CFG0
**/
#define STS_CFG0_ID                          0x20000
#define STS_CFG0_LEN                         (4U)
#define STS_CFG0_MASK                        0xFFFFFFFFUL
#define STS_CFG0_CPS_LEN_BIT_OFFSET          (0U)
#define STS_CFG0_CPS_LEN_BIT_LEN             (8U)
#define STS_CFG0_CPS_LEN_BIT_MASK            0xffU


//DW3000 OTP operating parameter set selection
#define DWT_OPSET_LONG   (0x0<<11)
#define DWT_OPSET_SCP    (0x1<<11)
#define DWT_OPSET_SHORT  (0x2<<11)

#define IP_CONFIG_LO_SCP        0x0306
#define IP_CONFIG_HI_SCP        0x00000000
#define STS_CONFIG_LO_SCP       0x000C5A0A
#define STS_CONFIG_HI_SCP       0x7D

/******************************************************************************
* @brief Bit definitions for register STS_CONFIG_LO
**/
#define STS_CONFIG_LO_ID                      0xe0012              /*  { aliased = true}  */
#define STS_CONFIG_LO_LEN                     (4U)
#define STS_CONFIG_LO_MASK                    0xFFFFFFFFUL
#define STS_CONFIG_LO_STS_MAN_TH_BIT_OFFSET (16U)
#define STS_CONFIG_LO_STS_MAN_TH_BIT_LEN (7U)
#define STS_CONFIG_LO_STS_MAN_TH_BIT_MASK 0x7f0000UL
#define STS_CONFIG_LO_STS_PMULT_BIT_OFFSET (5U)
#define STS_CONFIG_LO_STS_PMULT_BIT_LEN  (2U)
#define STS_CONFIG_LO_STS_PMULT_BIT_MASK 0x60U
#define STS_CONFIG_LO_STS_NTM_BIT_OFFSET (0U)
#define STS_CONFIG_LO_STS_NTM_BIT_LEN (5U)
#define STS_CONFIG_LO_STS_NTM_BIT_MASK 0x1fU

/******************************************************************************
* @brief Bit definitions for register STS_CONFIG_HI
**/
#define STS_CONFIG_HI_ID                      0xe0016              /*  { aliased = true}  */
#define STS_CONFIG_HI_LEN                     (4U)
#define STS_CONFIG_HI_MASK                    0xFFFFFFFFUL
#define STS_CONFIG_HI_STS_PGR_EN_BIT_OFFSET (31U)
#define STS_CONFIG_HI_STS_PGR_EN_BIT_LEN  (1U)
#define STS_CONFIG_HI_STS_PGR_EN_BIT_MASK 0x80000000UL
#define STS_CONFIG_HI_STS_SS_EN_BIT_OFFSET (30U)
#define STS_CONFIG_HI_STS_SS_EN_BIT_LEN    (1U)
#define STS_CONFIG_HI_STS_SS_EN_BIT_MASK   0x40000000UL
#define STS_CONFIG_HI_STS_CQ_EN_BIT_OFFSET (29U)
#define STS_CONFIG_HI_STS_CQ_EN_BIT_LEN    (1U)
#define STS_CONFIG_HI_STS_CQ_EN_BIT_MASK   0x20000000UL
#define STS_CONFIG_HI_FP_AGREED_EN_BIT_OFFSET (28U)
#define STS_CONFIG_HI_FP_AGREED_EN_BIT_LEN    (1U)
#define STS_CONFIG_HI_FP_AGREED_EN_BIT_MASK   0x10000000UL


/******************************************************************************
* @brief Bit definitions for register IP_CONFIG_LO
**/
#define IP_CONFIG_LO_ID                      0xe000c
#define IP_CONFIG_LO_LEN                     (2U)
#define IP_CONFIG_LO_MASK                    0xFFFFU
#define IP_CONFIG_LO_IP_PMULT_BIT_OFFSET (5U)
#define IP_CONFIG_LO_IP_PMULT_BIT_LEN  (2U)
#define IP_CONFIG_LO_IP_PMULT_BIT_MASK 0x60U
#define IP_CONFIG_LO_IP_NTM_BIT_OFFSET (0U)
#define IP_CONFIG_LO_IP_NTM_BIT_LEN (5U)
#define IP_CONFIG_LO_IP_NTM_BIT_MASK 0x1fU

// Defines for enable_clocks function
#define FORCE_CLK_SYS_TX        (1)
#define FORCE_CLK_AUTO          (5)

/*
	Lookup table default values for channel 5
*/
typedef enum
{
    CH5_DGC_LUT_0 = 0x1c0fd,
    CH5_DGC_LUT_1 = 0x1c43e,
    CH5_DGC_LUT_2 = 0x1c6be,
    CH5_DGC_LUT_3 = 0x1c77e,
    CH5_DGC_LUT_4 = 0x1cf36,
    CH5_DGC_LUT_5 = 0x1cfb5,
    CH5_DGC_LUT_6 = 0x1cff5
} dwt_configmrxlut_ch5_e;

/*
	Lookup table default values for channel 9
*/
typedef enum
{
    CH9_DGC_LUT_0 = 0x2a8fe,
    CH9_DGC_LUT_1 = 0x2ac36,
    CH9_DGC_LUT_2 = 0x2a5fe,
    CH9_DGC_LUT_3 = 0x2af3e,
    CH9_DGC_LUT_4 = 0x2af7d,
    CH9_DGC_LUT_5 = 0x2afb5,
    CH9_DGC_LUT_6 = 0x2afb5
} dwt_configmrxlut_ch9_e;


/******************************************************************************
* @brief Bit definitions for register DGC_CFG0
**/
#define DGC_CFG0_ID                          0x3001c
#define DGC_CFG0_LEN                         (4U)
#define DGC_CFG0_MASK                        0xFFFFFFFFUL

/******************************************************************************
* @brief Bit definitions for register DGC_CFG1
**/
#define DGC_CFG1_ID                          0x30020
#define DGC_CFG1_LEN                         (4U)
#define DGC_CFG1_MASK                        0xFFFFFFFFUL

/******************************************************************************
* @brief Bit definitions for register DGC_LUT_0_CFG
**/
#define DGC_LUT_0_CFG_ID                 0x30038
#define DGC_LUT_0_CFG_LEN                (4U)
#define DGC_LUT_0_CFG_MASK               0xFFFFFFFFUL

/******************************************************************************
* @brief Bit definitions for register DGC_LUT_1_CFG
**/
#define DGC_LUT_1_CFG_ID                 0x3003c
#define DGC_LUT_1_CFG_LEN                (4U)
#define DGC_LUT_1_CFG_MASK               0xFFFFFFFFUL

/******************************************************************************
* @brief Bit definitions for register DGC_LUT_2_CFG
**/
#define DGC_LUT_2_CFG_ID                 0x30040
#define DGC_LUT_2_CFG_LEN                (4U)
#define DGC_LUT_2_CFG_MASK               0xFFFFFFFFUL

/******************************************************************************
* @brief Bit definitions for register DGC_LUT_3_CFG
**/
#define DGC_LUT_3_CFG_ID                 0x30044
#define DGC_LUT_3_CFG_LEN                (4U)
#define DGC_LUT_3_CFG_MASK               0xFFFFFFFFUL

/******************************************************************************
* @brief Bit definitions for register DGC_LUT_4_CFG
**/
#define DGC_LUT_4_CFG_ID                 0x30048
#define DGC_LUT_4_CFG_LEN                (4U)
#define DGC_LUT_4_CFG_MASK               0xFFFFFFFFUL

/******************************************************************************
* @brief Bit definitions for register DGC_LUT_5_CFG
**/
#define DGC_LUT_5_CFG_ID                 0x3004c
#define DGC_LUT_5_CFG_LEN                (4U)
#define DGC_LUT_5_CFG_MASK               0xFFFFFFFFUL

/******************************************************************************
* @brief Bit definitions for register DGC_LUT_6_CFG
**/
#define DGC_LUT_6_CFG_ID                 0x30050
#define DGC_LUT_6_CFG_LEN                (4U)
#define DGC_LUT_6_CFG_MASK               0xFFFFFFFFUL


/******************************************************************************
* @brief Bit definitions for register SEQ_CTRL
**/
#define SEQ_CTRL_ID                          0x110008
#define SEQ_CTRL_LEN                         (4U)
#define SEQ_CTRL_MASK                        0xFFFFFFFFUL
#define SEQ_CTRL_LP_CLK_DIV_BIT_OFFSET       (26U)
#define SEQ_CTRL_LP_CLK_DIV_BIT_LEN          (6U)
#define SEQ_CTRL_LP_CLK_DIV_BIT_MASK         0xfc000000UL
#define SEQ_CTRL_FORCE2INIT_BIT_OFFSET       (23U)
#define SEQ_CTRL_FORCE2INIT_BIT_LEN          (1U)
#define SEQ_CTRL_FORCE2INIT_BIT_MASK         0x800000UL
#define SEQ_CTRL_CIA_SEQ_EN_BIT_OFFSET       (17U)
#define SEQ_CTRL_CIA_SEQ_EN_BIT_LEN          (1U)
#define SEQ_CTRL_CIA_SEQ_EN_BIT_MASK         0x20000UL
#define SEQ_CTRL_PLL_SYNC_MODE_BIT_OFFSET    (15U)
#define SEQ_CTRL_PLL_SYNC_MODE_BIT_LEN       (1U)
#define SEQ_CTRL_PLL_SYNC_MODE_BIT_MASK      0x8000U
#define SEQ_CTRL_ARX2SLP_BIT_OFFSET      (12U)
#define SEQ_CTRL_ARX2SLP_BIT_LEN         (1U)
#define SEQ_CTRL_ARX2SLP_BIT_MASK        0x1000U
#define SEQ_CTRL_ATX2SLP_BIT_OFFSET      (11U)
#define SEQ_CTRL_ATX2SLP_BIT_LEN         (1U)
#define SEQ_CTRL_ATX2SLP_BIT_MASK        0x800U
#define SEQ_CTRL_AINIT2IDLE_BIT_OFFSET   (8U)
#define SEQ_CTRL_AINIT2IDLE_BIT_LEN      (1U)
#define SEQ_CTRL_AINIT2IDLE_BIT_MASK     0x100U

//SYSCLK
#define FORCE_SYSCLK_PLL        (2)
#define FORCE_SYSCLK_FOSCDIV4   (1)

/******************************************************************************
* @brief Bit definitions for register RX_CTRL_HI
**/
#define RX_CTRL_HI_ID                        0x70010
#define RX_CTRL_HI_LEN                       (4U)
#define RX_CTRL_HI_MASK                      0xFFFFFFFFUL

/******************************************************************************
* @brief Bit definitions for register TX_FCTRL_HI
**/
#define TX_FCTRL_HI_ID                       0x28
#define TX_FCTRL_HI_LEN                      (4U)
#define TX_FCTRL_HI_MASK                     0xFFFFFFFFUL
#define TX_FCTRL_HI_FINE_PLEN_BIT_OFFSET     (8U)
#define TX_FCTRL_HI_FINE_PLEN_BIT_LEN        (8U)
#define TX_FCTRL_HI_FINE_PLEN_BIT_MASK       0xff00U

#define dwt_and8bitoffsetreg(ctx, addr, offset, and_val) dwt_modify8bitoffsetreg(ctx, addr, offset, and_val, 0)

/******************************************************************************
* @brief Bit definitions for register IP_CONFIG_HI
**/
#define IP_CONFIG_HI_ID                      0xe000e              /*  { aliased = true}  */
#define IP_CONFIG_HI_LEN                     (4U)
#define IP_CONFIG_HI_MASK                    0xFFFFFFFFUL
#define IP_CONFIG_HI_IP_RTM_BIT_OFFSET (0U)
#define IP_CONFIG_HI_IP_RTM_BIT_LEN (5U)
#define IP_CONFIG_HI_IP_RTM_BIT_MASK 0x1fU

/* Enum used for selecting channel for DGC on-wake kick. */
typedef enum
{
    DWT_DGC_SEL_CH5=0,
    DWT_DGC_SEL_CH9
} dwt_dgc_chan_sel;

/******************************************************************************
* @brief Bit definitions for register DTUNE0
**/
#define DTUNE0_ID                            0x60000
#define DTUNE0_LEN                           (4U)
#define DTUNE0_MASK                          0xFFFFFFFFUL
#define DTUNE0_RX_SFD_TOC_BIT_OFFSET         (16U)
#define DTUNE0_RX_SFD_TOC_BIT_LEN            (16U)
#define DTUNE0_RX_SFD_TOC_BIT_MASK           0xffff0000UL
#define DTUNE0_PRE_PAC_SYM_BIT_OFFSET        (0U)
#define DTUNE0_PRE_PAC_SYM_BIT_LEN           (2U)
#define DTUNE0_PRE_PAC_SYM_BIT_MASK          0x3U

// Define DW3000 PDOA modes
#define DWT_PDOA_M0           0x0     // DW PDOA mode is off
#define DWT_PDOA_M1           0x1     // DW PDOA mode  mode 1
#define DWT_PDOA_M3           0x3     // DW PDOA mode  mode 3

/* LNA/PA and LED modes */
#define DWT_LNA_ENABLE 0x01
#define DWT_PA_ENABLE 0x02
#define DWT_LEDS_ENABLE 0x01
#define DWT_LEDS_INIT_BLINK 0x02

#define DWT_AUTO_CLKS          (0x200 | 0x200000 | 0x100000)

#define FORCE_CLK_PLL           (2)

/******************************************************************************
* @brief Bit definitions for register CLK_CTRL
**/
#define CLK_CTRL_ID2                          0x110004
#define CLK_CTRL_LEN                         (4U)
#define CLK_CTRL_MASK                        0xFFFFFFFFUL
#define CLK_CTRL_LP_CLK_EN_BIT_OFFSET        (23U)
#define CLK_CTRL_LP_CLK_EN_BIT_LEN           (1U)
#define CLK_CTRL_LP_CLK_EN_BIT_MASK          0x800000UL
#define CLK_CTRL_GPIO_DRST_N_BIT_OFFSET  (19U)
#define CLK_CTRL_GPIO_DRST_N_BIT_LEN     (1U)
#define CLK_CTRL_GPIO_DRST_N_BIT_MASK    0x80000UL
#define CLK_CTRL_GPIO_DCLK_EN_BIT_OFFSET (18U)
#define CLK_CTRL_GPIO_DCLK_EN_BIT_LEN    (1U)
#define CLK_CTRL_GPIO_DCLK_EN_BIT_MASK   0x40000UL
#define CLK_CTRL_GPIO_CLK_EN_BIT_OFFSET      (16U)
#define CLK_CTRL_GPIO_CLK_EN_BIT_LEN         (1U)
#define CLK_CTRL_GPIO_CLK_EN_BIT_MASK        0x10000UL
#define CLK_CTRL_ACC_MCLK_EN_BIT_OFFSET   (15U)
#define CLK_CTRL_ACC_MCLK_EN_BIT_LEN      (1U)
#define CLK_CTRL_ACC_MCLK_EN_BIT_MASK     0x8000U
#define CLK_CTRL_TX_BUF_CLK_ON_BIT_OFFSET    (12U)
#define CLK_CTRL_TX_BUF_CLK_ON_BIT_LEN       (1U)
#define CLK_CTRL_TX_BUF_CLK_ON_BIT_MASK      0x1000U
#define CLK_CTRL_RX_BUF_CLK_ON_BIT_OFFSET    (11U)
#define CLK_CTRL_RX_BUF_CLK_ON_BIT_LEN       (1U)
#define CLK_CTRL_RX_BUF_CLK_ON_BIT_MASK      0x800U
#define CLK_CTRL_SAR_CLK_EN_BIT_OFFSET (10U)
#define CLK_CTRL_SAR_CLK_EN_BIT_LEN    (1U)
#define CLK_CTRL_SAR_CLK_EN_BIT_MASK   0x400U
#define CLK_CTRL_OTP_CLK_EN_BIT_OFFSET (9U)
#define CLK_CTRL_OTP_CLK_EN_BIT_LEN    (1U)
#define CLK_CTRL_OTP_CLK_EN_BIT_MASK   0x200U
#define CLK_CTRL_CIA_CLK_EN_BIT_OFFSET (8U)
#define CLK_CTRL_CIA_CLK_EN_BIT_LEN   (1U)
#define CLK_CTRL_CIA_CLK_EN_BIT_MASK  0x100U
#define CLK_CTRL_ACC_CLK_EN_BIT_OFFSET    (6U)
#define CLK_CTRL_ACC_CLK_EN_BIT_LEN       (1U)
#define CLK_CTRL_ACC_CLK_EN_BIT_MASK      0x40U
#define CLK_CTRL_TX_CLK_SEL_BIT_OFFSET       (4U)
#define CLK_CTRL_TX_CLK_SEL_BIT_LEN          (2U)
#define CLK_CTRL_TX_CLK_SEL_BIT_MASK         0x30U
#define CLK_CTRL_RX_CLK_SEL_BIT_OFFSET       (2U)
#define CLK_CTRL_RX_CLK_SEL_BIT_LEN          (2U)
#define CLK_CTRL_RX_CLK_SEL_BIT_MASK         0xcU
#define CLK_CTRL_SYS_CLK_SEL_BIT_OFFSET      (0U)
#define CLK_CTRL_SYS_CLK_SEL_BIT_LEN         (2U)
#define CLK_CTRL_SYS_CLK_SEL_BIT_MASK        0x3U

#define FORCE_SYSCLK_FOSC2       (3)

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

#define GET_STS_REG_SET_VALUE(x)     ((uint16_t)1<<((x)+2))    /* Returns the value to set in CP_CFG0_ID for STS length. The x is the enum value from dwt_sts_lengths_e */

#define originaldwt_or8bitoffsetreg(ctx, addr, offset, or_val) dwt_modify8bitoffsetreg(ctx, addr, offset, -1, or_val)

/* Structure definitions from Decawave example */
typedef struct
{
    uint8_t chan ;           //!< Channel number (5 or 9)
    uint8_t txPreambLength ; //!< DWT_PLEN_64..DWT_PLEN_4096
    uint8_t rxPAC ;          //!< Acquisition Chunk Size (Relates to RX preamble length)
    uint8_t txCode ;         //!< TX preamble code (the code configures the PRF, e.g. 9 -> PRF of 64 MHz)
    uint8_t rxCode ;         //!< RX preamble code (the code configures the PRF, e.g. 9 -> PRF of 64 MHz)
    uint8_t sfdType;         //!< SFD type (0 for short IEEE 8-bit standard, 1 for DW 8-bit, 2 for DW 16-bit, 3 for 4z BPRF)
    uint8_t dataRate ;       //!< Data rate {DWT_BR_850K or DWT_BR_6M8}
    uint8_t phrMode ;        //!< PHR mode {0x0 - standard DWT_PHRMODE_STD, 0x3 - extended frames DWT_PHRMODE_EXT}
    uint8_t phrRate;         //!< PHR rate {0x0 - standard DWT_PHRRATE_STD, 0x1 - at datarate DWT_PHRRATE_DTA}
    uint16_t sfdTO ;         //!< SFD timeout value (in symbols)
    uint8_t stsMode;         //!< STS mode (no STS, STS before PHR or STS after data)
    dwt_sts_lengths_e stsLength;    //!< STS length (the allowed values are listed in dwt_sts_lengths_e
    uint8_t pdoaMode;        //!< PDOA mode
} dwt_config_t ;


typedef struct
{
    uint8_t   PGdly;
    //TX POWER
    //31:24     TX_CP_PWR
    //23:16     TX_SHR_PWR
    //15:8      TX_PHR_PWR
    //7:0       TX_DATA_PWR
    uint32_t  power;
    uint16_t  PGcount;
} dwt_txconfig_t ;


/******************************************************************************
* @brief Bit definitions for register TX_POWER
**/
#define TX_POWER_ID                          0x1000c
#define TX_POWER_LEN                         (4U)
#define TX_POWER_MASK                        0xFFFFFFFFUL

/******************************************************************************
* @brief Bit definitions for register PG_CAL_TARGET
**/
#define PG_CAL_TARGET_ID                     0x8001c
#define PG_CAL_TARGET_LEN                    (4U)
#define PG_CAL_TARGET_MASK                   0xFFFFFFFFUL
#define PG_CAL_TARGET_TARGET_BIT_OFFSET      (0U)
#define PG_CAL_TARGET_TARGET_BIT_LEN         (12U)
#define PG_CAL_TARGET_TARGET_BIT_MASK        0xfffU

/******************************************************************************
* @brief Bit definitions for register RF_CTRL_MASK
**/
#define RF_CTRL_MASK_ID                      0x70004
#define RF_CTRL_MASK_LEN                     (4U)
#define RF_CTRL_MASK_MASK                    0xFFFFFFFFUL

#define SYS_STATE_LO_ID                      0xf0030
#define SYS_STATE_LO_LEN                     (4U)
#define SYS_STATE_LO_MASK                    0xFFFFFFFFUL


// SYS_STATE_LO register errors
#define DW_SYS_STATE_TXERR          0xD0000         // TSE is in TX but TX is in IDLE in SYS_STATE_LO register


//! fast commands
#define CMD_DB_TOGGLE     0x13   //!< Toggle double buffer pointer
#define CMD_CLR_IRQS      0x12   //!< Clear all events/clear interrupt
#define CMD_CCA_TX_W4R    0x11   //!< Check if channel clear prior to TX, enable RX when TX done
#define CMD_DTX_REF_W4R   0x10   //!< Start delayed TX (as DTX_REF below), enable RX when TX done
#define CMD_DTX_RS_W4R    0xF    //!< Start delayed TX (as DTX_RS below), enable RX when TX done
#define CMD_DTX_TS_W4R    0xE    //!< Start delayed TX (as DTX_TS below), enable RX when TX done
#define CMD_DTX_W4R       0xD    //!< Start delayed TX (as DTX below), enable RX when TX done
#define CMD_TX_W4R        0xC    //!< Start TX (as below), enable RX when TX done
#define CMD_CCA_TX        0xB    //!< Check if channel clear prior to TX
#define CMD_DRX_REF       0xA    //!< Enable RX @ time = DREF_TIME + DX_TIME
#define CMD_DTX_REF       0x9    //!< Start delayed TX (RMARKER will be @ time = DREF_TIME + DX_TIME)
#define CMD_DRX_RS        0x8    //!< Enable RX @ time = RX_TIME + DX_TIME
#define CMD_DTX_RS        0x7    //!< Start delayed TX (RMARKER will be @ time = RX_TIME + DX_TIME)
#define CMD_DRX_TS        0x6    //!< Enable RX @ time = TX_TIME + DX_TIME
#define CMD_DTX_TS        0x5    //!< Start delayed TX (RMARKER will be @ time = TX_TIME + DX_TIME)
#define CMD_DRX           0x4    //!< Enable RX @ time specified in DX_TIME register
#define CMD_DTX           0x3    //!< Start delayed TX (RMARKER will be @ time set in DX_TIME register)
#define CMD_RX            0x2    //!< Enable RX
#define CMD_TX            0x1    //!< Start TX
#define CMD_TXRXOFF       0x0    //!< Turn off TX or RX, clear any TX/RX events and put DW3000 into IDLE


/******************************************************************************
* @brief Bit definitions for register RF_ENABLE
**/
#define RF_ENABLE_ID                            0x70000
#define RF_ENABLE_LEN                           (4U)
#define RF_ENABLE_MASK                          0xFFFFFFFFUL
#define RF_ENABLE_TX_SW_EN_BIT_OFFSET           (25U)
#define RF_ENABLE_TX_SW_EN_BIT_LEN              (1U)
#define RF_ENABLE_TX_SW_EN_BIT_MASK             0x2000000UL
#define RF_ENABLE_TX_CH5_BIT_OFFSET             (13U)
#define RF_ENABLE_TX_CH5_BIT_LEN                (1U)
#define RF_ENABLE_TX_CH5_BIT_MASK               0x2000U
#define RF_ENABLE_TX_EN_BIT_OFFSET              (12U)
#define RF_ENABLE_TX_EN_BIT_LEN                 (1U)
#define RF_ENABLE_TX_EN_BIT_MASK                0x1000U
#define RF_ENABLE_TX_EN_BUF_BIT_OFFSET          (11U)
#define RF_ENABLE_TX_EN_BUF_BIT_LEN             (1U)
#define RF_ENABLE_TX_EN_BUF_BIT_MASK            0x800U
#define RF_ENABLE_TX_BIAS_EN_BIT_OFFSET         (10U)
#define RF_ENABLE_TX_BIAS_EN_BIT_LEN            (1U)
#define RF_ENABLE_TX_BIAS_EN_BIT_MASK           0x400U

/******************************************************************************
* @brief Bit definitions for register RF_SWITCH
**/
#define RF_SWITCH_CTRL_ID                         0x70014
#define RF_SWITCH_CTRL_LEN                        (4U)
#define RF_SWITCH_CTRL_MASK                       0xFFFFFFFFUL


/******************************************************************************
* @brief Bit definitions for register PGC_CTRL
**/
#define PGC_CTRL_ID                          0x80010
#define PGC_CTRL_LEN                         (4U)
#define PGC_CTRL_MASK                        0xFFFFFFFFUL
#define PGC_CTRL_PGC_AUTO_CAL_BIT_OFFSET     (1U)
#define PGC_CTRL_PGC_AUTO_CAL_BIT_LEN        (1U)
#define PGC_CTRL_PGC_AUTO_CAL_BIT_MASK       0x2U
#define PGC_CTRL_PGC_START_BIT_OFFSET        (0U)
#define PGC_CTRL_PGC_START_BIT_LEN           (1U)
#define PGC_CTRL_PGC_START_BIT_MASK          0x1U

#define TXRXSWITCH_TX           0x01011100
#define TXRXSWITCH_AUTO         0x1C000000

#define SEL_CHANNEL5            (5)
#define SEL_CHANNEL9            (9)

#define dwt_write32bitreg(ctx, addr,value)  dwt_write32bitoffsetreg(ctx, addr,0,value)
#define dwt_or32bitoffsetreg(ctx, addr, offset, or_val) dwt_modify32bitoffsetreg(ctx, addr, offset, -1, or_val)


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

/******************************************************************************
* @brief Bit definitions for register LDO_CTRL
**/
#define LDO_CTRL_ID                          0x70048
#define LDO_CTRL_LEN                         (4U)
#define LDO_CTRL_MASK                        0xFFFFFFFFUL
#define LDO_CTRL_LDO_VDDHVTX_VREF_BIT_OFFSET (27U)
#define LDO_CTRL_LDO_VDDHVTX_VREF_BIT_LEN    (1U)
#define LDO_CTRL_LDO_VDDHVTX_VREF_BIT_MASK   0x8000000UL
#define LDO_CTRL_LDO_VDDTX2_VREF_BIT_OFFSET  (22U)
#define LDO_CTRL_LDO_VDDTX2_VREF_BIT_LEN     (1U)
#define LDO_CTRL_LDO_VDDTX2_VREF_BIT_MASK    0x400000UL
#define LDO_CTRL_LDO_VDDTX1_VREF_BIT_OFFSET  (21U)
#define LDO_CTRL_LDO_VDDTX1_VREF_BIT_LEN     (1U)
#define LDO_CTRL_LDO_VDDTX1_VREF_BIT_MASK    0x200000UL
#define LDO_CTRL_LDO_VDDHVTX_EN_BIT_OFFSET   (11U)
#define LDO_CTRL_LDO_VDDHVTX_EN_BIT_LEN      (1U)
#define LDO_CTRL_LDO_VDDHVTX_EN_BIT_MASK     0x800U
#define LDO_CTRL_LDO_VDDIF2_EN_BIT_OFFSET    (8U)
#define LDO_CTRL_LDO_VDDIF2_EN_BIT_LEN       (1U)
#define LDO_CTRL_LDO_VDDIF2_EN_BIT_MASK      0x100U
#define LDO_CTRL_LDO_VDDTX2_EN_BIT_OFFSET    (6U)
#define LDO_CTRL_LDO_VDDTX2_EN_BIT_LEN       (1U)
#define LDO_CTRL_LDO_VDDTX2_EN_BIT_MASK      0x40U
#define LDO_CTRL_LDO_VDDTX1_EN_BIT_OFFSET    (5U)
#define LDO_CTRL_LDO_VDDTX1_EN_BIT_LEN       (1U)
#define LDO_CTRL_LDO_VDDTX1_EN_BIT_MASK      0x20U
#define LDO_CTRL_LDO_VDDPLL_EN_BIT_OFFSET    (4U)
#define LDO_CTRL_LDO_VDDPLL_EN_BIT_LEN       (1U)
#define LDO_CTRL_LDO_VDDPLL_EN_BIT_MASK      0x10U
#define LDO_CTRL_LDO_VDDMS3_EN_BIT_OFFSET    (2U)
#define LDO_CTRL_LDO_VDDMS3_EN_BIT_LEN       (1U)
#define LDO_CTRL_LDO_VDDMS3_EN_BIT_MASK      0x4U
#define LDO_CTRL_LDO_VDDMS2_EN_BIT_OFFSET    (1U)
#define LDO_CTRL_LDO_VDDMS2_EN_BIT_LEN       (1U)
#define LDO_CTRL_LDO_VDDMS2_EN_BIT_MASK      0x2U
#define LDO_CTRL_LDO_VDDMS1_EN_BIT_OFFSET    (0U)
#define LDO_CTRL_LDO_VDDMS1_EN_BIT_LEN       (1U)
#define LDO_CTRL_LDO_VDDMS1_EN_BIT_MASK      0x1U


// Defined constants for "mode" bitmask parameter passed into dwt_starttx() function.
#define DWT_START_TX_IMMEDIATE      0x00    //! Send the frame immediately
#define DWT_START_TX_DELAYED        0x01    //! Send the frame at specified time (time must be less that half period away)
#define DWT_RESPONSE_EXPECTED       0x02    //! Will enable the receiver after TX has completed
#define DWT_START_TX_DLY_REF        0x04    //! Send the frame at specified time (time in DREF_TIME register + any time in DX_TIME register)
#define DWT_START_TX_DLY_RS         0x08    //! Send the frame at specified time (time in RX_TIME_0 register + any time in DX_TIME register)
#define DWT_START_TX_DLY_TS         0x10    //! Send the frame at specified time (time in TX_TIME_LO register + any time in DX_TIME register)

#define DWT_START_TX_CCA            0x20    //! Send the frame if no preamble detected within PTO time

// Defined constants for "mode" bitmask parameter passed into dwt_rxenable() function.
#define DWT_START_RX_IMMEDIATE      0x00    //! Enable the receiver immediately
#define DWT_START_RX_DELAYED        0x01    //! Set up delayed RX, if "late" error triggers, then the RX will be enabled immediately
#define DWT_IDLE_ON_DLY_ERR         0x02    //! If delayed RX failed due to "late" error then if this
                                            //! flag is set the RX will not be re-enabled immediately, and device will be in IDLE when function exits
#define DWT_START_RX_DLY_REF        0x04    //! Enable the receiver at specified time (time in DREF_TIME register + any time in DX_TIME register)
#define DWT_START_RX_DLY_RS         0x08    //! Enable the receiver at specified time (time in RX_TIME_0 register + any time in DX_TIME register)
#define DWT_START_RX_DLY_TS         0x10    //! Enable the receiver at specified time (time in TX_TIME_LO register + any time in DX_TIME register)


/******************************************************************************
* @brief Bit definitions for register SAR_CTRL
**/
#define SAR_CTRL_ID                          0x80000
#define SAR_CTRL_LEN                         (4U)
#define SAR_CTRL_MASK                        0xFFFFFFFFUL
#define SAR_CTRL_SAR_START_BIT_OFFSET        (0U)
#define SAR_CTRL_SAR_START_BIT_LEN           (1U)
#define SAR_CTRL_SAR_START_BIT_MASK          0x1U

// LDO and BIAS tune kick
#define LDO_BIAS_KICK (0x180)  // Writing to bit 7 and 8

#define DWT_TX_BUFF_OFFSET_ADJUST  128 // TX buffer offset adjustment when txBufferOffset > 127

#define PANADR_PAN_ID_BYTE_OFFSET       2
#define PMSC_CTRL0_PLL2_SEQ_EN          0x01000000UL    /* Enable PLL2 on/off sequencing by SNIFF mode */
#define RX_BUFFER_MAX_LEN               (1023)
#define TX_BUFFER_MAX_LEN               (1024)
#define RX_FINFO_STD_RXFLEN_MASK        0x0000007FUL    /* Receive Frame Length (0 to 127) */
#define RX_TIME_RX_STAMP_LEN            (5)             /* read only 5 bytes (the adjusted timestamp (40:0)) */
#define TX_TIME_TX_STAMP_LEN            (5)             /* 40-bits = 5 bytes */
#define SCRATCH_BUFFER_MAX_LEN          (127)           /* AES scratch memory */
#define STD_FRAME_LEN                   (127)

/******************************************************************************
* @brief Bit definitions for register INDIRECT_ADDR_A
**/
#define INDIRECT_ADDR_A_ID                   0x1f0004
#define INDIRECT_ADDR_A_LEN                  (4U)
#define INDIRECT_ADDR_A_MASK                 0xFFFFFFFFUL

/******************************************************************************
* @brief Bit definitions for register ADDR_OFFSET_A
**/
#define ADDR_OFFSET_A_ID                     0x1f0008
#define ADDR_OFFSET_A_LEN                    (4U)
#define ADDR_OFFSET_A_MASK                   0xFFFFFFFFUL


#define REG_DIRECT_OFFSET_MAX_LEN   (127)
#define EXT_FRAME_LEN               (1023)
#define INDIRECT_POINTER_A_ID       0x1D0000            /* pointer to access indirect access buffer A */
#define INDIRECT_POINTER_B_ID       0x1E0000            /* pointer to access indirect access buffer B */


#define TX_BUFFER_ID            0x140000            /* Transmit Data Buffer */
#define SCRATCH_RAM_ID          0x160000
#define AES_KEY_RAM_MEM_ADDRESS 0x170000            /*Address of the AES keys in RAM*/


#define dwt_or16bitoffsetreg(ctx, addr, offset, or_val) dwt_modify16bitoffsetreg(ctx, addr, offset, -1, or_val)
#define dwt_and16bitoffsetreg(ctx, addr, offset, and_val) dwt_modify16bitoffsetreg(ctx, addr, offset, and_val, 0)
#define dwt_and_or16bitoffsetreg(ctx, addr,offset, and_val, or_val) dwt_modify16bitoffsetreg(ctx, addr,offset,and_val,or_val)

#define dwt_read32bitreg(ctx, addr)     dwt_read32bitoffsetreg(ctx, addr,0)

#define dwt_writefastCMD(ctx, cmd)     dwt_writetodevice(ctx, cmd,0,0,0)

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
        int regFileID,
        int regOffset);
    
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

    int dwt_configure(struct dwm3000_context *ctx, dwt_config_t *config);

    int dwt_writetxdata(struct dwm3000_context *ctx, uint16_t txDataLength, uint8_t *txDataBytes, uint16_t txBufferOffset);

    void dwt_writetxfctrl(struct dwm3000_context *ctx, uint16_t length, uint16_t offset, uint8_t ranging);

    int dwt_starttx(struct dwm3000_context *ctx, uint8_t mode);

    void original_dwt_write8bitoffsetreg(struct dwm3000_context *ctx, int regFileID, int regOffset, uint8_t regval);

int dwt_softreset(struct dwm3000_context *ctx);
int dwt_clearaonconfig(struct dwm3000_context *ctx);
int dwt_checkidlerc(struct dwm3000_context *ctx);
void dwt_configuretxrf(struct dwm3000_context *ctx, dwt_txconfig_t *txconfig);
void dwt_setrxantennadelay(uint16_t delay);
void dwt_settxantennadelay(uint16_t delay);
void dwt_setrxaftertxdelay(uint32_t delay);
void dwt_setrxtimeout(uint16_t timeout);
void dwt_setpreambledetecttimeout(uint16_t timeout);
void dwt_setlnapamode(uint8_t mode);
void dwt_setleds(uint8_t mode);



void dwt_readrxdata(uint8_t *buffer, uint16_t length, uint16_t offset);
uint64_t get_tx_timestamp_u64(void);
uint64_t get_rx_timestamp_u64(void);
void final_msg_set_ts(uint8_t *ts_field, uint64_t ts);
void dwt_setdelayedtrxtime(uint32_t time);

#endif /* DWM3000_H */