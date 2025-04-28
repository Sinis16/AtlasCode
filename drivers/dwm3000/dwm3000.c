/*
 * Copyright (c) 2025 Your Name
 * SPDX-License-License-Identifier: Apache-2.0
 */
#include <zephyr/logging/log.h>
 #include <assert.h>
 #include <zephyr/kernel.h>
 #include <zephyr/drivers/spi.h>
 #include <zephyr/drivers/gpio.h>
 #include "dwm3000.h"

 LOG_MODULE_REGISTER(dwt_read8bitoffsetreg, CONFIG_LOG_DEFAULT_LEVEL);
 

 static uint8_t crcTable[256];

 const uint16_t sts_length_factors[STS_LEN_SUPPORTED]=
{
    1024,1448,2048,2896,4096,5793,8192
};
 
 typedef struct
{
    uint32_t      partID ;            // IC Part ID - read during initialisation
    uint32_t      lotID ;             // IC Lot ID - read during initialisation
    uint8_t       bias_tune;          // bias tune code
    uint8_t       dgc_otp_set;        // Flag to check if DGC values are programmed in OTP
    uint8_t       vBatP;              // IC V bat read during production and stored in OTP (Vmeas @ 3V3)
    uint8_t       tempP;              // IC temp read during production and stored in OTP (Tmeas @ 23C)
    uint8_t       longFrames ;        // Flag in non-standard long frame mode
    uint8_t       otprev ;            // OTP revision number (read during initialisation)
    uint8_t       init_xtrim;         // initial XTAL trim value read from OTP (or defaulted to mid-range if OTP not programmed)
    uint8_t       dblbuffon;          // Double RX buffer mode and DB status flag
    uint16_t      sleep_mode;         // Used for automatic reloading of LDO tune and microcode at wake-up
    int16_t       ststhreshold;       // Threshold for deciding if received STS is good or bad
    dwt_spi_crc_mode_e   spicrc;      // Use SPI CRC when this flag is true
    uint8_t       stsconfig;          // STS configuration mode
    uint8_t       cia_diagnostic;     // CIA dignostic logging level
    dwt_cb_data_t cbData;             // Callback data structure
    dwt_spierrcb_t cbSPIRDErr;        // Callback for SPI read error events
    dwt_cb_t    cbTxDone;             // Callback for TX confirmation event
    dwt_cb_t    cbRxOk;               // Callback for RX good frame event
    dwt_cb_t    cbRxTo;               // Callback for RX timeout events
    dwt_cb_t    cbRxErr;              // Callback for RX error events
    dwt_cb_t    cbSPIErr;             // Callback for SPI error events
    dwt_cb_t    cbSPIRdy;             // Callback for SPI ready events
} dwt_local_data_t ;
 
static dwt_local_data_t   DW3000local[DWT_NUM_DW_DEV] ; // Local device data, can be an array to support multiple DW3000 testing applications/platforms
static dwt_local_data_t *pdw3000local = &DW3000local[0];   // Local data structure pointer
static uint8_t crcTable[256];

static uint32_t _dwt_otpread(struct dwm3000_context *ctx, uint16_t address); 

 int dwm3000_spi_transceive(struct dwm3000_context *ctx, uint8_t *tx_buf, uint8_t *rx_buf, size_t len)
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

 /* Write 8-bit value to register with offset */
static int dwt_write8bitoffsetreg(struct dwm3000_context *ctx, uint16_t reg, uint16_t offset, uint8_t value)
{
    uint8_t tx_buf[3] = { (uint8_t)(reg & 0x7F) | 0x80, (uint8_t)offset, value };
    uint8_t rx_buf[3] = {0};

    return dwm3000_spi_transceive(ctx, tx_buf, rx_buf, 3);
}

void original_dwt_write8bitoffsetreg(struct dwm3000_context *ctx, int regFileID, int regOffset, uint8_t regval)
{
    //uint8_t   buf[1];
    //buf[0] = regval;
    dwt_writetodevice(ctx, regFileID, regOffset, 1, &regval);
}

/* Write 16-bit value to register with offset */
static int dwt_write16bitoffsetreg(struct dwm3000_context *ctx, uint16_t reg, uint16_t offset, uint16_t value)
{
    uint8_t tx_buf[4] = { (uint8_t)(reg & 0x7F) | 0x80, (uint8_t)offset, (uint8_t)value, (uint8_t)(value >> 8) };
    uint8_t rx_buf[4] = {0};

    return dwm3000_spi_transceive(ctx, tx_buf, rx_buf, 4);
}


void original_dwt_write16bitoffsetreg(struct dwm3000_context *ctx, int regFileID, int regOffset, uint16_t regval)
{
    uint8_t   buffer[2] ;

    buffer[0] = (uint8_t)regval;
    buffer[1] = regval >> 8 ;

    dwt_writetodevice(ctx, regFileID,regOffset,2,buffer);
}

 /* OR 8-bit value with register at offset */
static int dwt_or8bitoffsetreg(struct dwm3000_context *ctx, uint16_t reg, uint16_t offset, uint8_t value)
{
    uint8_t tx_buf[2] = { (uint8_t)(reg & 0x7F), (uint8_t)offset };
    uint8_t rx_buf[2] = {0};
    int ret = dwm3000_spi_transceive(ctx, tx_buf, rx_buf, 2);
    if (ret) {
        return ret;
    }

    tx_buf[0] |= 0x80; // Write mode
    tx_buf[1] = rx_buf[1] | value;
    return dwm3000_spi_transceive(ctx, tx_buf, rx_buf, 2);
}


void dwt_init_crc_table(void)
{
    const uint8_t polynomial = 0x31; // CRC-8-Dallas/Maxim: x^8 + x^5 + x^4 + 1
    for (int i = 0; i < 256; i++) {
        uint8_t crc = i;
        for (int j = 0; j < 8; j++) {
            crc = (crc & 0x80) ? (crc << 1) ^ polynomial : crc << 1;
        }
        crcTable[i] = crc;
    }
}

int dwm3000_init(struct dwm3000_context *ctx, const struct dwm3000_config *cfg)
{
    if (!ctx || !cfg) {
        return -EINVAL;
    }

    ctx->config = cfg;
    ctx->dblbuffon = DBL_BUFF_ACCESS_BUFFER_0;
    ctx->sleep_mode = 0;
    ctx->buf_size = DWM3000_SPI_BUF_SIZE;

    if (!device_is_ready(cfg->spi_dev)) {
        return -ENODEV;
    }

    if (!device_is_ready(cfg->gpio_dev)) {
        return -ENODEV;
    }

    int ret;
    ret = gpio_pin_configure(cfg->gpio_dev, cfg->cs_pin, GPIO_OUTPUT_HIGH);
    if (ret) return ret;
    ret = gpio_pin_configure(cfg->gpio_dev, cfg->reset_pin, GPIO_OUTPUT_HIGH);
    if (ret) return ret;
    ret = gpio_pin_configure(cfg->gpio_dev, cfg->wakeup_pin, GPIO_OUTPUT_HIGH);
    if (ret) return ret;
    ret = gpio_pin_configure(cfg->gpio_dev, cfg->irq_pin, GPIO_INPUT);
    if (ret) return ret;

    dwt_init_crc_table(); // Initialize CRC table
    return 0;
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
 
 /**/
 int dwm3000_read_dev_id(struct dwm3000_context *ctx, uint32_t *dev_id)
 {
     if (!ctx || !dev_id) {
         return -EINVAL;
     }
 
     uint8_t tx_buf[5] = {DWM3000_REG_DEV_ID, 0x80, 0x00, 0x00, 0x00};
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

/* DS TWR function implementations */

/* Reference: ds_twr_initiator_sts.c - port_set_dw_ic_spi_fastrate() sets SPI to high speed (e.g., 8 MHz) */
int port_set_dw_ic_spi_fastrate(struct dwm3000_context *ctx)
{
    if (!ctx || !ctx->config || !device_is_ready(ctx->config->spi_dev)) {
        return -EINVAL;
    }

    struct dwm3000_config *cfg = (struct dwm3000_config *)ctx->config;
    cfg->spi_cfg.frequency = 8000000;
    cfg->spi_cfg.operation = SPI_WORD_SET(8) | SPI_OP_MODE_MASTER;

    return 0;
}

int port_set_dw_ic_spi_slowrate(struct dwm3000_context *ctx)
{
    if (!ctx || !ctx->config || !device_is_ready(ctx->config->spi_dev)) {
        return -EINVAL;
    }

    struct dwm3000_config *cfg = (struct dwm3000_config *)ctx->config;
    cfg->spi_cfg.frequency = 2000000;
    cfg->spi_cfg.operation = SPI_WORD_SET(8) | SPI_OP_MODE_MASTER;

    return 0;
}


/* Reference: ds_twr_initiator_sts.c - reset_DWIC() performs hardware or soft reset of DW IC */
int true_reset_DWIC(struct dwm3000_context *ctx)
{
    
    if (!ctx || !ctx->config || !device_is_ready(ctx->config->gpio_dev)) {
        return -EINVAL;
    }
    
    const struct dwm3000_config *cfg = ctx->config;
    int ret;

    ret = port_set_dw_ic_spi_slowrate(ctx);
    if (ret) {
        return ret;
    }

    ret = dwt_softreset(ctx);
    if (ret) {
        return ret;
    }
    
    ret = port_set_dw_ic_spi_fastrate(ctx);
    if (ret) {
        return ret;
    }
    return 0;
}



/* Reference: ds_twr_initiator_sts.c - reset_DWIC() performs hardware or soft reset of DW IC */
int reset_DWIC(struct dwm3000_context *ctx)
{
    if (!ctx || !ctx->config || !device_is_ready(ctx->config->gpio_dev)) {
        return -EINVAL;
    }

    const struct dwm3000_config *cfg = ctx->config;
    int ret;

    /* Hardware reset using RSTn pin, per DWM3000 datasheet */
    ret = gpio_pin_configure(cfg->gpio_dev, cfg->reset_pin, GPIO_OUTPUT | GPIO_OPEN_DRAIN);
    if (ret) {
        return ret;
    }

    ret = gpio_pin_set(cfg->gpio_dev, cfg->wakeup_pin, 0); // WAKEUP low
    if (ret) {
        return ret;
    }
    k_sleep(K_MSEC(1)); // 1 ms, per datasheet

    ret = gpio_pin_set(cfg->gpio_dev, cfg->wakeup_pin, 1); // WAKEUP high
    if (ret) {

        return ret;
    }
    k_sleep(K_MSEC(5)); // 5 ms to ensure wake-up

    ret = gpio_pin_set(cfg->gpio_dev, cfg->reset_pin, 0); // RSTn low
    if (ret) {
        return ret;
    }
    k_sleep(K_USEC(100)); // 100 us, per datasheet

    ret = gpio_pin_set(cfg->gpio_dev, cfg->reset_pin, 1); // RSTn high
    if (ret) {
        return ret;
    }
    k_sleep(K_MSEC(5)); // 5 ms for device to stabilize

    ret = gpio_pin_configure(cfg->gpio_dev, cfg->reset_pin, GPIO_OUTPUT_HIGH); // Back to default
    if (ret) {
        return ret;
    }

    return 0;
}

/* Reference: ds_twr_initiator_sts.c - dwt_softreset() performs soft reset */
int dwt_softreset(struct dwm3000_context *ctx)
{
    if (!ctx || !ctx->config || !device_is_ready(ctx->config->spi_dev)) {
        return -EINVAL;
    }

    int ret = dwt_clearaonconfig(ctx);
    if (ret) {
        return ret;
    }

    k_sleep(K_MSEC(1)); // 1 ms for AON config

    originaldwt_or8bitoffsetreg(ctx, CLK_CTRL_ID, 0, FORCE_SYSCLK_FOSC);


    original_dwt_write8bitoffsetreg(ctx, SOFT_RST_ID, 0, DWT_RESET_ALL);

    k_sleep(K_MSEC(1)); // 1 ms for PLL lock

    ctx->dblbuffon = DBL_BUFF_ACCESS_BUFFER_0;
    ctx->sleep_mode = 0;

    return 0;
}

/* Reference: ds_twr_initiator_sts.c - dwt_clearaonconfig() clears AON configuration */
int dwt_clearaonconfig(struct dwm3000_context *ctx)
{
    if (!ctx || !ctx->config || !device_is_ready(ctx->config->spi_dev)) {
        return -EINVAL;
    }

    int ret = dwt_write16bitoffsetreg(ctx, AON_DIG_CFG_ID, 0, 0x00);
    if (ret) {
        return ret;
    }

    original_dwt_write8bitoffsetreg(ctx, ANA_CFG_ID, 0, 0x00);


    original_dwt_write8bitoffsetreg(ctx, AON_CTRL_ID, 0, 0x00);

    original_dwt_write8bitoffsetreg(ctx, AON_CTRL_ID, 0, AON_CTRL_ARRAY_SAVE_BIT_MASK);

    return 0;
}




int dwt_initialise(struct dwm3000_context *ctx, int mode)
{
   //uint16_t otp_addr;
   //uint32_t devid;
    uint32_t ldo_tune_lo;
    uint32_t ldo_tune_hi;

    pdw3000local->dblbuffon = DBL_BUFF_OFF; // Double buffer mode off by default / clear the flag
    pdw3000local->sleep_mode = DWT_RUNSAR;  // Configure RUN_SAR on wake by default as it is needed when running PGF_CAL
    pdw3000local->spicrc = 0;
    pdw3000local->stsconfig = 0; //STS off
    pdw3000local->vBatP = 0;
    pdw3000local->tempP = 0;

    pdw3000local->cbTxDone = NULL;
    pdw3000local->cbRxOk = NULL;
    pdw3000local->cbRxTo = NULL;
    pdw3000local->cbRxErr = NULL;
    pdw3000local->cbSPIRdy = NULL;
    pdw3000local->cbSPIErr = NULL;

    //Read LDO_TUNE and BIAS_TUNE from OTP
    ldo_tune_lo = _dwt_otpread(ctx, LDOTUNELO_ADDRESS);
    ldo_tune_hi = _dwt_otpread(ctx, LDOTUNEHI_ADDRESS);
    pdw3000local->bias_tune = (_dwt_otpread(ctx, BIAS_TUNE_ADDRESS) >> 16) & BIAS_CTRL_BIAS_MASK;

    if ((ldo_tune_lo != 0) && (ldo_tune_hi != 0) && (pdw3000local->bias_tune != 0))
    {
        _dwt_prog_ldo_and_bias_tune(ctx);
    }

    // Read DGC_CFG from OTP
    if (_dwt_otpread(ctx, DGC_TUNE_ADDRESS) == DWT_DGC_CFG0)
    {
        pdw3000local->dgc_otp_set = DWT_DGC_LOAD_FROM_OTP;
    }
    else
    {
        pdw3000local->dgc_otp_set = DWT_DGC_LOAD_FROM_SW;
    }

    // Load Part and Lot ID from OTP
    if(mode & DWT_READ_OTP_PID)
        pdw3000local->partID = _dwt_otpread(ctx, PARTID_ADDRESS);
    if (mode & DWT_READ_OTP_LID)
        pdw3000local->lotID = _dwt_otpread(ctx, LOTID_ADDRESS);
    if (mode & DWT_READ_OTP_BAT)
        pdw3000local->vBatP = (uint8_t)_dwt_otpread(ctx, VBAT_ADDRESS);
    if (mode & DWT_READ_OTP_TMP)
        pdw3000local->tempP = (uint8_t)_dwt_otpread(ctx, VTEMP_ADDRESS);


    if(pdw3000local->tempP == 0) //if the reference temperature has not been programmed in OTP (early eng samples) set to default value
    {
        pdw3000local->tempP = 0x85 ; //@temp of 20 deg
    }

    if(pdw3000local->vBatP == 0) //if the reference voltage has not been programmed in OTP (early eng samples) set to default value
    {
        pdw3000local->vBatP = 0x74 ;  //@Vref of 3.0V
    }

    pdw3000local->otprev = (uint8_t) _dwt_otpread(ctx, OTPREV_ADDRESS);

    pdw3000local->init_xtrim = _dwt_otpread(ctx, XTRIM_ADDRESS) & 0x7f;
    if(pdw3000local->init_xtrim == 0)
    {
        pdw3000local->init_xtrim = 0x2E ; //set default value
    }
    original_dwt_write8bitoffsetreg(ctx, XTAL_ID, 0, pdw3000local->init_xtrim);


    return DWT_SUCCESS ;

}


void _dwt_prog_ldo_and_bias_tune(struct dwm3000_context *ctx)
{
    dwt_or16bitoffsetreg(ctx, OTP_CFG_ID, 0, LDO_BIAS_KICK);
    dwt_and_or16bitoffsetreg(ctx, BIAS_CTRL_ID, 0, (uint16_t)~BIAS_CTRL_BIAS_MASK, pdw3000local->bias_tune);
}

void dwt_modify16bitoffsetreg(struct dwm3000_context *ctx, const int regFileID, const int regOffset, const uint16_t _and, const uint16_t _or)
{
    uint8_t buf[4];
    buf[0] = (uint8_t)_and;//       &0xFF;
    buf[1] = (uint8_t)(_and>>8);//  &0xFF;
    buf[2] = (uint8_t)_or;//        &0xFF;
    buf[3] = (uint8_t)(_or>>8);//   &0xFF;
    dwt_xfer3000(ctx, regFileID, regOffset, sizeof(buf), buf, DW3000_SPI_AND_OR_16);
}

uint32_t _dwt_otpread(struct dwm3000_context *ctx, uint16_t address)
{
    uint32_t ret_data = 0;

    // Set manual access mode
    dwt_write16bitoffsetreg(ctx, OTP_CFG_ID, 0, 0x0001);
    // set the address
    dwt_write16bitoffsetreg(ctx, OTP_ADDR_ID, 0, address);
    // Assert the read strobe
    dwt_write16bitoffsetreg(ctx, OTP_CFG_ID, 0, 0x0002);
    // attempt a read from OTP address
    ret_data = dwt_read32bitoffsetreg(ctx, OTP_RDATA_ID, 0);

    // Return the 32bit of read data
    return ret_data;
}

uint32_t dwt_read32bitoffsetreg(struct dwm3000_context *ctx, int regFileID, int regOffset)
{
    int     j ;
    uint32_t  regval = 0 ;
    uint8_t   buffer[4] ;

    dwt_readfromdevice(ctx, regFileID,regOffset,4,buffer); // Read 4 bytes (32-bits) register into buffer

    for (j = 3 ; j >= 0 ; j --)
    {
        regval = (regval << 8) + buffer[j] ;
    }

    return (regval);

} 



uint16_t dwt_read16bitoffsetreg(struct dwm3000_context *ctx, int regFileID,int regOffset)
{
    uint16_t  regval = 0 ;
    uint8_t   buffer[2] ;

    dwt_readfromdevice(ctx, regFileID,regOffset,2,buffer); // Read 2 bytes (16-bits) register into buffer

    regval = ((uint16_t)buffer[1] << 8) + buffer[0] ;
    return regval ;

} 


int new_dwt_checkidlerc(struct dwm3000_context *ctx)
{

    uint32_t reg = ((uint32_t)dwt_read16bitoffsetreg(ctx, SYS_STATUS_ID, 2) << 16);

    return ( (reg & (SYS_STATUS_RCINIT_BIT_MASK)) == (SYS_STATUS_RCINIT_BIT_MASK));
}

void dwt_xfer3000(struct dwm3000_context *ctx, const uint32_t regFileID, const uint16_t indx,
                  const uint16_t length, uint8_t *buffer, const spi_modes_e mode)
{
    uint8_t header[2];
    uint16_t cnt = 0;
    uint16_t reg_file = 0x1F & ((regFileID + indx) >> 16);
    uint16_t reg_offset = 0x7F & (regFileID + indx);

    assert(reg_file <= 0x1F);
    assert(reg_offset <= 0x7F);
    assert(length < 0x3100);
    assert(mode == DW3000_SPI_WR_BIT || mode == DW3000_SPI_RD_BIT ||
           mode == DW3000_SPI_AND_OR_8 || mode == DW3000_SPI_AND_OR_16 || mode == DW3000_SPI_AND_OR_32);

    uint16_t addr = (reg_file << 9) | (reg_offset << 2);
    header[0] = (uint8_t)((mode | addr) >> 8);
    header[1] = (uint8_t)(addr | (mode & 0x03));

    if (length == 0) {
        assert(mode == DW3000_SPI_WR_BIT);
        header[0] = (uint8_t)((DW3000_SPI_WR_BIT >> 8) | (regFileID << 1) | DW3000_SPI_FAC);
        cnt = 1;
    } else if (reg_offset == 0 && (mode == DW3000_SPI_WR_BIT || mode == DW3000_SPI_RD_BIT)) {
        header[0] |= DW3000_SPI_FARW;
        cnt = 1;
    } else {
        header[0] |= DW3000_SPI_EAMRW;
        cnt = 2;
    }


    switch (mode) {
    case DW3000_SPI_AND_OR_8:
    case DW3000_SPI_AND_OR_16:
    case DW3000_SPI_AND_OR_32:
    case DW3000_SPI_WR_BIT:
    {
        uint8_t crc8 = 0;
        if (pdw3000local->spicrc != DWT_SPI_CRC_MODE_NO) {
            crc8 = dwt_generatecrc8(header, cnt, 0);
            crc8 = dwt_generatecrc8(buffer, length, crc8);
            //LOG_INF("Write CRC8: 0x%02x", crc8);
            writetospiwithcrc(ctx, cnt, header, length, buffer, crc8);
        } else {
            writetospi(ctx, cnt, header, length, buffer);
        }
        break;
    }
    case DW3000_SPI_RD_BIT:
    {
        int ret = readfromspi(ctx, cnt, header, length, buffer);
        if (ret) {
            //LOG_ERR("readfromspi failed: %d", ret);
        } else {
            //LOG_INF("Read data: 0x%02x", buffer[0]);
        }

        if ((pdw3000local->spicrc == DWT_SPI_CRC_MODE_WRRD) && (regFileID != SPICRC_CFG_ID)) {
            uint8_t crc8 = dwt_generatecrc8(header, cnt, 0);
            crc8 = dwt_generatecrc8(buffer, length, crc8);
            uint8_t dwcrc8 = dwt_read8bitoffsetreg(ctx, SPICRC_CFG_ID, 0);
            //LOG_INF("Read CRC8: calc=0x%02x, device=0x%02x", crc8, dwcrc8);
            if (crc8 != dwcrc8) {
                //LOG_ERR("CRC mismatch: calc=0x%02x, device=0x%02x", crc8, dwcrc8);
                if (pdw3000local->cbSPIRDErr != NULL)
                    pdw3000local->cbSPIRDErr();
            }
        }
        break;
    }
    default:
        //LOG_ERR("Invalid mode: %d", mode);
        while (1);
        break;
    }
}

uint8_t dwt_read8bitoffsetreg(struct dwm3000_context *ctx, int regFileID, int regOffset)
{
    uint8_t regval;

    dwt_readfromdevice(ctx, regFileID, regOffset, 1, &regval);

    return regval;
}


void dwt_readfromdevice
(
    struct dwm3000_context *ctx,
    uint32_t  regFileID,
    uint16_t  index,
    uint16_t  length,
    uint8_t   *buffer
)
{
    dwt_xfer3000(ctx, regFileID, index, length, buffer, DW3000_SPI_RD_BIT);
}





int readfromspi(
    struct dwm3000_context *ctx,
    uint16_t        headerLength,
    const uint8_t * headerBuffer,
    uint16_t        readLength,
    uint8_t       * readBuffer)
{
    uint16_t len = headerLength + readLength;
    uint8_t *tx_buf = ctx->tx_buf; // Assuming tx_buf is part of ctx
    uint8_t *rx_buf = ctx->rx_buf; // Assuming rx_buf is part of ctx

    if (len > ctx->buf_size) // Assuming buf_size is part of ctx
        return -1;

    memset(&tx_buf[0], 0, len);
    memcpy(&tx_buf[0], headerBuffer, headerLength);

    int ret = dwm3000_spi_transceive(ctx, tx_buf, rx_buf, len);
    if (ret)
        return ret;

    memcpy(readBuffer, rx_buf + headerLength, readLength);

#if 0
    LOG_HEXDUMP_INF(headerBuffer, headerLength, "readfromspi: Header");
    LOG_HEXDUMP_INF(readBuffer, readLength, "readfromspi: Body");
#endif

    return 0;
}

int writetospiwithcrc(
    struct dwm3000_context *ctx,
    uint16_t           headerLength,
    const    uint8_t * headerBuffer,
    uint16_t           bodyLength,
    const    uint8_t * bodyBuffer,
    uint8_t            crc8)
{
    uint16_t len = headerLength + bodyLength + sizeof(crc8);
    uint8_t *tx_buf = ctx->tx_buf; // Assuming tx_buf is part of ctx
    uint8_t *rx_buf = ctx->rx_buf; // Assuming rx_buf is part of ctx

    if (len > ctx->buf_size) // Assuming buf_size is part of ctx
        return -1;

    memcpy(&tx_buf[0], headerBuffer, headerLength);
    memcpy(&tx_buf[headerLength], bodyBuffer, bodyLength);
    tx_buf[headerLength + bodyLength] = crc8;

    return dwm3000_spi_transceive(ctx, tx_buf, rx_buf, len);
}

int writetospi(
    struct dwm3000_context *ctx,
    uint16_t           headerLength,
    const    uint8_t * headerBuffer,
    uint16_t           bodyLength,
    const    uint8_t * bodyBuffer)
{
#if 0
    LOG_HEXDUMP_INF(headerBuffer, headerLength, "writetospi: Header");
    LOG_HEXDUMP_INF(bodyBuffer, bodyLength, "writetospi: Body");
#endif

    uint16_t len = headerLength + bodyLength;
    uint8_t *tx_buf = ctx->tx_buf; // Assuming tx_buf is part of ctx
    uint8_t *rx_buf = ctx->rx_buf; // Assuming rx_buf is part of ctx

    if (len > ctx->buf_size) // Assuming buf_size is part of ctx
        return -1;

    memcpy(&tx_buf[0], headerBuffer, headerLength);
    memcpy(&tx_buf[headerLength], bodyBuffer, bodyLength);

    return dwm3000_spi_transceive(ctx, tx_buf, rx_buf, len);
}

uint8_t dwt_generatecrc8(const uint8_t* byteArray, int len, uint8_t crcRemainderInit)
{
    uint8_t data;
    int byte;

    for (byte = 0; byte < len; ++byte)
    {
        data = byteArray[byte] ^ crcRemainderInit;
        crcRemainderInit = crcTable[data];// ^ (crcRemainderInit << 8);
    }

    return(crcRemainderInit);
}


/* Reference: ds_twr_initiator_sts.c - dwt_checkidlerc() checks if DW IC is in IDLE_RC state */
int dwt_checkidlerc(struct dwm3000_context *ctx)
{
    if (!ctx || !ctx->config || !device_is_ready(ctx->config->spi_dev)) {
        return -EINVAL;
    }

    uint8_t tx_buf[5] = {SYS_STATUS_ID, 0x00, 0x00, 0x00, 0x00};
    uint8_t rx_buf[5] = {0};
    int ret;
    uint32_t status;
    int64_t start_time = k_uptime_get();
    int64_t timeout_ms = 2000; // Timeout after 2000 ms

    while (k_uptime_get() - start_time < timeout_ms) {
        ret = dwm3000_spi_transceive(ctx, tx_buf, rx_buf, sizeof(tx_buf));
        if (ret) {
            ctx->last_sys_status = 0xFFFFFFFF; // Indicate SPI error
            return DWT_ERROR;
        }

        status = (rx_buf[4] << 24) | (rx_buf[3] << 16) | (rx_buf[2] << 8) | rx_buf[1];
        ctx->last_sys_status = status; // Store for main.c
        if (status & SYS_STATUS_IDLE_BIT) {
            return DWT_SUCCESS; // IDLE_RC state reached
        }

        k_sleep(K_MSEC(1)); // Poll every 1 ms
    }

    return DWT_ERROR; // Timeout, not in IDLE_RC
}

void dwt_modify8bitoffsetreg(struct dwm3000_context *ctx, const int regFileID, const int regOffset, const uint8_t _and, const uint8_t _or)
{
    uint8_t buf[2];
    buf[0] = _and;
    buf[1] = _or;
    dwt_xfer3000(ctx, regFileID, regOffset, sizeof(buf),buf, DW3000_SPI_AND_OR_8);
}

void dwt_modify32bitoffsetreg(struct dwm3000_context *ctx, const int regFileID, const int regOffset, const uint32_t _and, const uint32_t _or)
{
    uint8_t buf[8];
    buf[0] = (uint8_t)_and;//       &0xFF;
    buf[1] = (uint8_t)(_and>>8);//  &0xFF;
    buf[2] = (uint8_t)(_and>>16);// &0xFF;
    buf[3] = (uint8_t)(_and>>24);// &0xFF;
    buf[4] = (uint8_t)_or;//        &0xFF;
    buf[5] = (uint8_t)(_or>>8);//   &0xFF;
    buf[6] = (uint8_t)(_or>>16);//  &0xFF;
    buf[7] = (uint8_t)(_or>>24);//  &0xFF;
    dwt_xfer3000(ctx, regFileID, regOffset, sizeof(buf), buf, DW3000_SPI_AND_OR_32);
}

void dwt_writetodevice
(
    struct dwm3000_context *ctx,
    uint32_t      regFileID,
    uint16_t      index,
    uint16_t      length,
    uint8_t       *buffer
)
{
    dwt_xfer3000(ctx, regFileID, index, length, buffer, DW3000_SPI_WR_BIT);
}

void dwt_write32bitoffsetreg(struct dwm3000_context *ctx, int regFileID, int regOffset, uint32_t regval)
{
    int     j ;
    uint8_t   buffer[4] ;

    for ( j = 0 ; j < 4 ; j++ )
    {
        buffer[j] = (uint8_t)regval;
        regval >>= 8 ;
    }

    dwt_writetodevice(ctx, regFileID,regOffset,4,buffer);
} 

static
void dwt_force_clocks(struct dwm3000_context *ctx, int clocks)
{

    if (clocks == FORCE_CLK_SYS_TX)
    {
        uint16_t regvalue0 = CLK_CTRL_TX_BUF_CLK_ON_BIT_MASK | CLK_CTRL_RX_BUF_CLK_ON_BIT_MASK;

        //SYS_CLK_SEL = PLL
        regvalue0 |= ((uint16_t) FORCE_SYSCLK_PLL) << CLK_CTRL_SYS_CLK_SEL_BIT_OFFSET;

        //TX_CLK_SEL = ON
        regvalue0 |= ((uint16_t) FORCE_CLK_PLL) << CLK_CTRL_TX_CLK_SEL_BIT_OFFSET;

        dwt_write16bitoffsetreg(ctx, CLK_CTRL_ID2, 0x0, regvalue0);

    }

    if (clocks == FORCE_CLK_AUTO)
    {
        //Restore auto clock mode
        dwt_write16bitoffsetreg(ctx, CLK_CTRL_ID2, 0x0, (uint16_t) DWT_AUTO_CLKS);  //we only need to restore the low 16 bits as they are the only ones to change as a result of  FORCE_CLK_SYS_TX
    }

} 

void dwt_setplenfine(struct dwm3000_context *ctx, uint8_t preambleLength)
{
    original_dwt_write8bitoffsetreg(ctx, TX_FCTRL_HI_ID, 1, preambleLength);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This function can place DW3000 into IDLE/IDLE_PLL or IDLE_RC mode when it is not actively in TX or RX.
 *
 * input parameters
 * @param state - DWT_DW_IDLE (1) to put DW3000 into IDLE/IDLE_PLL state; DWT_DW_INIT (0) to put DW3000 into INIT_RC state;
 *                DWT_DW_IDLE_RC (2) to put DW3000 into IDLE_RC state.
 *
 * output parameters none
 *
 * no return value
 */
void dwt_setdwstate(struct dwm3000_context *ctx,int state)
{
    if (state == DWT_DW_IDLE) // Set the auto INIT2IDLE bit so that DW3000 enters IDLE mode before switching clocks to system_PLL
    //NOTE: PLL should be configured prior to this, and the device should be in IDLE_RC (if the PLL does not lock device will remain in IDLE_RC)
    {
        //switch clock to auto - if coming here from INIT_RC the clock will be FOSC/4, need to switch to auto prior to setting auto INIT2IDLE bit
        dwt_force_clocks(ctx, FORCE_CLK_AUTO);
        originaldwt_or8bitoffsetreg(ctx, SEQ_CTRL_ID, 0x01, SEQ_CTRL_AINIT2IDLE_BIT_MASK>>8);
    }
    else if(state == DWT_DW_IDLE_RC)  //Change state to IDLE_RC and clear auto INIT2IDLE bit
    {
        //switch clock to FOSC
        originaldwt_or8bitoffsetreg(ctx, CLK_CTRL_ID2, 0, FORCE_SYSCLK_FOSC2);
        //clear the auto INIT2IDLE bit and set FORCE2INIT
        dwt_modify32bitoffsetreg(ctx, SEQ_CTRL_ID, 0x0, (uint32_t) ~SEQ_CTRL_AINIT2IDLE_BIT_MASK, SEQ_CTRL_FORCE2INIT_BIT_MASK);
        //clear force bits (device will stay in IDLE_RC)
        dwt_and8bitoffsetreg(ctx, SEQ_CTRL_ID, 0x2, (uint8_t) ~(SEQ_CTRL_FORCE2INIT_BIT_MASK>>16));
        //switch clock to auto
        dwt_force_clocks(ctx, FORCE_CLK_AUTO);
    }
    else
    //NOTE: the SPI rate needs to be <= 7MHz as device is switching to INIT_RC state
    {
        originaldwt_or8bitoffsetreg(ctx, CLK_CTRL_ID, 0, FORCE_SYSCLK_FOSCDIV4);
        //clear the auto INIT2IDLE bit and set FORCE2INIT
        dwt_modify32bitoffsetreg(ctx, SEQ_CTRL_ID, 0x0, (uint32_t) ~SEQ_CTRL_AINIT2IDLE_BIT_MASK, SEQ_CTRL_FORCE2INIT_BIT_MASK);
        dwt_and8bitoffsetreg(ctx, SEQ_CTRL_ID, 0x2, (uint8_t) ~(SEQ_CTRL_FORCE2INIT_BIT_MASK>>16));
    }
}



static
uint16_t get_sts_mnth (uint16_t cipher, uint8_t threshold, uint8_t shift_val)
{
    uint32_t  value;
    uint16_t  mod_val;

    value = cipher* (uint32_t)threshold;
    if (shift_val == 3)
    {
        value *= SQRT_FACTOR;//Factor to sqrt(2)
        value >>= SQRT_SHIFT_VAL;
    }

    mod_val = value % MOD_VALUE+ HALF_MOD;
    value >>= SHIFT_VALUE;
    /* Check if modulo greater than MOD_VALUE, if yes add 1 */
    if (mod_val >= MOD_VALUE)
        value += 1;

    return (uint16_t)value;
}

static
void _dwt_kick_dgc_on_wakeup(struct dwm3000_context *ctx, int8_t channel)
{
    /* The DGC_SEL bit must be set to '0' for channel 5 and '1' for channel 9 */
    if (channel == 5)
    {
        dwt_modify32bitoffsetreg(ctx, OTP_CFG_ID, 0, ~(OTP_CFG_DGC_SEL_BIT_MASK),
                (DWT_DGC_SEL_CH5 << OTP_CFG_DGC_SEL_BIT_OFFSET) | OTP_CFG_DGC_KICK_BIT_MASK);
    }
    else if (channel == 9)
    {
        dwt_modify32bitoffsetreg(ctx, OTP_CFG_ID, 0, ~(OTP_CFG_DGC_SEL_BIT_MASK),
                (DWT_DGC_SEL_CH9 << OTP_CFG_DGC_SEL_BIT_OFFSET) | OTP_CFG_DGC_KICK_BIT_MASK);
    }
}

int dwt_run_pgfcal(struct dwm3000_context *ctx)
{
    int result = DWT_SUCCESS;
    uint32_t    data;
    uint32_t    val = 0;
    uint8_t     cnt,flag;

    //put into cal mode
    //Turn on delay mode
    data = (((uint32_t)0x02) << RX_CAL_CFG_COMP_DLY_BIT_OFFSET) | (RX_CAL_CFG_CAL_MODE_BIT_MASK & 0x1);
    dwt_write32bitoffsetreg(ctx, RX_CAL_CFG_ID, 0x0, data);
    // Trigger PGF Cal
    originaldwt_or8bitoffsetreg(ctx, RX_CAL_CFG_ID, 0x0, RX_CAL_CFG_CAL_EN_BIT_MASK);

    for (flag=1,cnt=0;cnt<MAX_RETRIES_FOR_PGF;cnt++)
    {
        k_usleep(40);
        if(dwt_read8bitoffsetreg(ctx, RX_CAL_STS_ID, 0x0) == 1)
        {//PGF cal is complete
            flag=0;
            break;
        }
    }
    if (flag)
    {
        result = DWT_ERROR;
        //LOG_INF("PGF failed");
    }

    // Put into normal mode
    original_dwt_write8bitoffsetreg(ctx, RX_CAL_CFG_ID, 0x0, 0);
    original_dwt_write8bitoffsetreg(ctx, RX_CAL_STS_ID, 0x0, 1); //clear the status
    originaldwt_or8bitoffsetreg(ctx, RX_CAL_CFG_ID, 0x2, 0x1); //enable reading
    val = dwt_read32bitoffsetreg(ctx, RX_CAL_RESI_ID, 0x0);
    if (val == ERR_RX_CAL_FAIL)
    {
        //PGF I Cal Fail
        result = DWT_ERROR;
        //LOG_INF("PGF I failed");
    } 
    val = dwt_read32bitoffsetreg(ctx, RX_CAL_RESQ_ID, 0x0);
    if (val == ERR_RX_CAL_FAIL)
    {
        //PGF Q Cal Fail
        result = DWT_ERROR;
        //LOG_INF("PGF Q failed");
    }

    return result;
}

int dwt_pgf_cal(struct dwm3000_context *ctx, int ldoen)
{
    int temp;
    uint16_t val;

    //PGF needs LDOs turned on - ensure PGF LDOs are enabled
    if (ldoen == 1)
    {
        val = dwt_read16bitoffsetreg(ctx, LDO_CTRL_ID, 0);

        dwt_or16bitoffsetreg(ctx, LDO_CTRL_ID, 0, (
            LDO_CTRL_LDO_VDDIF2_EN_BIT_MASK |
            LDO_CTRL_LDO_VDDMS3_EN_BIT_MASK |
            LDO_CTRL_LDO_VDDMS1_EN_BIT_MASK));
    }

    //Run PGF Cal
    temp = dwt_run_pgfcal(ctx);

    //Turn off RX LDOs if previously off
    if (ldoen == 1)
    {
        dwt_and16bitoffsetreg(ctx, LDO_CTRL_ID, 0, val); // restore LDO values
    }
    return temp;
}


int dwt_configure(struct dwm3000_context *ctx, dwt_config_t *config)
{
    uint8_t chan = config->chan,cnt,flag;
    uint32_t temp;
    uint8_t scp = ((config->rxCode > 24) || (config->txCode > 24)) ? 1 : 0;
    uint8_t mode = (config->phrMode == DWT_PHRMODE_EXT) ? SYS_CFG_PHR_MODE_BIT_MASK : 0;
    uint16_t sts_len;
    int error = DWT_SUCCESS;

    

#ifdef DWT_API_ERROR_CHECK
    assert((config->dataRate == DWT_BR_6M8) || (config->dataRate == DWT_BR_850K));
    assert(config->rxPAC <= DWT_PAC4);
    assert((chan == 5) || (chan == 9));
    assert((config->txPreambLength == DWT_PLEN_32)
           || (config->txPreambLength == DWT_PLEN_64)
           || (config->txPreambLength == DWT_PLEN_72)
           || (config->txPreambLength == DWT_PLEN_128)
           || (config->txPreambLength == DWT_PLEN_256)
           || (config->txPreambLength == DWT_PLEN_512)
           || (config->txPreambLength == DWT_PLEN_1024)
           || (config->txPreambLength == DWT_PLEN_1536)
           || (config->txPreambLength == DWT_PLEN_2048)
           || (config->txPreambLength == DWT_PLEN_4096));
    assert((config->phrMode == DWT_PHRMODE_STD)
           || (config->phrMode == DWT_PHRMODE_EXT));
    assert((config->phrRate == DWT_PHRRATE_STD)
           || (config->phrRate == DWT_PHRRATE_DTA));
    assert((config->pdoaMode == DWT_PDOA_M0)
           || (config->pdoaMode == DWT_PDOA_M1)
           || (config->pdoaMode == DWT_PDOA_M3));
    assert(((config->stsMode & DWT_STS_CONFIG_MASK) == DWT_STS_MODE_OFF)
           || ((config->stsMode & DWT_STS_CONFIG_MASK) == DWT_STS_MODE_1)
           || ((config->stsMode & DWT_STS_CONFIG_MASK) == DWT_STS_MODE_2)
           || ((config->stsMode & DWT_STS_CONFIG_MASK) == DWT_STS_MODE_ND)
           || ((config->stsMode & DWT_STS_CONFIG_MASK) == DWT_STS_MODE_SDC)
           || ((config->stsMode & DWT_STS_CONFIG_MASK) == (DWT_STS_MODE_1 | DWT_STS_MODE_SDC))
           || ((config->stsMode & DWT_STS_CONFIG_MASK) == (DWT_STS_MODE_2 | DWT_STS_MODE_SDC))
           || ((config->stsMode & DWT_STS_CONFIG_MASK) == (DWT_STS_MODE_ND | DWT_STS_MODE_SDC))
           || ((config->stsMode & DWT_STS_CONFIG_MASK) == DWT_STS_CONFIG_MASK));
#endif
    int preamble_len;
    switch (config->txPreambLength)
    {
    case DWT_PLEN_32:
        preamble_len = 32;
        break;
    case DWT_PLEN_64:
        preamble_len = 64;
        break;
    case DWT_PLEN_72:
        preamble_len = 72;
        break;
    case DWT_PLEN_128:
        preamble_len = 128;
        break;
    default:
        preamble_len = 256;
        break;
    }


    pdw3000local->sleep_mode &= (~(DWT_ALT_OPS | DWT_SEL_OPS3));  //clear the sleep mode ALT_OPS bit
    pdw3000local->longFrames = config->phrMode ;
    sts_len=GET_STS_REG_SET_VALUE((uint16_t)(config->stsLength));
    pdw3000local->ststhreshold = (int16_t)((((uint32_t)sts_len) * 8) * STSQUAL_THRESH_64);
    pdw3000local->stsconfig = config->stsMode;


    /////////////////////////////////////////////////////////////////////////
    //SYS_CFG
    //clear the PHR Mode, PHR Rate, STS Protocol, SDC, PDOA Mode,
    //then set the relevant bits according to configuration of the PHR Mode, PHR Rate, STS Protocol, SDC, PDOA Mode,
    dwt_modify32bitoffsetreg(ctx, SYS_CFG_ID, 0, ~(SYS_CFG_PHR_MODE_BIT_MASK  |
                                              SYS_CFG_PHR_6M8_BIT_MASK   |
                                              SYS_CFG_CP_SPC_BIT_MASK    |
                                              SYS_CFG_PDOA_MODE_BIT_MASK |
                                              SYS_CFG_CP_SDC_BIT_MASK),
        ((uint32_t)config->pdoaMode) << SYS_CFG_PDOA_MODE_BIT_OFFSET
        | ((uint16_t)config->stsMode & DWT_STS_CONFIG_MASK) << SYS_CFG_CP_SPC_BIT_OFFSET
        | (SYS_CFG_PHR_6M8_BIT_MASK & ((uint32_t)config->phrRate << SYS_CFG_PHR_6M8_BIT_OFFSET))
        | mode);


    if (scp)
    {
        //configure OPS tables for SCP mode
        pdw3000local->sleep_mode |= DWT_ALT_OPS | DWT_SEL_OPS1;  //configure correct OPS table is kicked on wakeup
        dwt_modify32bitoffsetreg(ctx, OTP_CFG_ID, 0, ~(OTP_CFG_OPS_ID_BIT_MASK), DWT_OPSET_SCP | OTP_CFG_OPS_KICK_BIT_MASK);

        dwt_write32bitoffsetreg(ctx, IP_CONFIG_LO_ID, 0, IP_CONFIG_LO_SCP);       //Set this if Ipatov analysis is used in SCP mode
        dwt_write32bitoffsetreg(ctx, IP_CONFIG_HI_ID, 0, IP_CONFIG_HI_SCP);

        dwt_write32bitoffsetreg(ctx, STS_CONFIG_LO_ID, 0, STS_CONFIG_LO_SCP);
        original_dwt_write8bitoffsetreg(ctx, STS_CONFIG_HI_ID, 0, STS_CONFIG_HI_SCP);

    }
    else 
    {
        uint16_t sts_mnth;
        if (config->stsMode != DWT_STS_MODE_OFF)
        {

            //configure CIA STS lower bound
            if ((config->pdoaMode == DWT_PDOA_M1) || (config->pdoaMode == DWT_PDOA_M0))
            {
                //In PDOA mode 1, number of accumulated symbols is the whole length of the STS
                sts_mnth=get_sts_mnth(sts_length_factors[(uint8_t)(config->stsLength)], CIA_MANUALLOWERBOUND_TH_64, 3);
            }
            else
            {
                //In PDOA mode 3 number of accumulated symbols is half of the length of STS symbols
                sts_mnth=get_sts_mnth(sts_length_factors[(uint8_t)(config->stsLength)], CIA_MANUALLOWERBOUND_TH_64, 4);
            }

            preamble_len += (sts_len) * 8;

            dwt_modify16bitoffsetreg(ctx, STS_CONFIG_LO_ID, 2, (uint16_t)~(STS_CONFIG_LO_STS_MAN_TH_BIT_MASK >> 16), sts_mnth & 0x7F);

        }

        //configure OPS tables for non-SCP mode
        if (preamble_len >= 256)
        {
            pdw3000local->sleep_mode |= DWT_ALT_OPS | DWT_SEL_OPS0;
            dwt_modify32bitoffsetreg(ctx, OTP_CFG_ID, 0, ~(OTP_CFG_OPS_ID_BIT_MASK), DWT_OPSET_LONG | OTP_CFG_OPS_KICK_BIT_MASK);
        }
        else
        {
            dwt_modify32bitoffsetreg(ctx, OTP_CFG_ID, 0, ~(OTP_CFG_OPS_ID_BIT_MASK), DWT_OPSET_SHORT | OTP_CFG_OPS_KICK_BIT_MASK);
        }
    }

    dwt_modify8bitoffsetreg(ctx, DTUNE0_ID, 0, (uint8_t) ~DTUNE0_PRE_PAC_SYM_BIT_MASK, config->rxPAC);

    original_dwt_write8bitoffsetreg(ctx, STS_CFG0_ID, 0, sts_len-1);    /*Starts from 0 that is why -1*/


    if (config->txPreambLength == DWT_PLEN_72)
    {
        dwt_setplenfine(ctx, 8); //value 8 sets fine preamble length to 72 symbols - this is needed to set 72 length.
    }
    else
    {
        dwt_setplenfine(ctx, 0); //clear the setting in the FINE_PLEN register.
    }

    if((config->stsMode & DWT_STS_MODE_ND) == DWT_STS_MODE_ND)
    {
        //configure lower preamble detection threshold for no data STS mode
        dwt_write32bitoffsetreg(ctx, DTUNE3_ID, 0, PD_THRESH_NO_DATA);
    }
    else
    {
        //configure default preamble detection threshold for other modes
        dwt_write32bitoffsetreg(ctx, DTUNE3_ID, 0, PD_THRESH_DEFAULT);
    }

    /////////////////////////////////////////////////////////////////////////
    //CHAN_CTRL
    temp = dwt_read32bitoffsetreg(ctx, CHAN_CTRL_ID, 0);
    temp &= (~(CHAN_CTRL_RX_PCODE_BIT_MASK | CHAN_CTRL_TX_PCODE_BIT_MASK | CHAN_CTRL_SFD_TYPE_BIT_MASK | CHAN_CTRL_RF_CHAN_BIT_MASK));

    if (chan == 9) temp |= CHAN_CTRL_RF_CHAN_BIT_MASK;

    temp |= (CHAN_CTRL_RX_PCODE_BIT_MASK & ((uint32_t)config->rxCode << CHAN_CTRL_RX_PCODE_BIT_OFFSET));
    temp |= (CHAN_CTRL_TX_PCODE_BIT_MASK & ((uint32_t)config->txCode << CHAN_CTRL_TX_PCODE_BIT_OFFSET));
    temp |= (CHAN_CTRL_SFD_TYPE_BIT_MASK & ((uint32_t)config->sfdType << CHAN_CTRL_SFD_TYPE_BIT_OFFSET));

    dwt_write32bitoffsetreg(ctx, CHAN_CTRL_ID, 0, temp);


    /////////////////////////////////////////////////////////////////////////
    //TX_FCTRL
    // Set up TX Preamble Size, PRF and Data Rate
    dwt_modify32bitoffsetreg(ctx, TX_FCTRL_ID, 0, ~(TX_FCTRL_TXBR_BIT_MASK | TX_FCTRL_TXPSR_BIT_MASK),
                                              ((uint32_t)config->dataRate << TX_FCTRL_TXBR_BIT_OFFSET)
                                              | ((uint32_t) config->txPreambLength) << TX_FCTRL_TXPSR_BIT_OFFSET);


    //DTUNE (SFD timeout)
    // Don't allow 0 - SFD timeout will always be enabled
    if (config->sfdTO == 0)
    {
        config->sfdTO = DWT_SFDTOC_DEF;
    }
    original_dwt_write16bitoffsetreg(ctx, DTUNE0_ID, 2, config->sfdTO);

    ///////////////////////
    // RF
    if (chan == 9)
    {
        // Setup TX analog for ch9
        dwt_write32bitoffsetreg(ctx, TX_CTRL_HI_ID, 0, RF_TXCTRL_CH9);
        original_dwt_write16bitoffsetreg(ctx, PLL_CFG_ID, 0, RF_PLL_CFG_CH9);
        // Setup RX analog for ch9
        dwt_write32bitoffsetreg(ctx, RX_CTRL_HI_ID, 0, RF_RXCTRL_CH9);
    }
    else
    {
        // Setup TX analog for ch5
        dwt_write32bitoffsetreg(ctx, TX_CTRL_HI_ID, 0, RF_TXCTRL_CH5);
        original_dwt_write16bitoffsetreg(ctx, PLL_CFG_ID, 0, RF_PLL_CFG_CH5);
    }

    original_dwt_write8bitoffsetreg(ctx, LDO_RLOAD_ID, 1, LDO_RLOAD_VAL_B1);
    original_dwt_write8bitoffsetreg(ctx, TX_CTRL_LO_ID, 2, RF_TXCTRL_LO_B2);
    original_dwt_write8bitoffsetreg(ctx, PLL_CAL_ID, 0, RF_PLL_CFG_LD);        // Extend the lock delay


    //Verify PLL lock bit is cleared
    original_dwt_write8bitoffsetreg(ctx, SYS_STATUS_ID, 0, SYS_STATUS_CP_LOCK_BIT_MASK);


    ///////////////////////
    // auto cal the PLL and change to IDLE_PLL state
    dwt_setdwstate(ctx, DWT_DW_IDLE);


    for (flag=1, cnt=0; cnt < MAX_RETRIES_FOR_PLL; cnt++)
    {
        k_usleep(20);
        if ((dwt_read8bitoffsetreg(ctx, SYS_STATUS_ID, 0) & SYS_STATUS_CP_LOCK_BIT_MASK))
        {
            /* PLL is locked */
            flag = 0;
            break;
        }
    }
    if (flag)
    {
        return  DWT_ERROR;
    }

    if ((config->rxCode >= 9) && (config->rxCode <= 24)) //only enable DGC for PRF 64
    {
        //load RX LUTs
        /* If the OTP has DGC info programmed into it, do a manual kick from OTP. */
        if (pdw3000local->dgc_otp_set == DWT_DGC_LOAD_FROM_OTP)
        {
            _dwt_kick_dgc_on_wakeup(ctx, chan);
        }
        /* Else we manually program hard-coded values into the DGC registers. */
        else
        {
            dwt_configmrxlut(ctx, chan);
        }
        dwt_modify16bitoffsetreg(ctx, DGC_CFG_ID, 0x0, (uint16_t)~DGC_CFG_THR_64_BIT_MASK, DWT_DGC_CFG << DGC_CFG_THR_64_BIT_OFFSET);
    }
    else
    {
        dwt_and8bitoffsetreg(ctx, DGC_CFG_ID, 0x0, (uint8_t)~DGC_CFG_RX_TUNE_EN_BIT_MASK);
    }

    ///////////////////////
    // PGF
    error = dwt_pgf_cal(ctx, 1);  //if the RX calibration routine fails the device receiver performance will be severely affected, the application should reset and try again


    return error;
} // end dwt_configure()


void dwt_configmrxlut(struct dwm3000_context *ctx, int channel)
{
	uint32_t lut0, lut1, lut2, lut3, lut4, lut5, lut6 = 0;

    if (channel == 5)
    {
        lut0 = (uint32_t)CH5_DGC_LUT_0;
        lut1 = (uint32_t)CH5_DGC_LUT_1;
        lut2 = (uint32_t)CH5_DGC_LUT_2;
        lut3 = (uint32_t)CH5_DGC_LUT_3;
        lut4 = (uint32_t)CH5_DGC_LUT_4;
        lut5 = (uint32_t)CH5_DGC_LUT_5;
        lut6 = (uint32_t)CH5_DGC_LUT_6;
    }
    else
    {
        lut0 = (uint32_t)CH9_DGC_LUT_0;
        lut1 = (uint32_t)CH9_DGC_LUT_1;
        lut2 = (uint32_t)CH9_DGC_LUT_2;
        lut3 = (uint32_t)CH9_DGC_LUT_3;
        lut4 = (uint32_t)CH9_DGC_LUT_4;
        lut5 = (uint32_t)CH9_DGC_LUT_5;
        lut6 = (uint32_t)CH9_DGC_LUT_6;
    }
    dwt_write32bitoffsetreg(ctx, DGC_LUT_0_CFG_ID, 0x0, lut0);
    dwt_write32bitoffsetreg(ctx, DGC_LUT_1_CFG_ID, 0x0, lut1);
    dwt_write32bitoffsetreg(ctx, DGC_LUT_2_CFG_ID, 0x0, lut2);
    dwt_write32bitoffsetreg(ctx, DGC_LUT_3_CFG_ID, 0x0, lut3);
    dwt_write32bitoffsetreg(ctx, DGC_LUT_4_CFG_ID, 0x0, lut4);
    dwt_write32bitoffsetreg(ctx, DGC_LUT_5_CFG_ID, 0x0, lut5);
    dwt_write32bitoffsetreg(ctx, DGC_LUT_6_CFG_ID, 0x0, lut6);
    dwt_write32bitoffsetreg(ctx, DGC_CFG0_ID, 0x0, DWT_DGC_CFG0);
    dwt_write32bitoffsetreg(ctx, DGC_CFG1_ID, 0x0, DWT_DGC_CFG1);
}





uint8_t dwt_calcbandwidthadj(struct dwm3000_context *ctx, uint16_t target_count, int channel)
{
    // Force system clock to FOSC/4 and TX clocks on and enable RF blocks
    dwt_force_clocks(ctx, FORCE_CLK_SYS_TX);
    dwt_enable_rf_tx(ctx, channel, 0);
    dwt_enable_rftx_blocks(ctx, channel);

    // Write to the PG target before kicking off PG auto-cal with given target value
    original_dwt_write16bitoffsetreg(ctx, PG_CAL_TARGET_ID, 0x0, target_count & PG_CAL_TARGET_TARGET_BIT_MASK);
    // Run PG count cal
    originaldwt_or8bitoffsetreg(ctx, PGC_CTRL_ID, 0x0, (uint8_t)(PGC_CTRL_PGC_START_BIT_MASK | PGC_CTRL_PGC_AUTO_CAL_BIT_MASK));
    // Wait for calibration to complete
    while (dwt_read8bitoffsetreg(ctx, PGC_CTRL_ID, 0) & PGC_CTRL_PGC_START_BIT_MASK);

    //Restore clocks to AUTO and turn off TX blocks
    dwt_disable_rftx_blocks(ctx);
    dwt_disable_rf_tx(ctx, 0);
    dwt_force_clocks(ctx, FORCE_CLK_AUTO);

    return  (dwt_read8bitoffsetreg(ctx, TX_CTRL_HI_ID, 0) & TX_CTRL_HI_TX_PG_DELAY_BIT_MASK);
}


void dwt_disable_rftx_blocks(struct dwm3000_context *ctx)
{
    dwt_write32bitoffsetreg(ctx, RF_CTRL_MASK_ID, 0, 0x00000000);
}


void dwt_disable_rf_tx(struct dwm3000_context *ctx, uint8_t switch_config)
{
    //Turn off TX LDOs
    dwt_write32bitoffsetreg(ctx, LDO_CTRL_ID, 0, 0x00000000);

    //Disable RF blocks for TX (configure RF_ENABLE_ID reg)
    dwt_write32bitoffsetreg(ctx, RF_ENABLE_ID, 0, 0x00000000);

    if (switch_config)
    {
        //Restore the TXRX switch to auto
        dwt_write32bitoffsetreg(ctx, RF_SWITCH_CTRL_ID, 0x0, TXRXSWITCH_AUTO);
    }
}


void dwt_enable_rftx_blocks(struct dwm3000_context *ctx, uint32_t channel)
{
    if (channel == SEL_CHANNEL5)
    {
        dwt_or32bitoffsetreg(ctx, RF_CTRL_MASK_ID, 0, (RF_ENABLE_TX_SW_EN_BIT_MASK
                | RF_ENABLE_TX_CH5_BIT_MASK | RF_ENABLE_TX_EN_BIT_MASK
                | RF_ENABLE_TX_EN_BUF_BIT_MASK | RF_ENABLE_TX_BIAS_EN_BIT_MASK));
    }
    else if (channel == SEL_CHANNEL9)
    {
        dwt_or32bitoffsetreg(ctx, RF_CTRL_MASK_ID, 0, (RF_ENABLE_TX_SW_EN_BIT_MASK
                | RF_ENABLE_TX_EN_BIT_MASK
                | RF_ENABLE_TX_EN_BUF_BIT_MASK | RF_ENABLE_TX_BIAS_EN_BIT_MASK));
    }
}


void dwt_enable_rf_tx(struct dwm3000_context *ctx, uint32_t channel, uint8_t switch_control)
{
    //Turn on TX LDOs
    dwt_or32bitoffsetreg(ctx, LDO_CTRL_ID, 0, (LDO_CTRL_LDO_VDDHVTX_VREF_BIT_MASK |
            LDO_CTRL_LDO_VDDHVTX_EN_BIT_MASK));
    dwt_or32bitoffsetreg(ctx, LDO_CTRL_ID, 0, (LDO_CTRL_LDO_VDDTX2_VREF_BIT_MASK |
            LDO_CTRL_LDO_VDDTX1_VREF_BIT_MASK |
            LDO_CTRL_LDO_VDDTX2_EN_BIT_MASK |
            LDO_CTRL_LDO_VDDTX1_EN_BIT_MASK));

    //Enable RF blocks for TX (configure RF_ENABLE_ID reg)
    if (channel == SEL_CHANNEL5)
    {
        dwt_or32bitoffsetreg(ctx, RF_ENABLE_ID, 0, (RF_ENABLE_TX_SW_EN_BIT_MASK
            | RF_ENABLE_TX_CH5_BIT_MASK | RF_ENABLE_TX_EN_BIT_MASK
            | RF_ENABLE_TX_EN_BUF_BIT_MASK | RF_ENABLE_TX_BIAS_EN_BIT_MASK));
    }
    else
    {
        dwt_or32bitoffsetreg(ctx, RF_ENABLE_ID, 0, (RF_ENABLE_TX_SW_EN_BIT_MASK
            | RF_ENABLE_TX_EN_BIT_MASK
            | RF_ENABLE_TX_EN_BUF_BIT_MASK | RF_ENABLE_TX_BIAS_EN_BIT_MASK));
    }

    if (switch_control)
    {
        //configure the TXRX switch for TX mode
        dwt_write32bitoffsetreg(ctx, RF_SWITCH_CTRL_ID, 0x0, TXRXSWITCH_TX);
    }

}


void dwt_configuretxrf(struct dwm3000_context *ctx, dwt_txconfig_t *config)
{
    if (config->PGcount == 0) {
        // Configure RF TX PG_DELAY
        original_dwt_write8bitoffsetreg(ctx, TX_CTRL_HI_ID, 0, config->PGdly);
    }
    else
    {
        uint8_t channel = 5;
        if (dwt_read8bitoffsetreg(ctx, CHAN_CTRL_ID, 0) & 0x1)
        {
            channel = 9;
        }
        dwt_calcbandwidthadj(ctx, config->PGcount, channel);
    }

    // Configure TX power
    dwt_write32bitreg(ctx, TX_POWER_ID, config->power);
}


int dwt_writetxdata(struct dwm3000_context *ctx, uint16_t txDataLength, uint8_t *txDataBytes, uint16_t txBufferOffset)
{
#ifdef DWT_API_ERROR_CHECK
    assert((pdw3000local->longFrames && (txDataLength <= EXT_FRAME_LEN)) ||\
           (txDataLength <= STD_FRAME_LEN));
    assert((txBufferOffset + txDataLength) < TX_BUFFER_MAX_LEN);
#endif

    if ((txBufferOffset + txDataLength) < TX_BUFFER_MAX_LEN)
    {
        if(txBufferOffset <= REG_DIRECT_OFFSET_MAX_LEN)
        {
            /* Directly write the data to the IC TX buffer */
            dwt_writetodevice(ctx, TX_BUFFER_ID, txBufferOffset, txDataLength, txDataBytes);
        }
        else
        {
            /* Program the indirect offset register A for specified offset to TX buffer */
            dwt_write32bitreg(ctx, INDIRECT_ADDR_A_ID, (TX_BUFFER_ID >> 16) );
            dwt_write32bitreg(ctx, ADDR_OFFSET_A_ID,   txBufferOffset);

            /* Indirectly write the data to the IC TX buffer */
            dwt_writetodevice(ctx, INDIRECT_POINTER_A_ID, 0, txDataLength, txDataBytes);
        }
        return DWT_SUCCESS;
    }
    else
        return DWT_ERROR;
}

void dwt_writetxfctrl(struct dwm3000_context *ctx, uint16_t txFrameLength, uint16_t txBufferOffset, uint8_t ranging)
{
    uint32_t reg32;
#ifdef DWT_API_ERROR_CHECK
    assert((pdw3000local->longFrames && (txFrameLength <= EXT_FRAME_LEN)) ||\
           (txFrameLength <= STD_FRAME_LEN));
#endif

    //DW3000/3700 - if offset is > 127, 128 needs to be added before data is written, this will be subtracted internally
    //prior to writing the data
    if(txBufferOffset <= 127)
    {
        // Write the frame length to the TX frame control register
        reg32 = txFrameLength | ((uint32_t)(txBufferOffset) << TX_FCTRL_TXB_OFFSET_BIT_OFFSET) | ((uint32_t)ranging << TX_FCTRL_TR_BIT_OFFSET);
        dwt_modify32bitoffsetreg(ctx, TX_FCTRL_ID, 0, ~(TX_FCTRL_TXB_OFFSET_BIT_MASK | TX_FCTRL_TR_BIT_MASK | TX_FCTRL_TXFLEN_BIT_MASK), reg32);
    }
    else
    {
        // Write the frame length to the TX frame control register
        reg32 = txFrameLength | ((uint32_t)(txBufferOffset + DWT_TX_BUFF_OFFSET_ADJUST) << TX_FCTRL_TXB_OFFSET_BIT_OFFSET) | ((uint32_t)ranging << TX_FCTRL_TR_BIT_OFFSET);
        dwt_modify32bitoffsetreg(ctx, TX_FCTRL_ID, 0, ~(TX_FCTRL_TXB_OFFSET_BIT_MASK | TX_FCTRL_TR_BIT_MASK | TX_FCTRL_TXFLEN_BIT_MASK), reg32);
        reg32 = dwt_read8bitoffsetreg(ctx, SAR_CTRL_ID, 0); //DW3000/3700 - need to read this to load the correct TX buffer offset value
    }

}

int dwt_starttx(struct dwm3000_context *ctx, uint8_t mode)
{
    int retval = DWT_SUCCESS ;
    uint16_t checkTxOK = 0 ;
    uint32_t sys_state;

    if ((mode & DWT_START_TX_DELAYED) || (mode & DWT_START_TX_DLY_REF)
            || (mode & DWT_START_TX_DLY_RS) || (mode & DWT_START_TX_DLY_TS))
    {
        if(mode & DWT_START_TX_DELAYED) //delayed TX
        {
            if(mode & DWT_RESPONSE_EXPECTED)
            {
                dwt_writefastCMD(ctx, CMD_DTX_W4R);
            }
            else
            {
                dwt_writefastCMD(ctx, CMD_DTX);
            }
        }
        else if (mode & DWT_START_TX_DLY_RS) //delayed TX WRT RX timestamp
        {
            if(mode & DWT_RESPONSE_EXPECTED)
            {
                dwt_writefastCMD(ctx, CMD_DTX_RS_W4R);
            }
            else
            {
                dwt_writefastCMD(ctx, CMD_DTX_RS);
            }
        }
        else if (mode & DWT_START_TX_DLY_TS) //delayed TX WRT TX timestamp
        {
            if(mode & DWT_RESPONSE_EXPECTED)
            {
                dwt_writefastCMD(ctx, CMD_DTX_TS_W4R);
            }
            else
            {
                dwt_writefastCMD(ctx, CMD_DTX_TS);
            }
        }
        else  //delayed TX WRT reference time
        {
            if(mode & DWT_RESPONSE_EXPECTED)
            {
                dwt_writefastCMD(ctx, CMD_DTX_REF_W4R);
            }
            else
            {
                dwt_writefastCMD(ctx, CMD_DTX_REF);
            }
        }

        checkTxOK = dwt_read8bitoffsetreg(ctx, SYS_STATUS_ID, 3); // Read at offset 3 to get the upper 2 bytes out of 5
        if ((checkTxOK & (SYS_STATUS_HPDWARN_BIT_MASK>>24)) == 0) // Transmit Delayed Send set over Half a Period away.
        {
            sys_state = dwt_read32bitreg(ctx, SYS_STATE_LO_ID);
            if (sys_state == DW_SYS_STATE_TXERR)
            {
                dwt_writefastCMD(ctx,CMD_TXRXOFF);
                retval = DWT_ERROR ; // Failed !
            }
            else
            {
                retval = DWT_SUCCESS ; // All okay
            }
        }
        else
        {
            dwt_writefastCMD(ctx, CMD_TXRXOFF);
            retval = DWT_ERROR ; // Failed !

            //optionally could return error, and still send the frame at indicated time
            //then if the application want to cancel the sending this can be done in a separate command.
        }
    }
    else if(mode & DWT_START_TX_CCA)
    {
        if(mode & DWT_RESPONSE_EXPECTED)
        {
            dwt_writefastCMD(ctx, CMD_CCA_TX_W4R);
        }
        else
        {
            dwt_writefastCMD(ctx, CMD_CCA_TX);
        }
    }
    else
    {
        if(mode & DWT_RESPONSE_EXPECTED)
        {
            dwt_writefastCMD(ctx, CMD_TX_W4R);
        }
        else
        {
            dwt_writefastCMD(ctx, CMD_TX);
        }
    }

    return retval;

} // end dwt_starttx()

int dwt_rxenable(struct dwm3000_context *ctx, int mode)
{
    uint8_t temp1 ;

    if(mode == DWT_START_RX_IMMEDIATE)
    {
        dwt_writefastCMD(ctx,CMD_RX);
    }
    else //delayed RX
    {
        switch(mode & ~DWT_IDLE_ON_DLY_ERR)
        {
            case DWT_START_RX_DELAYED:
                dwt_writefastCMD(ctx, CMD_DRX);
            break;
            case DWT_START_RX_DLY_REF:
                dwt_writefastCMD(ctx, CMD_DRX_REF);
            break;
            case DWT_START_RX_DLY_RS:
                dwt_writefastCMD(ctx, CMD_DRX_RS);
            break;
            case DWT_START_RX_DLY_TS:
                dwt_writefastCMD(ctx, CMD_DRX_TS);
            break;
            default:
                return DWT_ERROR; // return error
        }

        temp1 = dwt_read8bitoffsetreg(ctx, SYS_STATUS_ID, 3); // Read 1 byte at offset 3 to get the 4th byte out of 5
        if ((temp1 & (SYS_STATUS_HPDWARN_BIT_MASK >> 24)) != 0) // if delay has passed do immediate RX on unless DWT_IDLE_ON_DLY_ERR is true
        {
            dwt_writefastCMD(ctx, CMD_TXRXOFF);

            if((mode & DWT_IDLE_ON_DLY_ERR) == 0) // if DWT_IDLE_ON_DLY_ERR not set then re-enable receiver
            {
                dwt_writefastCMD(ctx, CMD_RX);
            }
            return DWT_ERROR; // return warning indication
        }
    }

    return DWT_SUCCESS;
}



/* Reference: ds_twr_initiator_sts.c - dwt_setrxantennadelay() sets RX antenna delay */
void dwt_setrxantennadelay(uint16_t delay)
{
    // TODO: Set RX antenna delay in device time units
}

/* Reference: ds_twr_initiator_sts.c - dwt_settxantennadelay() sets TX antenna delay */
void dwt_settxantennadelay(uint16_t delay)
{
    // TODO: Set TX antenna delay in device time units
}

/* Reference: ds_twr_initiator_sts.c - dwt_setrxaftertxdelay() sets RX after TX delay */
void dwt_setrxaftertxdelay(uint32_t delay)
{
    // TODO: Set delay between TX and RX enable
}

/* Reference: ds_twr_initiator_sts.c - dwt_setrxtimeout() sets RX timeout */
void dwt_setrxtimeout(uint16_t timeout)
{
    // TODO: Set RX timeout in UWB microseconds
}

/* Reference: ds_twr_initiator_sts.c - dwt_setpreambledetecttimeout() sets preamble detection timeout */
void dwt_setpreambledetecttimeout(uint16_t timeout)
{
    // TODO: Set preamble detection timeout
}

/* Reference: ds_twr_initiator_sts.c - dwt_setlnapamode() enables LNA/PA */
void dwt_setlnapamode(uint8_t mode)
{
    // TODO: Enable/disable LNA and PA modes
}

/* Reference: ds_twr_initiator_sts.c - dwt_setleds() configures diagnostic LEDs */
void dwt_setleds(uint8_t mode)
{
    // TODO: Configure LEDs for debugging
}

/* Reference: ds_twr_initiator_sts.c - dwt_readrxdata() reads RX frame data */
void dwt_readrxdata(uint8_t *buffer, uint16_t length, uint16_t offset)
{
    // TODO: Read data from RX buffer
}

/* Reference: ds_twr_initiator_sts.c - get_tx_timestamp_u64() gets TX timestamp */
uint64_t get_tx_timestamp_u64(void)
{
    // TODO: Read 64-bit TX timestamp
    return 0;
}

/* Reference: ds_twr_initiator_sts.c - get_rx_timestamp_u64() gets RX timestamp */
uint64_t get_rx_timestamp_u64(void)
{
    // TODO: Read 64-bit RX timestamp
    return 0;
}

/* Reference: ds_twr_initiator_sts.c - final_msg_set_ts() embeds timestamp in final message */
void final_msg_set_ts(uint8_t *ts_field, uint64_t ts)
{
    // TODO: Set timestamp in message payload
}

/* Reference: ds_twr_initiator_sts.c - dwt_setdelayedtrxtime() sets delayed TX time */
void dwt_setdelayedtrxtime(uint32_t time)
{
    // TODO: Set delayed TX time in device time units
}