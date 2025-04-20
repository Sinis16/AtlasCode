/*! ----------------------------------------------------------------------------
 *  @file    ds_twr_initiator_sts.c
 *  @brief   Double-sided two-way ranging (DS TWR) initiator example code
 *
 *           This is a simple code example which acts as the initiator in a 
 *           DS TWR distance measurement exchange. This application sends a 
 *           "poll" frame (recording the TX time-stamp of the poll), after 
 *           which it waits for a "response" message from the 
 *           "ds_twr_responder_sts" example code (companion to this application)
 *           to complete the exchange.
 *
 *           This example utilises the 802.15.4z STS to accomplish secure 
 *           timestamps between the initiator and responder. A 32-bit STS 
 *           counter is part of the STS IV used to generate the scrambled 
 *           timestamp sequence (STS) in the transmitted packet and to cross 
 *           correlate in the receiver. 
 *           This count normally advances by 1 for every 1024 chips (~2�s) of 
 *           STS in BPRF mode, and by 1 for every 512 chips (~1�s) of STS
 *           in HPRF mode. If both devices (initiator and responder) have count 
 *           values that are synced, then the communication between devices 
 *           should result in secure timestamps which can be used to calculate 
 *           distance. 
 *           If not, then the devices need to re-sync their STS counter values.
 *           In this example, the initiator will send a plain-text value of 
 *           it's 32-bit STS counter inside the "poll" frame. The receiver first
 *           checks the quality of the STS of the received frame. If the 
 *           received frame has bad STS quality, it can then use the plain-text
 *           counter value received to adjust it's own STS counter value to match.
 *           This means that the next message in the sequence should be in sync 
 *           again.
 *
 * @attention
 *
 * Copyright 2019 - 2020 (c) Decawave Ltd, Dublin, Ireland.
 * Copyright 2021 (c) Callender-Consulting, LLC  (port to Zephyr)
 *
 * All rights reserved.
 *
 * @author Decawave
 */
#include <deca_device_api.h>
#include <deca_regs.h>
#include <deca_spi.h>
#include <port.h>
#include <shared_defines.h>
#include <shared_functions.h>

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

/* Example application name */
#define APP_NAME "DS TWR INIT v1.0"

/* Default communication configuration */
static dwt_config_t config = {
    5,               /* Channel number */
    DWT_PLEN_128,    /* Preamble length */
    DWT_PAC8,        /* Preamble acquisition chunk size */
    9,               /* TX preamble code */
    9,               /* RX preamble code */
    1,               /* 0 for std 8 symbol SFD, 1 for non-std 8, 2 for non-std 16, 3 for 4z 8 */
    DWT_BR_6M8,      /* Data rate */
    DWT_PHRMODE_STD, /* PHY header mode */
    DWT_PHRRATE_STD, /* PHY header rate */
    (129 + 8 - 8),   /* SFD timeout (preamble length + 1 + SFD length - PAC size) */
    DWT_STS_MODE_OFF,/* STS disabled */
    DWT_STS_LEN_64,  /* STS length */
    DWT_PDOA_M0      /* PDOA mode off */
};

/* Inter-ranging delay period, in milliseconds */
#define RNG_DELAY_MS 1000

/* Default antenna delay values for 64 MHz PRF */
#define TX_ANT_DLY 16385
#define RX_ANT_DLY 16385

/* Frames used in the ranging process */
static uint8_t tx_poll_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0x21};
static uint8_t rx_resp_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'V', 'E', 'W', 'A', 0x10, 0x02, 0, 0};
static uint8_t tx_final_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0x23, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

/* Length of common message part */
#define ALL_MSG_COMMON_LEN 10

/* Indexes for frame fields */
#define ALL_MSG_SN_IDX 2
#define FINAL_MSG_POLL_TX_TS_IDX 10
#define FINAL_MSG_RESP_RX_TS_IDX 14
#define FINAL_MSG_FINAL_TX_TS_IDX 18

/* Frame sequence number */
static uint8_t frame_seq_nb = 0;

/* Buffer for received response */
#define RX_BUF_LEN 20
static uint8_t rx_buffer[RX_BUF_LEN];

/* Status register copy */
static uint32_t status_reg = 0;

/* Delays in UWB microseconds */
#define POLL_TX_TO_RESP_RX_DLY_UUS 700
#define RESP_RX_TO_FINAL_TX_DLY_UUS 700
#define RESP_RX_TIMEOUT_UUS 300
#define PRE_TIMEOUT 5

/* Timestamps */
static uint64_t poll_tx_ts;
static uint64_t resp_rx_ts;
static uint64_t final_tx_ts;

/* TX config for spectrum */
extern dwt_txconfig_t txconfig_options;

/*! ---------------------------------------------------------------------------
 * @fn ds_twr_initiator()
 *
 * @brief Application entry point.
 *
 * @param  none
 *
 * @return none
 */
/* ... [keep includes and globals] ... */

int app_main(void)
{
    /* Print ASAP to confirm entry */
    printk("Entering app_main()\n");
    printk("%s\n", APP_NAME);

    port_set_dw_ic_spi_fastrate();
    printk("SPI configured\n");

    reset_DWIC(); 
    Sleep(2);
    printk("DW IC reset\n");

    while (!dwt_checkidlerc()) { 
        printk("Waiting for IDLE_RC...\n");
        Sleep(1); // Slow it down to catch output
    };
    printk("IDLE_RC reached\n");

    if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR) {
        printk("INIT FAILED\n");
        while (1) { /* spin */ };
    }
    printk("DW IC initialized\n");

    if (dwt_configure(&config)) {
        printk("CONFIG FAILED\n");
        while (1) { /* spin */ };
    }
    printk("DW IC configured\n");

    dwt_configuretxrf(&txconfig_options);
    printk("TX RF configured\n");

    dwt_setrxantennadelay(RX_ANT_DLY);
    dwt_settxantennadelay(TX_ANT_DLY);
    printk("Antenna delays set\n");

    dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
    dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);
    dwt_setpreambledetecttimeout(PRE_TIMEOUT);
    printk("RX delays set\n");

    dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE);
    dwt_setleds(DWT_LEDS_ENABLE | DWT_LEDS_INIT_BLINK);
    printk("LNA/PA and LEDs enabled\n");

    printk("Initiator ready\n");
    printk("DS TWR Initiator started - waiting for responder\n");

    while (1) {
        printk("Polling for responder...\n");

        tx_poll_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
        dwt_writetxdata(sizeof(tx_poll_msg), tx_poll_msg, 0);
        dwt_writetxfctrl(sizeof(tx_poll_msg)+FCS_LEN, 0, 1);

        dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);

        while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & 
                (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)))
        { /* spin */ };

        frame_seq_nb++;

        if (status_reg & SYS_STATUS_RXFCG_BIT_MASK) {
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_TXFRS_BIT_MASK);

            uint32_t frame_len = dwt_read32bitreg(RX_FINFO_ID) & FRAME_LEN_MAX_EX;
            if (frame_len <= RX_BUF_LEN) {
                dwt_readrxdata(rx_buffer, frame_len, 0);
            }

            rx_buffer[ALL_MSG_SN_IDX] = 0;
            if (memcmp(rx_buffer, rx_resp_msg, ALL_MSG_COMMON_LEN) == 0) {
                uint32_t final_tx_time;

                poll_tx_ts = get_tx_timestamp_u64();
                resp_rx_ts = get_rx_timestamp_u64();

                final_tx_time = (resp_rx_ts + (RESP_RX_TO_FINAL_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;
                dwt_setdelayedtrxtime(final_tx_time);

                final_tx_ts = (((uint64_t)(final_tx_time & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY;

                final_msg_set_ts(&tx_final_msg[FINAL_MSG_POLL_TX_TS_IDX], poll_tx_ts);
                final_msg_set_ts(&tx_final_msg[FINAL_MSG_RESP_RX_TS_IDX], resp_rx_ts);
                final_msg_set_ts(&tx_final_msg[FINAL_MSG_FINAL_TX_TS_IDX], final_tx_ts);

                tx_final_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
                dwt_writetxdata(sizeof(tx_final_msg), tx_final_msg, 0);
                dwt_writetxfctrl(sizeof(tx_final_msg)+FCS_LEN, 0, 1);

                int ret = dwt_starttx(DWT_START_TX_DELAYED);
                if (ret == DWT_SUCCESS) {
                    while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS_BIT_MASK))
                    { /* spin */ };
                    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);
                    frame_seq_nb++;
                }
            }
        }
        else {
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR | SYS_STATUS_TXFRS_BIT_MASK);
        }

        Sleep(RNG_DELAY_MS);
    }
}

/*****************************************************************************************************************************************************
 * NOTES:
 *
 * 1. The double-sided two-way ranging scheme implemented here has to be considered carefully as the accuracy of the distance measured is highly
 *    sensitive to the clock offset error between the devices and the length of the response delay between frames. To achieve the best possible
 *    accuracy, this response delay must be kept as low as possible. In order to do so, 6.8 Mbps data rate is used in this example and the response
 *    delay between frames is defined as low as possible. The user is referred to User Manual for more details about the double-sided two-way ranging
 *    process.  NB:SEE ALSO NOTE 14.
 * 2. The sum of the values is the TX to RX antenna delay, this should be experimentally determined by a calibration process. Here we use a hard coded
 *    value (expected to be a little low so a positive error will be seen on the resultant distance estimate). For a real production application, each
 *    device should have its own antenna delay properly calibrated to get good precision when performing range measurements.
 * 3. The frames used here are Decawave specific ranging frames, complying with the IEEE 802.15.4 standard data frame encoding. The frames are the
 *    following:
 *     - a poll message sent by the initiator to trigger the ranging exchange.
 *     - a response message sent by the responder to complete the exchange and provide all information needed by the initiator to compute the
 *       time-of-flight (distance) estimate.
 *    The first 10 bytes of those frame are common and are composed of the following fields:
 *     - byte 0/1: frame control (0x8841 to indicate a data frame using 16-bit addressing).
 *     - byte 2: sequence number, incremented for each new frame.
 *     - byte 3/4: PAN ID (0xDECA).
 *     - byte 5/6: destination address, see NOTE 4 below.
 *     - byte 7/8: source address, see NOTE 4 below.
 *     - byte 9: function code (specific values to indicate which message it is in the ranging process).
 *    The remaining bytes are specific to each message as follows:
 *    Poll message:
 *     - no more data
 *    Response message:
 *     - byte 10 -> 13: poll message reception timestamp.
 *     - byte 14 -> 17: response message transmission timestamp.
 *    All messages end with a 2-byte checksum automatically set by DW IC.
 * 4. Source and destination addresses are hard coded constants in this example to keep it simple but for a real product every device should have a
 *    unique ID. Here, 16-bit addressing is used to keep the messages as short as possible but, in an actual application, this should be done only
 *    after an exchange of specific messages used to define those short addresses for each device participating to the ranging exchange.
 * 5. This timeout is for complete reception of a frame, i.e. timeout duration must take into account the length of the expected frame. Here the value
 *    is arbitrary but chosen large enough to make sure that there is enough time to receive the complete response frame sent by the responder at the
 *    6.8M data rate used (around 200 �s).
 * 6. In a real application, for optimum performance within regulatory limits, it may be necessary to set TX pulse bandwidth and TX power, (using
 *    the dwt_configuretxrf API call) to per device calibrated values saved in the target system or the DW IC OTP memory.
 * 7. dwt_writetxdata() takes the full size of the message as a parameter but only copies (size - 2) bytes as the check-sum at the end of the frame is
 *    automatically appended by the DW IC. This means that our variable could be two bytes shorter without losing any data (but the sizeof would not
 *    work anymore then as we would still have to indicate the full length of the frame to dwt_writetxdata()).
 * 8. We use polled mode of operation here to keep the example as simple as possible but all status events can be used to generate interrupts. Please
 *    refer to DW IC User Manual for more details on "interrupts". It is also to be noted that STATUS register is 5 bytes long but, as the event we
 *    use are all in the first bytes of the register, we can use the simple dwt_read32bitreg() API call to access it instead of reading the whole 5
 *    bytes.
 * 9. The high order byte of each 40-bit time-stamps is discarded here. This is acceptable as, on each device, those time-stamps are not separated by
 *    more than 2**32 device time units (which is around 67 ms) which means that the calculation of the round-trip delays can be handled by a 32-bit
 *    subtraction.
 * 10. The user is referred to DecaRanging ARM application (distributed with EVK1000 product) for additional practical example of usage, and to the
 *     DW IC API Guide for more details on the DW IC driver functions.
 * 11. The use of the clock offset value to correct the TOF calculation, significantly improves the result of the SS-TWR where the remote
 *     responder unit's clock is a number of PPM offset from the local initiator unit's clock.
 *     As stated in NOTE 2 a fixed offset in range will be seen unless the antenna delay is calibrated and set correctly.
 * 12. In this example, the DW IC is put into IDLE state after calling dwt_initialise(). This means that a fast SPI rate of up to 20 MHz can be used
 *     thereafter.
 * 13. This example uses the 802.15.4z STS with a packet configuration of mode 1 which looks like so:
 *     ---------------------------------------------------
 *     | Ipatov Preamble | SFD | STS | PHR | PHY Payload |
 *     ---------------------------------------------------
 *     There is a possibility that the TX and RX units in this example will go out of sync as their STS IV values may be misaligned. The STS IV value
 *     changes upon each receiving and transmitting event by the chip. While the TX and RX devices in this example start at the same STS IV values, it
 *     is possible that they can go out sync if a signal is not received correctly, devices are out of range, etc. To combat this, the 'poll message'
 *     that the initiator sends to the responder contains a plain-text STS counter value. The responder receives this message and first checks if
 *     the received frame is out of sync with it's own counter. If so, it will use this received counter value to update it's own counter. When out
 *     of sync with each other, the STS will not align correctly - thus we get no secure timestamp values.
 * 14. The receiver is enabled with reference to the timestamp of the previously received signal.
 *     The receiver will start after a defined delay.
 *     This defined delay is currently the same as the delay between the responder's received
 *     timestamp of it's last received frame and the timestamp of the transmitted signal that is
 *     sent in response.
 *     This means that the initiator needs to reduce it's delay by the configured preamble length.
 *     This allows for the receiver to enable on the initiator at the same time as responder is
 *     transmitting it's message. It should look something like this:
 *
 *     Initiator: |Poll TX| ..... |Resp RX| ........ |Final TX|
 *     Responder: |Poll RX| ..... |Resp TX| ........ |Final RX|
 *                    ^|P RMARKER|                                    - time of Poll TX/RX
 *                                    ^|R RMARKER|                    - time of Resp TX/RX
 *                                                       ^|R RMARKER| - time of Final TX/RX
 *
 *                        <--TDLY->                                   - POLL_TX_TO_RESP_RX_DLY_UUS (RDLY-RLEN)
 *                                <-RLEN->                            - RESP_RX_TIMEOUT_UUS   (length of poll frame)
 *                     <----RDLY------>                               - POLL_RX_TO_RESP_TX_DLY_UUS (depends on how quickly responder
 *                                                                                                                       can turn around and reply)
 *
 *
 *                                         <--T2DLY->                 - RESP_TX_TO_FINAL_RX_DLY_UUS (R2DLY-FLEN)
 *                                                   <-FLEN--->       - FINAL_RX_TIMEOUT_UUS   (length of response frame)
 *                                     <----RDLY--------->            - RESP_RX_TO_FINAL_TX_DLY_UUS (depends on how quickly initiator
 *                                                                                                                       can turn around and reply)
 * 15. Desired configuration by user may be different to the current programmed configuration. dwt_configure is called to set desired
 *     configuration.
 * 16. This example will set the STS key and IV upon each iteration of the main while loop. While this has the benefit of keeping the STS count in
 *     sync with the responder device (which does the same), it should be noted that this is not a 'secure' implementation as the count is reset upon
 *     each iteration of the loop. An attacker could potentially recognise this pattern if the signal was being monitored. While it serves it's
 *     purpose in this simple example, it should not be utilised in any final solution.
 * 17. Delays between frames have been chosen here to ensure proper synchronisation of transmission and reception of the frames between the initiator
 *     and the responder and to ensure a correct accuracy of the computed distance. The user is referred to DecaRanging ARM Source Code Guide for more
 *     details about the timings involved in the ranging process.
 *
 *  EXAMPLE 1: with SPI rate set to 18 MHz (default on this platform), and frame lengths of ~190 us, the delays can be set to:
 *             POLL_RX_TO_RESP_TX_DLY_UUS of 400uus, and RESP_RX_TO_FINAL_TX_DLY_UUS of 400uus (TXtoRX delays are set to 210uus)
 *             reducing the delays further can be achieved by using interrupt to handle the TX/RX events, or other code optimisations/faster SPI
 *
 *  EXAMPLE 2: with SPI rate set to 4.5 MHz, and frame lengths of ~190 us, the delays can be set to:
 *             POLL_RX_TO_RESP_TX_DLY_UUS of 550uus, and RESP_RX_TO_FINAL_TX_DLY_UUS of 600uus (TXtoRX delays are set to 360 and 410 uus respectively)
 *
 * 18. This timeout is for complete reception of a frame, i.e. timeout duration must take into account the length of the expected frame. Here the value
 *     is arbitrary but chosen large enough to make sure that there is enough time to receive the complete response frame sent by the responder at the
 *     6.81 Mbps data rate used (around 200 us).
 * 19. As we want to send final TX timestamp in the final message, we have to compute it in advance instead of relying on the reading of DW IC
 *     register. Timestamps and delayed transmission time are both expressed in device time units so we just have to add the desired response delay to
 *     response RX timestamp to get final transmission time. The delayed transmission time resolution is 512 device time units which means that the
 *     lower 9 bits of the obtained value must be zeroed. This also allows to encode the 40-bit value in a 32-bit words by shifting the all-zero lower
 *     8 bits.
 ****************************************************************************************************************************************************/
