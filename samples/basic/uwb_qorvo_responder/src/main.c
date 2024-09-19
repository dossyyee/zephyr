/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

#include <zephyr/drivers/ieee802154/deca_device_api.h>
#include <zephyr/drivers/ieee802154/deca_interface.h>
#include <zephyr/drivers/ieee802154/deca_types.h>
#include <zephyr/drivers/ieee802154/deca_version.h>

#include <zephyr/drivers/ieee802154/ieee802154_dw3xxx.h>

#include "shared_defines.h"
#include "shared_functions.h"


LOG_MODULE_REGISTER(app);

/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS   1000

#define DEVICE_ROLE_RESPONDER

/* The devicetree node identifier for the "led0" alias. */
#define LED0_NODE DT_ALIAS(led0)

/*
 * A build error on this line means your board is unsupported.
 * See the sample documentation for information on how to fix this.
 */
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
static const struct device *uwb = DEVICE_DT_GET(DT_INST(0, qorvo_dw3xxx));


/* Interface functions for qorvo API */
int readfromspi(uint16_t headerLength, uint8_t *headerBuffer, uint16_t readlength, uint8_t *readBuffer)
{
	return dw_spi_read(uwb, headerLength, headerBuffer, readlength, readBuffer);
}
int writetospi(uint16_t headerLength, const uint8_t *headerBuffer, uint16_t bodyLength, const uint8_t *bodyBuffer)
{
	return dw_spi_write(uwb, headerLength, headerBuffer, bodyLength, bodyBuffer);
}
int writetospiwithcrc(uint16_t headerLength, const uint8_t *headerBuffer, uint16_t bodyLength, const uint8_t *bodyBuffer, uint8_t crc8)
{
	return 0;
}
void setslowrate(void) 
{
	return;
}
void setfastrate(void) 
{
	return;
}
void wakeup_device_with_io(void)
{
	dw_wakeup(uwb);
}

decaIrqStatus_t decamutexon(void)
{
	const struct dw3xxx_config *cfg = uwb->config;
	if (cfg->irq_gpio.port) {
		gpio_pin_interrupt_configure_dt(&cfg->irq_gpio, GPIO_INT_DISABLE);
	}
	return 1;
}

void decamutexoff(decaIrqStatus_t s)
{
	ARG_UNUSED(s);
	const struct dw3xxx_config *cfg = uwb->config;
	if (cfg->irq_gpio.port) {
		gpio_pin_interrupt_configure_dt(&cfg->irq_gpio, GPIO_INT_EDGE_RISING);
	}
}

void deca_sleep(unsigned int time_ms) {
	k_msleep(time_ms);
}

void deca_usleep(unsigned long time_us) {

	if  (time_us <= UINT16_MAX) {
		k_busy_wait((uint32_t)time_us);
	}
	else {
		k_msleep((int32_t)(time_us >> 10));
	}
}

/* Qorvo variables */
static const struct dwt_spi_s dw3000_spi_fct = {
    .readfromspi = readfromspi,
    .writetospi = writetospi,
    .writetospiwithcrc = writetospiwithcrc,
    .setslowrate = setslowrate,
    .setfastrate = setfastrate,
};

const struct dwt_probe_s dw3000_probe_interf = 
{
    .dw = NULL,
    .spi = (void*)&dw3000_spi_fct,
    .wakeup_device_with_io = wakeup_device_with_io,
};

//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

/* Example application name and version to display on LCD screen. */
#define APP_NAME "DSTWR RE STS-SDC v1.0"

/* Default communication configuration. We use STS with SDC DW mode. */
static dwt_config_t config = {
    5,               /* Channel number. */
    DWT_PLEN_64,     /* Preamble length. Used in TX only. */
    DWT_PAC8,        /* Preamble acquisition chunk size. Used in RX only. */
    9,               /* TX preamble code. Used in TX only. */
    9,               /* RX preamble code. Used in RX only. */
    1,               /* 0 to use standard 8 symbol SFD, 1 to use non-standard 8 symbol, 2 for non-standard 16 symbol SFD and 3 for 4z 8 symbol SDF type */
    DWT_BR_6M8,      /* Data rate. */
    DWT_PHRMODE_STD, /* PHY header mode. */
    DWT_PHRRATE_STD, /* PHY header rate. */
    (65 + 8 - 8),    /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
    DWT_STS_MODE_1 | DWT_STS_MODE_SDC, /* STS mode 1 with SDC see NOTE on SDC below*/
    DWT_STS_LEN_64,                    /* STS length see allowed values in Enum dwt_sts_lengths_e */
    DWT_PDOA_M0                        /* PDOA mode off */
};

/* have some delay after each range (e.g. so LDC can be updated (on ARM eval boards), needs to be slightly less than RNG_DELAY_MS in the initiator example*/
#define DELAY_MS 980

/* Default antenna delay values for 64 MHz PRF. See NOTE 1 below. */
#define TX_ANT_DLY 16385
#define RX_ANT_DLY 16385

/* Frames used in the ranging process. See NOTE 2 below. */
static uint8_t rx_poll_msg[] = { 0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0x21 };
static uint8_t tx_resp_msg[] = { 0x41, 0x88, 0, 0xCA, 0xDE, 'V', 'E', 'W', 'A', 0x10, 0x02, 0, 0 };
static uint8_t rx_final_msg[] = { 0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0x23, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
/* Length of the common part of the message (up to and including the function code, see NOTE 2 below). */
#define ALL_MSG_COMMON_LEN 10
/* Index to access some of the fields in the frames involved in the process. */
#define ALL_MSG_SN_IDX            2
#define FINAL_MSG_POLL_TX_TS_IDX  10
#define FINAL_MSG_RESP_RX_TS_IDX  14
#define FINAL_MSG_FINAL_TX_TS_IDX 18
/* Frame sequence number, incremented after each transmission. */
static uint8_t frame_seq_nb = 0;

/* Buffer to store received messages.
 * Its size is adjusted to longest frame that this example code is supposed to handle. */
#define RX_BUF_LEN 24
static uint8_t rx_buffer[RX_BUF_LEN];

/* Hold copy of status register state here for reference so that it can be examined at a debug breakpoint. */
static uint32_t status_reg = 0;

/* Delay between frames, in UWB microseconds. See NOTE 4 below. */
/* This is the delay from Frame RX timestamp to TX reply timestamp used for calculating/setting the DW3000's delayed TX function. This includes the
 * frame length of approximately 180 us with above configuration. */
#define POLL_RX_TO_RESP_TX_DLY_UUS 900
/* This is the delay from the end of the frame transmission to the enable of the receiver, as programmed for the DW3000's wait for response feature. */
#define RESP_TX_TO_FINAL_RX_DLY_UUS 670
/* Receive final timeout. See NOTE 5 below. */
#define FINAL_RX_TIMEOUT_UUS 300
/* Preamble timeout, in multiple of PAC size. See NOTE 6 below. */
#define PRE_TIMEOUT 5

/* Timestamps of frames transmission/reception. */
static uint64_t poll_rx_ts;
static uint64_t resp_tx_ts;
static uint64_t final_rx_ts;

/* Hold copies of computed time of flight and distance here for reference so that it can be examined at a debug breakpoint. */
static double tof;
static double distance;

/* Values for the PG_DELAY and TX_POWER registers reflect the bandwidth and power of the spectrum at the current
 * temperature. These values can be calibrated prior to taking reference measurements. See NOTE 2 below. */
extern dwt_txconfig_t txconfig_options;

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn ds_twr_sts_sdc_responder()
 *
 * @brief Application entry point.
 *
 * @param  none
 *
 * @return none
 */
int main(void)
{
    int range_ok = 0;
    
    /* Display application name on LCD. */
    printk("DSTWR RE STS-SDC v1.0\n");

    /* Configure SPI rate, DW3000 supports up to 36 MHz */
    //port_set_dw_ic_spi_fastrate();

    /* Reset DW IC */
    dw_reset(uwb); /* Target specific drive of RSTn line into DW IC low for a period. */

    k_msleep(2); // Time needed for DW3000 to start up (transition from INIT_RC to IDLE_RC

    /* Probe for the correct device driver. */
    dwt_probe((struct dwt_probe_s *)&dw3000_probe_interf);

    while (!dwt_checkidlerc()) /* Need to make sure DW IC is in IDLE_RC before proceeding */ { };

    if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR)
    {
        printk("INIT FAILED     \n");
        while (1) { };
    }

    /* Configure DW IC. See NOTE 15 below. */
    /* if the dwt_configure returns DWT_ERROR either the PLL or RX calibration has failed the host should reset the device */
    if (dwt_configure(&config))
    {
        printk((unsigned char *)"CONFIG FAILED     \n");
        while (1) { };
    }

    /* Configure the TX spectrum parameters (power, PG delay and PG count) */
    dwt_configuretxrf(&txconfig_options);

    /* Apply default antenna delay value. See NOTE 1 below. */
    dwt_setrxantennadelay(RX_ANT_DLY);
    dwt_settxantennadelay(TX_ANT_DLY);

    /* Next can enable TX/RX states output on GPIOs 5 and 6 to help debug, and also TX/RX LEDs
     * Note, in real low power applications the LEDs should not be used. */
    dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE);

    /* Loop forever responding to ranging requests. */
    while (1)
    {
        k_msleep(1);
        /* turn off preamble timeout as the responder does not know when the poll is coming. */
        dwt_setpreambledetecttimeout(0);
        /* Clear reception timeout to start next ranging process. */
        dwt_setrxtimeout(0);

        /* Activate reception immediately. */
        dwt_rxenable(DWT_START_RX_IMMEDIATE);

        /* Poll for reception of a frame or error/timeout. See NOTE 8 below. */
        waitforsysstatus(&status_reg, NULL, (DWT_INT_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR), 0);

        if (status_reg & DWT_INT_RXFCG_BIT_MASK)
        {
            uint16_t frame_len;
            int goodSts = 0;    /* Used for checking STS quality in received signal */
            int16_t stsQual;    /* This will contain STS quality index */
            uint16_t stsStatus; /* Used to check for good STS status (no errors). */

            /* Clear good RX frame event in the DW3000 status register. */
            dwt_writesysstatuslo(DWT_INT_RXFCG_BIT_MASK);

            // as STS mode is used, we only consider frames that are received with good STS quality
            if (((goodSts = dwt_readstsquality(&stsQual)) >= 0) && (dwt_readstsstatus(&stsStatus, 0) == DWT_SUCCESS)) // if STS is good this will be true >= 0
            {
                /* A frame has been received, read it into the local buffer. */
                frame_len = dwt_getframelength();
                if (frame_len <= RX_BUF_LEN)
                {
                    dwt_readrxdata(rx_buffer, frame_len, 0);
                }

                /* Check that the frame is a poll sent by "DS TWR initiator" example.
                 * As the sequence number field of the frame is not relevant, it is cleared to simplify the validation of the frame. */
                rx_buffer[ALL_MSG_SN_IDX] = 0;
                if (memcmp(rx_buffer, rx_poll_msg, ALL_MSG_COMMON_LEN) == 0)
                {
                    uint32_t resp_tx_time;
                    int ret;

                    /* Retrieve poll reception timestamp. */
                    poll_rx_ts = get_rx_timestamp_u64();

                    /* Set send time for response. See NOTE 9 below. */
                    resp_tx_time = (poll_rx_ts + (POLL_RX_TO_RESP_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;
                    dwt_setdelayedtrxtime(resp_tx_time);

                    /* Set expected delay and timeout for final message reception. See NOTE 4 and 5 below. */
                    dwt_setrxaftertxdelay(RESP_TX_TO_FINAL_RX_DLY_UUS);
                    dwt_setrxtimeout(FINAL_RX_TIMEOUT_UUS);

                    /* Write and send the response message. See NOTE 10 below.*/
                    tx_resp_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
                    dwt_writetxdata(sizeof(tx_resp_msg), tx_resp_msg, 0);  /* Zero offset in TX buffer. */
                    dwt_writetxfctrl(sizeof(tx_resp_msg) + FCS_LEN, 0, 1); /* Zero offset in TX buffer, ranging. */
                    /* Set preamble timeout for expected final frame from the initiator. See NOTE 6 below. */
                    dwt_setpreambledetecttimeout(PRE_TIMEOUT);
                    ret = dwt_starttx(DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED);

                    /* If dwt_starttx() returns an error, abandon this ranging exchange and proceed to the next one. See NOTE 11 below. */
                    if (ret == DWT_ERROR)
                    {
                        continue;
                    }

                    /* Poll for reception of expected "final" frame or error/timeout. See NOTE 8 below. */
                    waitforsysstatus(&status_reg, NULL, (DWT_INT_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR), 0);

                    /* Increment frame sequence number after transmission of the response message (modulo 256). */
                    frame_seq_nb++;

                    if (status_reg & DWT_INT_RXFCG_BIT_MASK)
                    {
                        /* Clear good RX frame event and TX frame sent in the DW3000 status register. */
                        dwt_writesysstatuslo(DWT_INT_RXFCG_BIT_MASK | DWT_INT_TXFRS_BIT_MASK);

                        // as STS mode is used, we only consider frames that are received with good STS quality
                        if (((goodSts = dwt_readstsquality(&stsQual)) >= 0)
                            && (dwt_readstsstatus(&stsStatus, 0) == DWT_SUCCESS)) // if STS is good this will be true >= 0
                        {
                            /* A frame has been received, read it into the local buffer. */
                            frame_len = dwt_getframelength();
                            if (frame_len <= RX_BUF_LEN)
                            {
                                dwt_readrxdata(rx_buffer, frame_len, 0);
                            }

                            /* Check that the frame is a final message sent by "DS TWR initiator" example.
                             * As the sequence number field of the frame is not used in this example, it can be zeroed to ease the validation of the frame. */
                            rx_buffer[ALL_MSG_SN_IDX] = 0;
                            if (memcmp(rx_buffer, rx_final_msg, ALL_MSG_COMMON_LEN) == 0)
                            {
                                uint32_t poll_tx_ts, resp_rx_ts, final_tx_ts;
                                uint32_t poll_rx_ts_32, resp_tx_ts_32, final_rx_ts_32;
                                double Ra, Rb, Da, Db;
                                int64_t tof_dtu;

                                /* Retrieve response transmission and final reception timestamps. */
                                resp_tx_ts = get_tx_timestamp_u64();
                                final_rx_ts = get_rx_timestamp_u64();

                                /* Get timestamps embedded in the final message. */
                                final_msg_get_ts(&rx_buffer[FINAL_MSG_POLL_TX_TS_IDX], &poll_tx_ts);
                                final_msg_get_ts(&rx_buffer[FINAL_MSG_RESP_RX_TS_IDX], &resp_rx_ts);
                                final_msg_get_ts(&rx_buffer[FINAL_MSG_FINAL_TX_TS_IDX], &final_tx_ts);

                                /* Compute time of flight. 32-bit subtractions give correct answers even if clock has wrapped. See NOTE 12 below. */
                                poll_rx_ts_32 = (uint32_t)poll_rx_ts;
                                resp_tx_ts_32 = (uint32_t)resp_tx_ts;
                                final_rx_ts_32 = (uint32_t)final_rx_ts;
                                Ra = (double)(resp_rx_ts - poll_tx_ts);
                                Rb = (double)(final_rx_ts_32 - resp_tx_ts_32);
                                Da = (double)(final_tx_ts - resp_rx_ts);
                                Db = (double)(resp_tx_ts_32 - poll_rx_ts_32);
                                tof_dtu = (int64_t)((Ra * Rb - Da * Db) / (Ra + Rb + Da + Db));

                                tof = tof_dtu * DWT_TIME_UNITS;
                                distance = tof * SPEED_OF_LIGHT;

                                range_ok = 1;

								printk("Distance: %f\n", distance);
                            }
                        } // if STS good on the Final message reception
                    }
                    else
                    {
                        /* Clear RX error/timeout events in the DW3000 status register. */
                        dwt_writesysstatuslo(SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
                    }
                }
            } // if STS good on the Poll message reception
        }
        else
        {
            /* Clear RX error/timeout events in the DW3000 status register. */
            dwt_writesysstatuslo(SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
        }

        /* add some delay before next ranging exchange */
        if (range_ok)
        {
            range_ok = 0;
            k_msleep(DELAY_MS);
        }
    }
}
