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


LOG_MODULE_REGISTER(app);

/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS   1000

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

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn get_tx_timestamp_u64()
 *
 * @brief Get the TX time-stamp in a 64-bit variable.
 *        /!\ This function assumes that length of time-stamps is 40 bits, for both TX and RX!
 *
 * @param  none
 *
 * @return  64-bit value of the read time-stamp.
 */
uint64_t get_tx_timestamp_u64(void)
{
    uint8_t ts_tab[5];
    uint64_t ts = 0;
    int8_t i;
    dwt_readtxtimestamp(ts_tab);
    for (i = 4; i >= 0; i--)
    {
        ts <<= 8;
        ts |= ts_tab[i];
    }
    return ts;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn get_rx_timestamp_u64()
 *
 * @brief Get the RX time-stamp in a 64-bit variable.
 *        /!\ This function assumes that length of time-stamps is 40 bits, for both TX and RX!
 *
 * @param  none
 *
 * @return  64-bit value of the read time-stamp.
 */
uint64_t get_rx_timestamp_u64(void)
{
    uint8_t ts_tab[5];
    uint64_t ts = 0;
    int8_t i;
    dwt_readrxtimestamp(ts_tab);
    for (i = 4; i >= 0; i--)
    {
        ts <<= 8;
        ts |= ts_tab[i];
    }
    return ts;
}

void waitforsysstatus(uint32_t *lo_result, uint32_t *hi_result, uint32_t lo_mask, uint32_t hi_mask)
{
    uint32_t lo_result_tmp = 0;
    uint32_t hi_result_tmp = 0;

    int cnt = 0;

    // If a mask has been passed into the function for the system status register (lower 32-bits)
    if (lo_mask)
    {
        while (!((lo_result_tmp = dwt_readsysstatuslo()) & (lo_mask)))
        {
            // If a mask value is set for the system status register (higher 32-bits)
            if (hi_mask)
            {
                // If mask value for the system status register (higher 32-bits) is found
                if ((hi_result_tmp = dwt_readsysstatushi()) & hi_mask)
                {
                    break;
                }
            }

	    if (cnt >= 100) {
		cnt = 0;
	    	k_msleep(1);
	    }

	    cnt++;
        }
    }
    // if only a mask value for the system status register (higher 32-bits) is set
    else if (hi_mask)
    {
        while (!((hi_result_tmp = dwt_readsysstatushi()) & (hi_mask))) { };
    }

    if (lo_result != NULL)
    {
        *lo_result = lo_result_tmp;
    }

    if (hi_result != NULL)
    {
        *hi_result = hi_result_tmp;
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

/* MAC configuration. STS mode 3. */
static dwt_config_t config = {
	9,			/* Channel number 9. */
	DWT_PLEN_64,		/* Preamble length. Used in TX only. */
	DWT_PAC8,		/* Preamble acquisition chunk size. Used in RX only. DWT_PAC4 alt */
	9,			/* TX preamble code. Used in TX only. */
	9,			/* RX preamble code. Used in RX only. */
	DWT_SFD_IEEE_4Z,	/* 4z 8 symbol SDF type */
	DWT_BR_6M8,		/* Data rate. Not applicable for STS mode 3 */
	DWT_PHRMODE_STD,	/* PHY header mode. */
	DWT_PHRRATE_STD,	/* PHY header rate. */
	(64 + 1 + 8 - 8),	/* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
	DWT_STS_MODE_ND,	/* STS mode 3. */
	DWT_STS_LEN_64,		/* STS length see allowed values in Enum dwt_sts_lengths_e */
	DWT_PDOA_M0		/* PDOA mode off */
};

dwt_txconfig_t txconfig_options_ch9 = {
    0x34,       /* PG delay. */
    0xfefefefe, /* TX power. */
    0x0         /*PG count*/
};

dwt_sts_cp_key_t sts_key = {
	0x12345678,
	0x90abcdef,
	0xfedcba09,
	0x87654321
};

dwt_sts_cp_iv_t sts_iv = {
	0x00000000,
	0x00000000,
	0x00000000,
	0x00000001
};



#define CPU_PROCESSING_TIME 400
/* Length of frame according with 64mhz PRF, STS Mode 3, 8 symbol SFD, 64 length SFD, 64 length Preamble */
#define FRAME_LENGTH_US 140
/* Inter-ranging delay period, in milliseconds. */
#define RNG_DELAY_MS 1000
/* Default antenna delay values for 64 MHz PRF. Tune these values experimentally for correct distance measurement calibration. */
#define TX_ANT_DLY 16385
#define RX_ANT_DLY 16385
/* Frame sequence number, incremented after each transmission. */
//static uint8_t frame_seq_nb = 0;
/* Hold copy of status register state here for reference so that it can be examined at a debug breakpoint. */
static uint32_t status_reg = 0;
/* Receive response timeout. */
#define RX_TIMEOUT_UUS 300
/* Preamble timeout, in multiple of PAC size. */
#define PRE_TIMEOUT 5
/* UWB microsecond (uus) to device time unit (dtu, around 15.65 ps) conversion factor.
 * 1 uus = 512 / 499.2 �s and 1 �s = 499.2 * 128 dtu. */
#define UUS_TO_DWT_TIME 63898

// ------------ Initiator specific ------------------ All timings may need tuning
/* Delay between frames, in UWB microseconds. */
/* This is the delay from the end of the frame transmission to the enable of the receiver, as programmed for the DW3000's wait for response feature. */
#define POLL_TX_TO_RESP_RX_DLY_UUS (290 + CPU_PROCESSING_TIME) 
/* This is the delay from Frame RX timestamp to TX reply timestamp used for calculating/setting the DW IC's delayed TX function.
 * This value is required to be larger than POLL_TX_TO_RESP_RX_DLY_UUS. */
#define RESP_RX_TO_FINAL_TX_DLY_UUS (480 + CPU_PROCESSING_TIME)
/* Time-stamps of frames transmission/reception, expressed in device time units. */
static uint64_t poll_tx_ts;
static uint64_t resp_rx_ts;
static uint64_t final_tx_ts;

/*
1. 682628146
2. 682864732
*/

void run_inititiator(void)
{
	dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
	dwt_setrxtimeout(RX_TIMEOUT_UUS);
	dwt_setpreambledetecttimeout(PRE_TIMEOUT);

	while (1) {
		printk("Staring Initiator ranging\n");
		// clear all events
		dwt_writesysstatuslo(0xFFFFFFFF);

		dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);
		printk("Waiting for system status\n");
		waitforsysstatus(&status_reg, NULL, (DWT_INT_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR), 0);

		/* check for RX good frame event */
		if (status_reg & DWT_INT_RXFCG_BIT_MASK) {
			printk("RX good frame recieved\n");
			//uint16_t frame_len;
            		int goodSts = 0;    /* Used for checking STS quality in received signal */
            		int16_t stsQual;    /* This will contain STS quality index */
            		uint16_t stsStatus; /* Used to check for good STS status (no errors). */

			/* Clear good RX frame event and TX frame sent in the DW3000 status register. */
            		dwt_writesysstatuslo(DWT_INT_RXFCG_BIT_MASK | DWT_INT_TXFRS_BIT_MASK);

			/* Check that the recieved frame has good STS quality */
			goodSts = dwt_readstsquality(&stsQual);
			if ((goodSts >= 0) && (dwt_readstsstatus(&stsStatus, 0) == DWT_SUCCESS)) {
				uint32_t final_tx_time;
                    		int ret;

				/* Retrieve poll transmission time and response reception time. */
				poll_tx_ts = get_tx_timestamp_u64();
                    		resp_rx_ts = get_rx_timestamp_u64();

				/* Compute final message transmission time. See NOTE 10 below. */
                    		final_tx_time = (resp_rx_ts + (RESP_RX_TO_FINAL_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;
				dwt_setdelayedtrxtime(final_tx_time);

				/* Final TX timestamp is the transmission time we programmed plus the TX antenna delay. */
				final_tx_ts = (((uint64_t)(final_tx_time & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY;


				ret = dwt_starttx(DWT_START_TX_DELAYED);
				/* If dwt_starttx() returns an error, abandon this ranging exchange and proceed to the next one. */

				if (ret == DWT_SUCCESS)	{
					/* Poll DW3000 until TX frame sent event set. */
					waitforsysstatus(NULL, NULL, DWT_INT_TXFRS_BIT_MASK, 0);

					/* Clear TXFRS event. */
					dwt_writesysstatuslo(DWT_INT_TXFRS_BIT_MASK);
				}
			} 
		} 
		else {
			/* Clear RX error/timeout events in the DW3000 status register. */
            		dwt_writesysstatuslo(SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
		}

		k_msleep(RNG_DELAY_MS);
	}
}

int main(void)
{
	printk("Entering Main\n");
	k_msleep(100);
	int ret;
	bool led_state = true;
	if (!gpio_is_ready_dt(&led)) {
		return 0;
	}	
	ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) {
		return 0;
	}


	int32_t driver_version = dwt_apiversion();
	printk("API Version: %x \n",driver_version);
	
	dw_reset(uwb);
	printk("Device Reset\n");
	
	int err = dwt_probe((struct dwt_probe_s *)&dw3000_probe_interf);
	printk("Probe return code: %d\n", err);
	
	/* Check device is in Idle_RC */
	while (!dwt_checkidlerc()){};

	if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR) {
		LOG_ERR("DWT Init Failed.");
		while (1) { };		// Better error management here
	}

	if (dwt_configure(&config)) {
		LOG_ERR("DWT Configuration Failed.");
		while (1) { };		// Better error management here
	}

	/* Tx spectrum parameters */
	dwt_configuretxrf(&txconfig_options_ch9);
	/* Antenna delay */
	dwt_setrxantennadelay(RX_ANT_DLY);
	dwt_settxantennadelay(TX_ANT_DLY);

	/* Enable TX/RX states output on GPIOs 5 and 6 to help debug */
	dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE);

	dwt_configurestskey(&sts_key);
	dwt_configurestsiv(&sts_iv);

	run_inititiator();

	LOG_ERR("Device not run");
	return 0;
}
