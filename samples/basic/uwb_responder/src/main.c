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


static dwt_sts_cp_key_t sts_key = { 0x14EB220F, 0xF86050A8, 0xD1D336AA, 0x14148674 };

static dwt_sts_cp_iv_t sts_iv = { 0x1F9A3DE4, 0xD37EC3CA, 0xC44FA8FB, 0x362EEB34 };


#define CPU_PROCESSING_TIME 500
/* Length of frame according with 64mhz PRF, STS Mode 3, 8 symbol SFD, 64 length SFD, 64 length Preamble */
#define FRAME_LENGTH_US 140
/* Inter-ranging delay period, in milliseconds. */
#define RNG_DELAY_MS 500
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

// ------------ Responder specific ------------------
/* Delay between frames, in UWB microseconds. See NOTE 4 below. */
/* This is the delay from Frame RX timestamp to TX reply timestamp used for calculating/setting the DW3000's delayed TX function. This includes the
 * frame length of approximately 180 us with above configuration. */
#define POLL_RX_TO_RESP_TX_DLY_UUS 900
/* This is the delay from the end of the frame transmission to the enable of the receiver, as programmed for the DW3000's wait for response feature. */
#define RESP_TX_TO_FINAL_RX_DLY_UUS 670
/* Timestamps of frames transmission/reception. */
static uint64_t poll_rx_ts;
static uint64_t resp_tx_ts;
static uint64_t final_rx_ts;

static uint8_t rx_count = 0;
static bool rx_event = false;
static bool rx_success = false;

/* Declaration of static functions. */
static void rx_ok_cb(const dwt_cb_data_t *cb_data);
static void rx_to_cb(const dwt_cb_data_t *cb_data);
static void rx_err_cb(const dwt_cb_data_t *cb_data);
static void tx_conf_cb(const dwt_cb_data_t *cb_data);

// void compute_resp_tx_frame_times(void)
// {
//     /*
//      * Different sized frames require different time delays.
//      */
//     uint32_t delay_time = POLL_RX_TO_RESP_TX_DLY_UUS + 0 + 0;

//     /* Length of the STS effects the size of the frame also.
//      * This means the delay required is greater for larger STS lengths. */
//     delay_time += ((1 << (config.stsLength + 2)) * 8);

//     dwt_setdelayedtrxtime((uint32_t)((delay_time * UUS_TO_DWT_TIME) >> 8));
// }

// void run_responder(void)
// {
// 	int16_t stsQual;    /* This will contain STS quality index and status */
// 	int goodSts = 0;    /* Used for checking STS quality in received signal */
// 	uint16_t stsStatus; /* Used to check for good STS status (no errors). */
// 	uint8_t messageFlag = 0; /* Used to track whether STS count should be reinitialised or not */

// 	printk("Staring Responder ranging\n");

//     	dwt_setrxaftertxdelay(RESP_TX_TO_FINAL_RX_DLY_UUS);
// 	dwt_configurestskey(&sts_key);

// 	while (1) {

// 		if (!messageFlag) {
// 			dwt_configurestsiv(&sts_iv);
// 			dwt_configurestsloadiv();
// 			dwt_rxenable(DWT_START_RX_IMMEDIATE);
// 		} else {
// 			//printk("Message 1 recieved\n");
// 		}

// 		waitforsysstatus(&status_reg, NULL, (DWT_INT_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR), 0);
// 		goodSts = dwt_readstsquality(&stsQual);

// 		// printk("%08x\n", status_reg);
// 		// k_msleep(2);

// 		if ((goodSts >= 0) && (dwt_readstsstatus(&stsStatus, 0) == DWT_SUCCESS)) {

// 			/* Recieved a good message. This is the first Rx event of he ranging transaction */
// 			if (!messageFlag) {
// 				uint32_t resp_tx_time;
// 				int ret;

// 				dwt_writesysstatuslo(DWT_INT_RXFCG_BIT_MASK);
// 				poll_rx_ts = get_rx_timestamp_u64();

// 				resp_tx_time = (poll_rx_ts                              /* Received timestamp value */
// 						+ ((POLL_RX_TO_RESP_TX_DLY_UUS          /* Set delay time */
// 						+ 0                			/* Added delay time for data rate set */
// 						+ 0					/* Added delay for TX preamble length */
// 						+ ((1 << (config.stsLength + 2)) * 8)) 	/* Added delay for STS length */
// 						* UUS_TO_DWT_TIME))
// 					>> 8;

// 				dwt_setdelayedtrxtime(resp_tx_time);
// 				resp_tx_ts = (((uint64_t)(resp_tx_time & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY;

// 				dwt_setrxaftertxdelay(100);
// 				ret = dwt_starttx(DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED);

// 				if (ret == DWT_SUCCESS) {
// 					/* Poll DW IC until TX frame sent event set. See NOTE 6 below. */
// 					waitforsysstatus(NULL, NULL, DWT_INT_TXFRS_BIT_MASK, 0);
// 					/* Clear TXFRS event. */
// 					dwt_writesysstatuslo(DWT_INT_TXFRS_BIT_MASK);

// 					/*
// 						* This flag is set high here so that we do not reset the STS count before receiving
// 						* the final message from the initiator. Otherwise, the STS count would be bad and
// 						* we would be unable to receive it.
// 						*/
// 					messageFlag = 1;
// 				}
// 			/* Recieved a good message. This is the last Rx of the ranging transaction */
// 			} else {
// 				dwt_writesysstatuslo(DWT_INT_RXFCG_BIT_MASK);
// 				/* Retrieve response transmission and final reception timestamps. */
// 				resp_tx_ts = get_tx_timestamp_u64();
// 				final_rx_ts = get_rx_timestamp_u64();

// 				/* Display computed distance on UART. */
// 				//printf("Ranging Successful\n");
// 				/* as DS-TWR initiator is waiting for RNG_DELAY_MS before next poll transmission
// 				* we can add a delay here before RX is re-enabled again
// 				*/

// 				k_msleep(RNG_DELAY_MS - 10); // start couple of ms earlier

// 				messageFlag = 0;
// 			}
// 		} else {
// 			dwt_writesysstatuslo(SYS_STATUS_ALL_RX_ERR);
// 			if (goodSts < 0) {
// 				//printk("Bad STS\n");
// 			} else {
// 				//printk("Bad sysStatus\n");
// 			}
// 			/* Recieved frame was of poor quality and unuseable */
// 			messageFlag = 0;
// 		}
// 	}
// }

void run_responder_irq(void)
{
	while (1) {
		if (!rx_count) {
			dwt_configurestsiv(&sts_iv);
			dwt_configurestsloadiv();
			dwt_setrxtimeout(0);
			dwt_rxenable(DWT_START_RX_IMMEDIATE);
		}

		// Wait for rx event
		while (!rx_event) {
			k_yield();
		}
		rx_event = false;

		// Delay if final rx sucessful
		if (rx_success) {
			rx_success = false;
			k_msleep(RNG_DELAY_MS - 2);
		}
	}
}

int main(void)
{
	printk("Entering Main\n");
	
	int ret;
	if (!gpio_is_ready_dt(&led)) {
		return 0;
	}	
	ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) {
		return 0;
	}


	// int32_t driver_version = dwt_apiversion();
	// printk("API Version: %x \n",driver_version);
	
	dw_reset(uwb);
	
	int err = dwt_probe((struct dwt_probe_s *)&dw3000_probe_interf);
	if (err < 0) {
		printk("Probe Error, code: %d\n", err);
	}
	
	/* Check device is in Idle_RC */
	while (!dwt_checkidlerc()){};

	if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR) {
		LOG_ERR("DWT Init Failed.");
		while (1) { };		// Better error management here
	}

	dwt_setleds(DWT_LEDS_ENABLE | DWT_LEDS_INIT_BLINK);

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

	dwt_setrxaftertxdelay(RESP_TX_TO_FINAL_RX_DLY_UUS);
	dwt_setrxtimeout(0);

	dwt_configurestskey(&sts_key);

	// run_responder();

	/* Register the call-backs (SPI CRC error callback is not used). */
   	dwt_setcallbacks(&tx_conf_cb, &rx_ok_cb, &rx_to_cb, &rx_err_cb, NULL, NULL, NULL);

    	/* Enable wanted interrupts (TX confirmation, RX good frames, RX timeouts and RX errors). */
    	dwt_setinterrupt(DWT_INT_TXFRS_BIT_MASK | DWT_INT_RXFCG_BIT_MASK | DWT_INT_RXFTO_BIT_MASK | DWT_INT_RXPTO_BIT_MASK | DWT_INT_RXPHE_BIT_MASK
                         | DWT_INT_RXFCE_BIT_MASK | DWT_INT_RXFSL_BIT_MASK | DWT_INT_RXSTO_BIT_MASK,
        		0, DWT_ENABLE_INT);

	dwt_writesysstatuslo(0xFFFFFFFF);
	dw_enable_irq(uwb);
	run_responder_irq();

	LOG_ERR("Device not run");
	return 0;
}


/*! ------------------------------------------------------------------------------------------------------------------
 * @fn rx_ok_cb()
 *
 * @brief Callback to process RX good frame events
 *
 * @param  cb_data  callback data
 *
 * @return  none
 */
static void rx_ok_cb(const dwt_cb_data_t *cb_data)
{
	(void)cb_data;

	uint32_t resp_tx_time;
	int goodSts = 0;
	int stsToast = 0;
	int16_t stsQual;
	uint16_t stsStatus;

	goodSts = dwt_readstsquality(&stsQual);
	stsToast = dwt_readstsstatus(&stsStatus, 0);

	if ((goodSts >= 0) && (stsToast == 0)) {

		if (!rx_count) {
			poll_rx_ts = get_rx_timestamp_u64();

			resp_tx_time = (poll_rx_ts + ((POLL_RX_TO_RESP_TX_DLY_UUS + 65) * UUS_TO_DWT_TIME)) >> 8;
			dwt_setdelayedtrxtime(resp_tx_time);

			resp_tx_ts = (((uint64_t)(resp_tx_time & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY;

			dwt_setrxaftertxdelay(100);//dwt_setrxaftertxdelay(RESP_TX_TO_FINAL_RX_DLY_UUS);
			//dwt_setrxtimeout(RX_TIMEOUT_UUS);
			//dwt_setpreambledetecttimeout(PRE_TIMEOUT);

			dwt_starttx(DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED);

			rx_count = 1;
		} else { /* second/final rx event instance*/
			// resp_tx_ts = get_tx_timestamp_u64();
			final_rx_ts = get_rx_timestamp_u64();
			rx_count = 0;
			rx_success = true;

			dwt_setrxtimeout(0);
			dwt_setpreambledetecttimeout(0);
		}

	} else {
		/* If any error on the sts, the ranging should be abandoned. reset rx_count to 0 */
		rx_count = 0;
	}
	rx_event = true;
	/* 
	 * This callback will be called when an initiator has sent either an initial poll message or the final tx 
	 * message. In the first case, the rx timestamp should be stored, the 
	 */
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn rx_to_cb()
 *
 * @brief Callback to process RX timeout events
 *
 * @param  cb_data  callback data
 *
 * @return  none
 */
static void rx_to_cb(const dwt_cb_data_t *cb_data)
{
    	(void)cb_data;
	rx_count = 0;
	rx_event = true;
	dwt_setrxtimeout(0);
	dwt_setpreambledetecttimeout(0);
    	/* 
     	 * Process when a response was expected but failed. For the responder, this occurs for the final rx listening
	 * event. If this occurs, the rx_count should be reset to zero
     	 */
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn rx_err_cb()
 *
 * @brief Callback to process RX error events
 *
 * @param  cb_data  callback data
 *
 * @return  none
 */
static void rx_err_cb(const dwt_cb_data_t *cb_data)
{
    	(void)cb_data;
	rx_count = 0;
	rx_event = true;
	dwt_setrxtimeout(0);
	dwt_setpreambledetecttimeout(0);
    	/* 
     	 * Process when a response was recieved but has errors. For the responder, this can happen at either the first 
	 * or the final rx event. In either case, the transaction can no longer be completed therefore rx_count reset
     	 */
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn tx_conf_cb()
 *
 * @brief Callback to process TX confirmation events
 *
 * @param  cb_data  callback data
 *
 * @return  none
 */
static void tx_conf_cb(const dwt_cb_data_t *cb_data)
{
    (void)cb_data;
    /*
     * No processing required.
     */

}
