/**
 * @file
 * @author Hayden Goodwin <hayden.goodwin@data61.csiro.au>
 * @copyright Commonwealth Scientific and Industrial Research Organisation, 2024
 */

#define DT_DRV_COMPAT qorvo_dw3xxx

#define LOG_MODULE_NAME ieee802154_dw3xxx
#define LOG_LEVEL CONFIG_IEEE802154_DRIVER_LOG_LEVEL
/* Includes ------------------------------------------------------------------*/

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/logging/log.h>
#include <zephyr/device.h>
#include <zephyr/pm/device.h>
#include <zephyr/pm/device_runtime.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/sys/util_macro.h>

#include <zephyr/drivers/ieee802154/ieee802154_dw3xxx.h>
#include <zephyr/drivers/ieee802154/deca_device_api.h>
#include <zephyr/drivers/ieee802154/deca_interface.h>
#include <zephyr/drivers/ieee802154/deca_types.h>
#include <zephyr/drivers/ieee802154/deca_version.h>

/* Private Defines -----------------------------------------------------------*/
#define MAX_DATA_BUF_LEN 200
#define RNG_DELAY_MS 500

#define DW3000_GET_RANGING_PHY_CONFIG(channel) (dwt_config_t) { 	\
	.chan = (channel), 						\
	.txPreambLength = DWT_PLEN_64,					\
	.rxPAC = DWT_PAC4, 						\
	.txCode = 9, 							\
	.rxCode = 9, 							\
	.sfdType = DWT_SFD_IEEE_4Z, 					\
	.dataRate = DWT_BR_6M8, 					\
	.phrMode = DWT_PHRMODE_STD, 					\
	.phrRate = DWT_PHRRATE_STD, 					\
	.sfdTO = (64 + 1 + 8 - 8), 					\
	.stsMode = DWT_STS_MODE_ND, 					\
	.stsLength = DWT_STS_LEN_64, 					\
	.pdoaMode = DWT_PDOA_M0, 					\
}

#define DW3000_GET_RANGING_TXPWR_CONFIG(channel) (dwt_txconfig_t) {		\
	.PGdly = 0x34,								\
        .power = ((channel) == HRP_UWB_PHY_CHANNEL_5) ? 0xfdfdfdfd : 0xfefefefe,\
        .PGcount = 0,								\
}

#define DW3000_RNG_IRQMASK_LO 	DWT_INT_TXFRS_BIT_MASK | 	\
				DWT_INT_RXFCG_BIT_MASK | 	\
				DWT_INT_RXFTO_BIT_MASK | 	\
				DWT_INT_RXPTO_BIT_MASK | 	\
				DWT_INT_RXPHE_BIT_MASK |	\
				DWT_INT_RXFCE_BIT_MASK | 	\
				DWT_INT_RXFSL_BIT_MASK | 	\
				DWT_INT_RXSTO_BIT_MASK
#define DW3000_RNG_IRQMASK_HI	0

/* Type Definitions ----------------------------------------------------------*/

/* Function Declarations -----------------------------------------------------*/
static void dw_isr_processing_thread(void *, void *, void *);

#define DW_ISR_THREAD_STACK_SIZE 	2048
#define DW_ISR_THREAD_PRIO		0
K_THREAD_DEFINE(dw_isr_thread_id, DW_ISR_THREAD_STACK_SIZE,
                dw_isr_processing_thread, NULL, NULL, NULL,
                DW_ISR_THREAD_PRIO, 0, CONFIG_SYS_CLOCK_TICKS_PER_SEC/5); // TODO: programatically create this thread in the init function
/* callbacks */
// static void initiator_ranging_rx_ok_cb(const dwt_cb_data_t *cb_data);
// static void initiator_ranging_rx_to_cb(const dwt_cb_data_t *cb_data);
// static void initiator_ranging_rx_err_cb(const dwt_cb_data_t *cb_data);
// static void initiator_ranging_tx_done_cb(const dwt_cb_data_t *cb_data);
// static void responder_ranging_rx_ok_cb(const dwt_cb_data_t *cb_data);
// static void responder_ranging_rx_to_cb(const dwt_cb_data_t *cb_data);
// static void responder_ranging_rx_err_cb(const dwt_cb_data_t *cb_data);
// static void responder_ranging_tx_done_cb(const dwt_cb_data_t *cb_data);

static uint8_t inter_tx_buf[MAX_DATA_BUF_LEN] = {0};
static uint8_t inter_rx_buf[MAX_DATA_BUF_LEN] = {0};

static dwt_sts_cp_key_t sts_key = { 0x14EB220F, 0xF86050A8, 0xD1D336AA, 0x14148674 };
static dwt_sts_cp_iv_t sts_iv = { 0x1F9A3DE4, 0xD37EC3CA, 0xC44FA8FB, 0x362EEB34 };

struct k_poll_signal dw_isr_sig;
struct k_poll_event dw_isr_event[1];

/* Private Variables ---------------------------------------------------------*/
static const struct device *uwb = DEVICE_DT_GET(DT_INST(0, qorvo_dw3xxx));

/*----------------------------------------------------------------------------*/
/* Qorvo API interface functions */
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

/*----------------------------------------------------------------------------*/
static void acquire_device(const struct device *dev)
{
	const struct dw3xxx_config *cfg = dev->config;

	/* Request bus to be on */
	pm_device_runtime_get(cfg->bus.bus);
}

/*----------------------------------------------------------------------------*/

static void release_device(const struct device *dev)
{
	const struct dw3xxx_config *cfg = dev->config;

	/* Release request for bus */
	pm_device_runtime_put(cfg->bus.bus);
}

/*----------------------------------------------------------------------------*/


/* 
    SPI function planning:
    Fast command: 8 bit command. Format bits 0 - 7 = {1, 0, cmd4, cmd3, cmd2, cmd1, cmd0, 1}
        Commands can be in the range of 0x0 to 0x13. eg command 0x1 packet  = {1,0,0,0,0,0,1,1}

    Short Addressed transaction: 8 bit header followed by n length written or read octets.
        Header format uses 5 bit base addr.  Bits 0 - 7 = {r/w, 0, BA4, BA3, BA2, BA1, BA0, 0}

    Full addressed transaction: 16 bit header followed by n length written or read octets.
        Header format has 5 bit base address and a 7 bit subaddress followed by two mode bits
        bits 0-15 = {r/w, 0, BA4, BA3, BA2, BA1, BA0, SA6, SA5, SA4, SA3, SA2, SA1, SA0, 0, 0}

    Masked write transaction: 16 bit header followed by 2, 4, or 8 bit bitmask. Bitmasks are 
        comprised of an AND mask and OR mask concatenated. AND and OR masks are 1, 2, and 4 octets.
        Header format has 5 bit base address and a 7 bit subaddress followed by two mode bits
        bits 0-15 = {r/w, 0, BA4, BA3, BA2, BA1, BA0, SA6, SA5, SA4, SA3, SA2, SA1, SA0, M1, M0}
        Mode {M1,M0} :  {0,1} = 1 octet AND and OR masks
                        {1,0} = 2 octet AND and OR masks
                        {1,1} = 4 octet AND and OR masks
*/
int dw_spi_read(const struct device *dev, uint16_t hdr_len, const uint8_t *hdr_buf, uint32_t data_len, uint8_t *data)
{
	struct spi_buf rx_buf[2];
	struct spi_buf tx_buf;
	struct spi_buf_set rx = {.buffers = rx_buf, .count = 1};
	struct spi_buf_set tx = {.buffers = &tx_buf, .count = 1};
	const struct dw3xxx_config *cfg = dev->config;
	uint32_t transaction_len = hdr_len + data_len;
	uint8_t *p = 0;
	int rc;

	if (transaction_len > MAX_DATA_BUF_LEN) {
		LOG_WRN("SPI transaction exceeds max length of %u", MAX_DATA_BUF_LEN);

		tx_buf.buf = (uint8_t*)hdr_buf;
		tx_buf.len = hdr_len;

		rx_buf[0].buf = NULL;
		rx_buf[0].len = hdr_len;
		rx_buf[1].buf = (uint8_t *)data;
		rx_buf[1].len = data_len;

		rx.count = 2;
	} else {
		p = inter_tx_buf;
		memcpy(p, hdr_buf, hdr_len);
		p += hdr_len;
		memset(p, 0x00, data_len);

		tx_buf.buf = (uint8_t*)inter_tx_buf;
		tx_buf.len = transaction_len;

		rx_buf[0].buf = (uint8_t *)inter_rx_buf;
		rx_buf[0].len = transaction_len;
	}

	LOG_DBG("spi read, header length %u, data length %u", (uint16_t)hdr_len, (uint32_t)data_len);
	LOG_HEXDUMP_DBG(hdr_buf, (uint16_t)hdr_len, "rd: header");

	// start = ARM_CM_DWT_CYCCNT;
	acquire_device(dev);
	// stop = ARM_CM_DWT_CYCCNT;

	// printk("acquire time: %u", stop - start);

	rc = spi_transceive_dt(&cfg->bus, &tx, &rx);
	if (rc < 0) {
		LOG_ERR("SPI transfer failed");
		goto release_read;
	}

	/* copy data from intermadiate rx buffer if pointer p has been used. */
	if (p) {
		memcpy(data, inter_rx_buf + hdr_len, data_len);
	}

	LOG_HEXDUMP_DBG(data, (uint32_t)data_len, "rd: data");
release_read:
	release_device(dev);
	return rc;
}

/*----------------------------------------------------------------------------*/

int dw_spi_write(const struct device *dev, uint16_t hdr_len, const uint8_t *hdr_buf, uint32_t data_len, const uint8_t *data)
{
	struct spi_buf tx_buf[2];
	struct spi_buf_set buf_set = {.buffers = tx_buf, .count = 1};
	int rc;
	const struct dw3xxx_config *cfg = dev->config;
	uint32_t tx_len = hdr_len + data_len;
	uint8_t *p;
	

	if (tx_len > MAX_DATA_BUF_LEN) {
		LOG_WRN("SPI transaction exceeds max length of %u", MAX_DATA_BUF_LEN);

		tx_buf[0].buf = (uint8_t *)hdr_buf;
		tx_buf[0].len = hdr_len;
		tx_buf[1].buf = (uint8_t *)data;
		tx_buf[1].len = data_len;

		buf_set.count = 2;
	} else {

		p = inter_tx_buf;
		memcpy(p, hdr_buf, hdr_len);
		p += hdr_len;
		memcpy(p, data, data_len);

		tx_buf[0].buf = (uint8_t*)inter_tx_buf;
		tx_buf[0].len = tx_len;
	}

	LOG_DBG("spi write, header length %u, data length %u", (uint16_t)hdr_len, (uint32_t)data_len);
	LOG_HEXDUMP_DBG(hdr_buf, (uint16_t)hdr_len, "wr: header");
	LOG_HEXDUMP_DBG(data, (uint32_t)data_len, "wr: data");

	acquire_device(dev);
	rc = spi_write_dt(&cfg->bus, &buf_set);
	if (rc < 0) {
		LOG_ERR("SPI write failed");
		goto release_write;
	}

release_write:
	release_device(dev);
	return rc;
}

// /*----------------------------------------------------------------------------*/

// static int dw_spi_fast_command(const struct device *dev, dw_fast_command_packet_e command)
// {
//         uint8_t buf = command;
        
//         /* Fast command consists of an 8 bit packet with no additional data. */
//         return dw_spi_write(dev, 1, &buf, 0, NULL);
// }

// /*----------------------------------------------------------------------------*/

// static int dw_short_addr_read(const struct device *dev, dw_base_addr_e base, uint32_t data_len,
// 		       	uint8_t *data)
// {
//         uint8_t buf = base;

//         return dw_spi_read(dev, 1, &buf, data_len, data);
// }

// /*----------------------------------------------------------------------------*/

// static int dw_short_addr_write(const struct device *dev, dw_base_addr_e base, uint32_t data_len,
// 		       	uint8_t *data)
// {
//         uint8_t buf = 0x80 | base;

//         return dw_spi_write(dev, 1, &buf, data_len, data);
// }

// /*----------------------------------------------------------------------------*/

// static int dw_full_addr_read(const struct device *dev, dw_full_addr_e addr, uint32_t data_len, 
// 			uint8_t *data)
// {
// 	uint8_t header[2] = {(addr >> 8), (addr & 0xFF)};
// 	return dw_spi_read(dev, sizeof(header), header, data_len, data);
// }

// /*----------------------------------------------------------------------------*/

// static int dw_full_addr_write(const struct device *dev, dw_full_addr_e addr, uint32_t data_len, 
// 			uint8_t *data)
// {
// 	uint8_t header[2] = {(addr >> 8) | 0x80, (addr & 0xFF)};
// 	return dw_spi_write(dev, sizeof(header), header, data_len, data);
// }

// /*----------------------------------------------------------------------------*/

// static int dw_full_addr_masked_write(const struct device *dev, dw_full_addr_e addr, uint32_t data_len, 
// 			uint8_t *data)
// {
// 	uint8_t header[2] = {(addr >> 8) | 0x80, (addr & 0xFF)};

// 	switch (data_len)
// 	{
// 	/*
// 	 * Data length of 2 indicates mode 1. 1 octal AND and OR masks
// 	 * Data length of 4 indicates mode 2. 2 octal AND and OR masks
// 	 * Data length of 8 indicates mode 3. 4 octal And and OR masks
// 	 * Data length other than these values is invalid.
// 	 */
// 	case BIT(1):
// 	case BIT(2):
// 		header[1] |= data_len >> 1;
// 		break;
// 	case BIT(3):
// 		header[1] |= 0x03;
// 		break;
// 	default:
// 		LOG_ERR("Invalid Masked Write Length");
// 		return -EINVAL;
// 		break;
// 	}

// 	return dw_spi_write(dev, sizeof(header), header, data_len, data);
// }

// /*----------------------------------------------------------------------------*/

void dw_wakeup(const struct device *dev)
{
	const struct dw3xxx_config *cfg = dev->config;

	if (gpio_is_ready_dt(&cfg->wkp_gpio)) {
		LOG_INF("GPIO_WAKEUP");
		gpio_pin_set_dt(&cfg->wkp_gpio, GPIO_OUTPUT_ACTIVE);
		k_sleep(K_USEC(200));
		gpio_pin_set_dt(&cfg->wkp_gpio, GPIO_OUTPUT_INACTIVE);
	} else {
		LOG_INF("SPI Wakeup");
		gpio_pin_set_dt(&cfg->bus.config.cs.gpio, GPIO_OUTPUT_ACTIVE);
		k_sleep(K_USEC(200));
		gpio_pin_set_dt(&cfg->bus.config.cs.gpio, GPIO_OUTPUT_INACTIVE);
	}
}

/*----------------------------------------------------------------------------*/

void dw_reset(const struct device *dev)
{
	const struct dw3xxx_config *cfg = dev->config;

	gpio_pin_configure_dt(&cfg->rst_gpio, GPIO_OUTPUT_ACTIVE);
	k_msleep(2);
	gpio_pin_configure_dt(&cfg->rst_gpio, GPIO_INPUT);
	k_msleep(2);
}


/*----------------------------------------------------------------------------*/
/* TODO: make the key and iv a part of the device data.*/
/* TODO: Add a check to make sure that the device is in a correct state for setting the key and IV 
  When the device is in some sleep modes, spi communications will not work. */
void dw3xxx_set_sts_key(const struct device *dev, uint32_t *key)
{
	memcpy(&sts_key, key, sizeof(sts_key));
	dwt_configurestskey(&sts_key);
}

void dw3xxx_set_sts_iv(const struct device *dev, uint32_t *iv)
{
	memcpy(&sts_iv, iv, sizeof(sts_iv));
}

void dw3xxx_set_sts_counter(const struct device *dev, uint32_t counter)
{
	sts_iv.iv3 = counter;
}

void dw3xxx_get_timestamp(const struct device *dev, uint64_t *ts)
{
	struct dw3xxx_data *data  = dev->data;
	memcpy(ts, &data->timestamps, 3 * sizeof(uint64_t));
}

uint32_t dw3xxx_get_sts_counter(const struct device *dev)
{
	return sts_iv.iv3;
}

void dw3xxx_get_sts_iv(const struct device *dev, uint32_t *iv)
{
	/* TODO: Should have a sts struct instead of uint32_t pointer */
	memcpy(iv, &sts_iv, sizeof(sts_iv));
}

void dw3xxx_get_sts_key(const struct device *dev, uint32_t *key)
{
	/* TODO: Should have a sts struct instead of uint32_t pointer */
	memcpy(key, &sts_key, sizeof(sts_key));
}

/*----------------------------------------------------------------------------*/
/* Initiator in ranging mode functions */
int run_initiator(const struct device* dev)
{
	struct dw3xxx_data *data  = dev->data;
	int signaled, result;
	int ret;

	// Reload counter and send a poll
	dwt_configurestsiv(&sts_iv);
	dwt_configurestsloadiv();
	dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);

	// Hold until rx event or timeout
	k_poll(data->rng_event, 1, K_MSEC(1));
	k_poll_signal_check(&data->rx_sig, &signaled, &result);

	if (signaled && (result == RX_EVENT_OK)) {
		/* Add processing here for processing rx OK event. In this situation the ranging transaction 
		completed successfully and timestamps can be packaged and shipped off to whoever wants them */

		// LOG_INF("%llu %llu %llu", data->timestamps.ts1, data->timestamps.ts2, data->timestamps.ts3);
		ret = 0;

	} else if (signaled && (result < 0)) {
		LOG_ERR("Rx error: %d", result);
		ret = result;
	} else {
		LOG_ERR("K_Poll timeout");
		ret = ERROR_POLL_TIMEOUT;
	}

	/* returning negative number for the moment. Should be more specific in future. */
	k_poll_signal_reset(&data->rx_sig);
	data->rng_event[0].state = K_POLL_STATE_NOT_READY;
	return ret;
}

void run_initiator_forever(const struct device* dev) 
{
	while (1) {
		/* start the initiator */
		run_initiator(dev);
		/* Sleep for ranging delay period */
		k_msleep(RNG_DELAY_MS);
	}
}

static void initiator_ranging_rx_ok_cb(const dwt_cb_data_t *cb_data)
{
	(void)cb_data;
	k_poll_signal_raise(&dw_isr_sig, INITIATOR_RXOK);
}
static void initiator_ranging_rx_ok(void)
{
	struct dw3xxx_data *data  = uwb->data;

	uint32_t final_tx_time;
	int goodSts = 0;
	int stsToast = 0;
	int16_t stsQual;
	uint16_t stsStatus;

	goodSts = dwt_readstsquality(&stsQual);
	stsToast = dwt_readstsstatus(&stsStatus, 0);

	if ((goodSts >= 0) && (stsToast == 0)) {
		data->timestamps.ts1 = get_tx_timestamp_u64();
		data->timestamps.ts2 = get_rx_timestamp_u64();

		// final_tx_time = (resp_rx_ts + (RESP_RX_TO_FINAL_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;
		final_tx_time = (data->timestamps.ts2 >> 8) + INITIATOR_DETERMINISTIC_DELAY;
		dwt_setdelayedtrxtime(final_tx_time);

		dwt_starttx(DWT_START_TX_DELAYED);

		data->timestamps.ts3 = (((uint64_t)(final_tx_time & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY;

		k_poll_signal_raise(&data->rx_sig,RX_EVENT_OK);

	} else {
		k_poll_signal_raise(&data->rx_sig,RX_EVENT_ERROR);
	}
}

static void initiator_ranging_rx_to_cb(const dwt_cb_data_t *cb_data)
{
    	(void)cb_data;
	k_poll_signal_raise(&dw_isr_sig,INITIATOR_RXTO);
}
static void initiator_ranging_rx_to(void)
{
	struct dw3xxx_data *data  = uwb->data;
	k_poll_signal_raise(&data->rx_sig,RX_EVENT_ERROR);
}

static void initiator_ranging_rx_err_cb(const dwt_cb_data_t *cb_data)
{
    	(void)cb_data;
	k_poll_signal_raise(&dw_isr_sig,INITIATOR_RXERR);
}
static void initiator_ranging_rx_err(void)
{
	struct dw3xxx_data *data  = uwb->data;
	k_poll_signal_raise(&data->rx_sig,RX_EVENT_ERROR);
}

static void initiator_ranging_tx_done_cb(const dwt_cb_data_t *cb_data)
{
	(void)cb_data;
	k_poll_signal_raise(&dw_isr_sig, INITIATOR_TXDONE);
}
static void initiator_ranging_tx_done(void)
{
	return;
}

/* Responder in ranging mode functions */
/* TODO: add a timeout parameter to the function.*/

int run_responder(const struct device* dev, k_timeout_t timeout) 
{
	struct dw3xxx_data *data  = dev->data;
	int signaled, result;
	int ret;

	data->rx_count = 0;

	/* Clear any unexpected signals */
	k_poll_signal_reset(&data->rx_sig);
	data->rng_event[0].state = K_POLL_STATE_NOT_READY;

	/* Configure STS and start RX listeneing */
	dwt_configurestsiv(&sts_iv);
	dwt_configurestsloadiv();
	dwt_setrxtimeout(0);
	dwt_rxenable(DWT_START_RX_IMMEDIATE);

	/* Wait for rx event for timeout (in ticks) */
	ret = k_poll(data->rng_event, 1, timeout);
	k_poll_signal_check(&data->rx_sig, &signaled, &result);
	k_poll_signal_reset(&data->rx_sig);
	data->rng_event[0].state = K_POLL_STATE_NOT_READY;
	// LOG_INF("1: %llu", k_uptime_ticks());

	if (ret) {
		LOG_ERR("Ranging Recieve Timeout %d", ret);
		return ret;
	}

	if (signaled && (result == RX_EVENT_OK)) {
		/* This means that the responding tx event have been scheduled and the rx timeout has been
			set. At this point, wait for the final rx event. */
		
		k_poll(data->rng_event, 1, K_MSEC(4));
		k_poll_signal_check(&data->rx_sig, &signaled, &result);
		// LOG_INF("2: %llu", k_uptime_ticks());

		// k_poll(dw_isr_event, 1, K_MSEC(2));
		// k_poll_signal_check(&dw_isr_sig, &signaled, &result);
		
		if (signaled && (result == RX_EVENT_OK)) {
			// LOG_INF("%llu %llu %llu", data->timestamps.ts1, data->timestamps.ts2, data->timestamps.ts3);
			ret = 0;
		} else if (signaled && (result < 0)){
			LOG_ERR("2nd Rx Error: %d", result);
			ret = result;
		} else {
			LOG_ERR("K_Poll timeout");
			ret = ERROR_POLL_TIMEOUT;
		}

	} else if (signaled && (result < 0)) {
		LOG_ERR("1st Rx Error: %d", result);
		ret = result;
	} else {
		/* Either the signal has not been signalled or an unknown value has been returned by the signal. */
		LOG_ERR("Unknown Error");
		ret = UNKNOWN_ERROR;
	}


	k_poll_signal_reset(&data->rx_sig);
	data->rng_event[0].state = K_POLL_STATE_NOT_READY;
	// k_poll_signal_reset(&dw_isr_sig);
	// dw_isr_event[0].state = K_POLL_STATE_NOT_READY;
	return ret;
}

void run_responder_forever(const struct device* dev) 
{
	while (1) {
		run_responder(dev, K_FOREVER);
		/* Responder is now synced with transmitter, delay for inter ranging delay minus a short period
		 * to wake up and restart the reciever. */
		k_msleep(RNG_DELAY_MS - 2);
	}
}

static void responder_ranging_rx_ok_cb(const dwt_cb_data_t *cb_data)
{
	(void)cb_data;
	k_poll_signal_raise(&dw_isr_sig, RESPONDER_RXOK);
	// LOG_INF("RXOK: %llu", k_uptime_ticks());
}
static void responder_ranging_rx_ok(void)
{
	struct dw3xxx_data *data  = uwb->data;

	uint32_t resp_tx_time;
	int goodSts = 0;
	int stsToast = 0;
	int16_t stsQual;
	uint16_t stsStatus;

	goodSts = dwt_readstsquality(&stsQual);
	stsToast = dwt_readstsstatus(&stsStatus, 0);

	if ((goodSts >= 0) && (stsToast == 0)) {

		if (!data->rx_count) {
			data->timestamps.ts1 = get_rx_timestamp_u64();

			resp_tx_time = (data->timestamps.ts1 >> 8) + RESPONDER_DETERMINISTIC_DELAY;
			dwt_setdelayedtrxtime(resp_tx_time);

			dwt_starttx(DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED);

			// dwt_setrxtimeout(RX_TIMEOUT);
			data->timestamps.ts2 = (((uint64_t)(resp_tx_time & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY;

			data->rx_count = 1;
			k_poll_signal_raise(&data->rx_sig,RX_EVENT_OK);
			// LOG_INF("1st Poll: %llu", k_uptime_ticks());

		} else { /* second/final rx event instance*/
			// resp_tx_ts = get_tx_timestamp_u64();
			data->timestamps.ts3 = get_rx_timestamp_u64();
			data->rx_count = 0;
			k_poll_signal_raise(&data->rx_sig,RX_EVENT_OK);
			// k_poll_signal_raise(&dw_isr_sig, RX_EVENT_OK);
			// LOG_INF("2nd Polled: %llu", k_uptime_ticks());
		}

	} else {
		k_poll_signal_raise(&data->rx_sig,RX_EVENT_ERROR);
		data->rx_count = 0;
	}
}

static void responder_ranging_rx_to_cb(const dwt_cb_data_t *cb_data)
{
    	(void)cb_data;
	k_poll_signal_raise(&dw_isr_sig, RESPONDER_RXTO);
	// LOG_INF("RXTO: %llu", k_uptime_ticks());
}
static void responder_ranging_rx_to(void)
{
	struct dw3xxx_data *data  = uwb->data;
	data->rx_count = 0;
	k_poll_signal_raise(&data->rx_sig,RX_EVENT_ERROR_TO);
}

static void responder_ranging_rx_err_cb(const dwt_cb_data_t *cb_data)
{
    	(void)cb_data;
	k_poll_signal_raise(&dw_isr_sig, RESPONDER_RXERR);
	// LOG_INF("RXERR: %llu", k_uptime_ticks());
}
static void responder_ranging_rx_err(void)
{
	struct dw3xxx_data *data  = uwb->data;
	data->rx_count = 0;
	k_poll_signal_raise(&data->rx_sig,RX_EVENT_ERROR);
}

static void responder_ranging_tx_done_cb(const dwt_cb_data_t *cb_data)
{
	(void)cb_data;
	k_poll_signal_raise(&dw_isr_sig, RESPONDER_TXDONE);
	// LOG_INF("TODN: %llu", k_uptime_ticks());
}
static void responder_ranging_tx_done(void)
{
	return;
}

static void dw_isr_processing_thread(void *, void *, void *)
{
	int signaled, result;
	int ret;
	while (1) {
		ret = k_poll(dw_isr_event, 1, K_FOREVER);
		// LOG_INF("Sig: %llu", k_uptime_ticks());
		if (ret) {
			/* process the error */
		}

		/* Get the outcome of the signal */
		k_poll_signal_check(&dw_isr_sig, &signaled, &result);
		k_poll_signal_reset(&dw_isr_sig);
		dw_isr_event[0].state = K_POLL_STATE_NOT_READY;

		switch (result) {
		case INITIATOR_TXDONE:
			initiator_ranging_tx_done();
			break;
		case INITIATOR_RXOK:
			initiator_ranging_rx_ok();
			break;
		case INITIATOR_RXTO:
			initiator_ranging_rx_to();
			break;
		case INITIATOR_RXERR:
			initiator_ranging_rx_err();
			break;
		case INITIATOR_SPIERR:
			break;
		case INITIATOR_SPIRDY:
			break;
		case RESPONDER_TXDONE:
			responder_ranging_tx_done();
			break;
		case RESPONDER_RXOK:
			responder_ranging_rx_ok();
			break;
		case RESPONDER_RXTO:
			responder_ranging_rx_to();
			break;
		case RESPONDER_RXERR:
			responder_ranging_rx_err();
			break;
		case RESPONDER_SPIERR:
			break;
		case RESPONDER_SPIRDY:
			break;
		}
	}
}
/*----------------------------------------------------------------------------*/

static void dw3xxx_hw_isr(const struct device* dev, struct gpio_callback* cb, uint32_t pins)
{
	// LOG_INF("isr: %llu", k_uptime_ticks());
	dwt_isr();
}

void dw_enable_irq(const struct device* dev)
{
	const struct dw3xxx_config *cfg = dev->config;
	gpio_pin_interrupt_configure_dt(&cfg->irq_gpio, GPIO_INT_EDGE_RISING);
}

void dw_disable_irq(const struct device* dev)
{
	const struct dw3xxx_config *cfg = dev->config;
	gpio_pin_interrupt_configure_dt(&cfg->irq_gpio, GPIO_INT_DISABLE);
}

static inline void dw3xxx_update_callbacks(const struct device* dev)
{
	struct dw3xxx_data *data  = dev->data;
	struct dw_isr_callbacks *cbs = &data->cbs;
	dwt_setcallbacks(cbs->cbTxDone, cbs->cbRxOk, cbs->cbRxTo, cbs->cbRxErr, cbs->cbSPIErr, cbs->cbSPIRdy, NULL);
}

static inline void dw3xxx_update_interrupts(const struct device* dev)
{
	struct dw3xxx_data *data  = dev->data;
	dwt_setinterrupt(data->irq_mask_lo, data->irq_mask_hi, DWT_ENABLE_INT_ONLY);//DWT_ENABLE_INT);
}


static int dw3xxx_set_device_ranging(const struct device* dev)
{
	struct dw3xxx_data *data  = dev->data;
        dwt_config_t *phy_cfg = &data->phy_cfg;
        dwt_txconfig_t *tx_pwr_cfg = &data->tx_pwr_cfg;
	while (!dwt_checkidlerc()){};

	// should check here IDLE RC
	if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR) {
		LOG_ERR("DWT Init Failed.");
		return -EIO;		// Check that the error code is most appropriate
	}

	// Set up including LEDs
	dwt_setleds(DWT_LEDS_ENABLE | DWT_LEDS_INIT_BLINK);

	if (dwt_configure(phy_cfg)) {
		LOG_ERR("DWT Configuration Failed.");
		return -EIO;		// Check that the error code is most appropriate
	}

	// Ensure that the tx power config is being appropriately set based on the channel in phy config
	dwt_configuretxrf(tx_pwr_cfg);

	dwt_setrxantennadelay(RX_ANT_DLY);
	dwt_settxantennadelay(TX_ANT_DLY);

	// Stuff
	dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE);

	// Set the appropriate delays
	if (data->role == RANGING_INITIATOR) {
		dwt_setrxaftertxdelay(INITIATOR_RX_WAKEUP_DELAY);
		dwt_setrxtimeout(RX_TIMEOUT);
		// dwt_setpreambledetecttimeout(PREAMBLE_TIMEOUT);
		
	} else if ((data->role == RANGING_RESPONDER)) {
		dwt_setrxaftertxdelay(RESPONDER_RX_WAKEUP_DELAY);
		// dwt_setrxtimeout(0);
		// dwt_setpreambledetecttimeout(PREAMBLE_TIMEOUT);
	}

	dwt_configurestskey(&sts_key);
	dwt_configurestsiv(&sts_iv);
	dwt_configurestsloadiv();

	dw3xxx_update_callbacks(dev);
	dw3xxx_update_interrupts(dev);

	// dwt_writesysstatuslo(0xFFFFFFFF);
	return 0;
}

int dw3xxx_configure_device(const struct device* dev, dw_configrole_e role, uint8_t channel)
{
	struct dw3xxx_data *data  = dev->data;

	if ((channel != HRP_UWB_PHY_CHANNEL_5) && (channel != HRP_UWB_PHY_CHANNEL_9)) {
		LOG_ERR("Invalid UWB channel %u", channel);
		return -EINVAL;
	}
	
	switch (role) {
		case RANGING_INITIATOR:
			data->role = role;
			data->phy_cfg = DW3000_GET_RANGING_PHY_CONFIG(channel);
			data->tx_pwr_cfg = DW3000_GET_RANGING_TXPWR_CONFIG(channel);
			data->irq_mask_hi = DW3000_RNG_IRQMASK_HI;
			data->irq_mask_lo = DW3000_RNG_IRQMASK_LO;
			
			data->cbs.cbRxOk = initiator_ranging_rx_ok_cb;
			data->cbs.cbRxTo = initiator_ranging_rx_to_cb;
			data->cbs.cbRxErr = initiator_ranging_rx_err_cb;
			data->cbs.cbTxDone = initiator_ranging_tx_done_cb;
			data->cbs.cbSPIErr = NULL;
			data->cbs.cbSPIRdy = NULL;
			dw3xxx_set_device_ranging(dev);
			break;
		case RANGING_RESPONDER:
			data->role = role;
			data->phy_cfg = DW3000_GET_RANGING_PHY_CONFIG(channel);
			data->tx_pwr_cfg = DW3000_GET_RANGING_TXPWR_CONFIG(channel);
			data->irq_mask_hi = DW3000_RNG_IRQMASK_HI;
			data->irq_mask_lo = DW3000_RNG_IRQMASK_LO;
			
			data->cbs.cbRxOk = responder_ranging_rx_ok_cb;
			data->cbs.cbRxTo = responder_ranging_rx_to_cb;
			data->cbs.cbRxErr = responder_ranging_rx_err_cb;
			data->cbs.cbTxDone = responder_ranging_tx_done_cb;
			data->cbs.cbSPIErr = NULL;
			data->cbs.cbSPIRdy = NULL;
			dw3xxx_set_device_ranging(dev);
			break;
		case TIME_SYNC_CONTROLER:
			break;
		case TIME_SYNC_LISTNER:
			break;
		default:
			LOG_ERR("Invalid role %u", role);
			return -EINVAL;
	}
	return 0;
}


/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/

static int dw3xxx_probe(const struct device *dev)
{
	int rc;

	dw_reset(uwb);
	rc = dwt_probe((struct dwt_probe_s *)&dw3000_probe_interf);
	if (rc < 0) {
		printk("Probe Error, code: %d\n", rc);
	}

	return rc;
}

/*----------------------------------------------------------------------------*/

static int dw3xxx_pm_control(const struct device *dev, enum pm_device_action action)
{
	// const struct dw3xxx_config *cfg = dev->config;
	int rc = 0;

	switch (action) {
	case PM_DEVICE_ACTION_SUSPEND:
	case PM_DEVICE_ACTION_RESUME:
	case PM_DEVICE_ACTION_TURN_ON:
        /* Query device ID */
		dw3xxx_probe(dev);
		break;
	case PM_DEVICE_ACTION_TURN_OFF:
		break;
	default:
		rc = -ENOSYS;
	}

	return rc;
}

/*----------------------------------------------------------------------------*/

int dw3xxx_init(const struct device *dev)
{
	const struct dw3xxx_config *cfg = dev->config;
	struct dw3xxx_data *data = dev->data;
    	int rc = 0;

	printk("DW3xxx init\n");

	/* Setup synchronisation signals */
	k_poll_signal_init(&data->rx_sig);
	k_poll_event_init(&data->rng_event[0], K_POLL_TYPE_SIGNAL, K_POLL_MODE_NOTIFY_ONLY, &data->rx_sig);
	k_poll_signal_init(&dw_isr_sig);
	k_poll_event_init(&dw_isr_event[0], K_POLL_TYPE_SIGNAL, K_POLL_MODE_NOTIFY_ONLY, &dw_isr_sig);

	/* Critical Devices */
	if (!spi_is_ready_dt(&cfg->bus)) {
		LOG_ERR("SPI bus not ready %s", cfg->bus.bus->name);
		return -ENODEV;
	}

	/* Setup the irq interrupt, to be enabled later */
	if (device_is_ready(cfg->irq_gpio.port)) {
		gpio_init_callback(&data->irq_callback, dw3xxx_hw_isr, BIT(cfg->irq_gpio.pin));
		rc = gpio_add_callback(cfg->irq_gpio.port, &data->irq_callback);
		if (rc < 0) {
			LOG_ERR("Failed to initialise data ready callback!");
			return rc;
		}
		gpio_pin_interrupt_configure_dt(&cfg->irq_gpio, GPIO_INT_DISABLE);
	} else {
		LOG_ERR("IRQ GPIO device is not ready");
		return -ENODEV;
	}

	/* Reset Pin */
	if (gpio_is_ready_dt(&cfg->rst_gpio)) {
		rc = gpio_pin_configure_dt(&cfg->rst_gpio, GPIO_INPUT);
		if (rc < 0) {
			LOG_ERR("Failed to configure reset gpio!");
			return rc;
		}
	} else {
		LOG_ERR("Reset GPIO device is not ready");
		return -ENODEV;
	}

	/* Not so critical Devices */
	if (gpio_is_ready_dt(&cfg->wkp_gpio)) {
		rc = gpio_pin_configure_dt(&cfg->wkp_gpio, GPIO_OUTPUT_ACTIVE);
		if (rc < 0) {
			LOG_ERR("Failed to configure Wakeup gpio!");
		}
	} else {
		LOG_ERR("Wakeup GPIO device is not ready");
	}
	
	return pm_device_driver_init(dev, dw3xxx_pm_control);
}

void set_tx_power(int16_t dbm) {
	/* Qorvo devices have a maximum power spectral density of -31 dbm/MHz. Thus at full beans and at the maximum 
	bandwidth of 499.2 MHz, the peak output power is 0.3965 mW or -4.0173 dBm. With the plots given in section 
	3.11.2 of the datasheet, a lookup table can be constructed. Things get more complicated when you start considering
	the regulations where the maximum allowed average PSD is -41.3dBm/MHz when averaged over 1ms. Thus the frame
	length, and the delay between frames must be considered to ensure regulations are met. Realistically in a ranging
	scenario, your frame will only be sent roughly once in a millisecond.
	(Note: the bandwidth can be tuned through PG_DELAY). */
}

// static const struct ieee802154_radio_api dwt_radio_api = {
// 	.iface_api.init		= dwt_iface_api_init,

// 	.get_capabilities	= dwt_get_capabilities,
// 	.cca			= dwt_cca,
// 	.set_channel		= dwt_set_channel,
// 	.filter			= dwt_filter,
// 	.set_txpower		= dwt_set_power,
// 	.start			= dwt_start,
// 	.stop			= dwt_stop,
// 	.configure		= dwt_configure,
// 	.ed_scan		= dwt_ed,
// 	.tx			= dwt_tx,
// 	.attr_get		= dwt_attr_get,
// };

/*----------------------------------------------------------------------------*/

#define DW3XXX_SPI_OP_MODE SPI_OP_MODE_MASTER | SPI_WORD_SET(8) | SPI_TRANSFER_MSB

#define DW3000_GET_TIME_SYNC_PHY_CONFIG(channel)

#define DW3XXX_INIT(n)                                                                      \
	struct dw3xxx_data dw3xxx_data_##n;                                                 \
	static const struct dw3xxx_config dw3xxx_config_##n = {                             \
		.bus = SPI_DT_SPEC_INST_GET(n, DW3XXX_SPI_OP_MODE, 0),                      \
		.irq_gpio = GPIO_DT_SPEC_INST_GET(n, irq_gpios),                            \
		.wkp_gpio = GPIO_DT_SPEC_INST_GET_OR(n, wakeup_gpios, {0}),				\
		.rst_gpio = GPIO_DT_SPEC_INST_GET(n, reset_gpios),				\
	};                                                                                  \
	PM_DEVICE_DT_INST_DEFINE(n, dw3xxx_pm_control);                                     \
	DEVICE_DT_INST_DEFINE(n, dw3xxx_init, PM_DEVICE_DT_INST_GET(n), &dw3xxx_data_##n,   \
			      &dw3xxx_config_##n, POST_KERNEL, CONFIG_IEEE802154_DW3XXX_INIT_PRIO, \
			      NULL);

DT_INST_FOREACH_STATUS_OKAY(DW3XXX_INIT)

/*----------------------------------------------------------------------------*/
