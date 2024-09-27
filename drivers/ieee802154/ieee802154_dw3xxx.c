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

/* Private Defines -----------------------------------------------------------*/
#define MAX_DATA_BUF_LEN 200

#define  ARM_CM_DEMCR      (*(uint32_t *)0xE000EDFC)
#define  ARM_CM_DWT_CTRL   (*(uint32_t *)0xE0001000)
#define  ARM_CM_DWT_CYCCNT (*(uint32_t *)0xE0001004)

/* Type Definitions ----------------------------------------------------------*/

/* Function Declarations -----------------------------------------------------*/
static uint8_t inter_tx_buf[MAX_DATA_BUF_LEN] = {0};
static uint8_t inter_rx_buf[MAX_DATA_BUF_LEN] = {0};

static uint32_t  start;
static uint32_t  stop;
static uint32_t  delta;


/* Private Variables ---------------------------------------------------------*/


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

/*----------------------------------------------------------------------------*/

static int dw_spi_fast_command(const struct device *dev, dw_fast_command_packet_e command)
{
        uint8_t buf = command;
        
        /* Fast command consists of an 8 bit packet with no additional data. */
        return dw_spi_write(dev, 1, &buf, 0, NULL);
}

/*----------------------------------------------------------------------------*/

static int dw_short_addr_read(const struct device *dev, dw_base_addr_e base, uint32_t data_len,
		       	uint8_t *data)
{
        uint8_t buf = base;

        return dw_spi_read(dev, 1, &buf, data_len, data);
}

/*----------------------------------------------------------------------------*/

static int dw_short_addr_write(const struct device *dev, dw_base_addr_e base, uint32_t data_len,
		       	uint8_t *data)
{
        uint8_t buf = 0x80 | base;

        return dw_spi_write(dev, 1, &buf, data_len, data);
}

/*----------------------------------------------------------------------------*/

static int dw_full_addr_read(const struct device *dev, dw_full_addr_e addr, uint32_t data_len, 
			uint8_t *data)
{
	uint8_t header[2] = {(addr >> 8), (addr & 0xFF)};
	return dw_spi_read(dev, sizeof(header), header, data_len, data);
}

/*----------------------------------------------------------------------------*/

static int dw_full_addr_write(const struct device *dev, dw_full_addr_e addr, uint32_t data_len, 
			uint8_t *data)
{
	uint8_t header[2] = {(addr >> 8) | 0x80, (addr & 0xFF)};
	return dw_spi_write(dev, sizeof(header), header, data_len, data);
}

/*----------------------------------------------------------------------------*/

static int dw_full_addr_masked_write(const struct device *dev, dw_full_addr_e addr, uint32_t data_len, 
			uint8_t *data)
{
	uint8_t header[2] = {(addr >> 8) | 0x80, (addr & 0xFF)};

	switch (data_len)
	{
	/*
	 * Data length of 2 indicates mode 1. 1 octal AND and OR masks
	 * Data length of 4 indicates mode 2. 2 octal AND and OR masks
	 * Data length of 8 indicates mode 3. 4 octal And and OR masks
	 * Data length other than these values is invalid.
	 */
	case BIT(1):
	case BIT(2):
		header[1] |= data_len >> 1;
		break;
	case BIT(3):
		header[1] |= 0x03;
		break;
	default:
		LOG_ERR("Invalid Masked Write Length");
		return -EINVAL;
		break;
	}

	return dw_spi_write(dev, sizeof(header), header, data_len, data);
}

/*----------------------------------------------------------------------------*/

void dw_wakeup(const struct device *dev)
{
	const struct dw3xxx_config *cfg = dev->config;
	printk("wakeup\n");

	if (&cfg->wkp_gpio.port) {
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

static void dw3xxx_irq_handler(struct k_work* item)
{
	// struct dw3xxx_data *data = CONTAINER_OF(cb, struct dw3xxx_data, irq_callback);

	/* Ensure that only one pin was triggered */
	// __ASSERT(POPCOUNT(pins) == 1, "Unknown interrupts %02X", pins);

	/* Send data ready signal */
	// k_poll_signal_raise(&data->irq_signal, 1);
	dwt_isr();
}

static void dw3xxx_hw_isr(const struct device* dev, struct gpio_callback* cb, uint32_t pins)
{
	struct dw3xxx_data *data = CONTAINER_OF(cb, struct dw3xxx_data, irq_callback);

	k_work_submit(&data->isr_work);
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

/*----------------------------------------------------------------------------*/

static int dw_configure(const struct device *dev)
{
	int rc;
	//acquire_device(dev);

	/* Query device ID */
	uint8_t id_buf[sizeof(uint32_t)] = {0};
	uint32_t dev_id = 0;

	/* Only short address required for device ID query */
	rc = dw_short_addr_read(dev, GEN_CFG_AES_0, sizeof(uint32_t), id_buf);
	if (rc < 0) {
		LOG_ERR("Could not read DEV_ID");
		goto release;
	}

	dev_id = sys_get_le32(id_buf);

	// TODO : write a better device ID check that isnt so magical
	if ((dev_id & 0xFFFFFF0F) != 0xDECA0302) {
		LOG_ERR("Incorrect DEV_ID found");
		goto release;
	}

	printk("DEV_ID: %08X\n", dev_id);

	/* testing write functionality */
/* 	uint8_t eui_buf[8] = {0};

	rc = dw_full_addr_read(dev, EUI_64, sizeof(eui_buf), eui_buf);
	if (rc < 0) {
		LOG_ERR("Reading EUI failed");
		goto release;
	}
	uint64_t eui_ident = sys_get_le64(eui_buf);
	printk("EUI: %llX\n", eui_ident);

	uint64_t new_eui = 	0x1234567890ABCDEF;
	uint8_t new_eui_buf[8] = {0};

	sys_put_le64(new_eui, new_eui_buf);
	rc = dw_full_addr_write(dev, EUI_64, sizeof(new_eui_buf), new_eui_buf);
	if (rc < 0) {
		LOG_ERR("Writing EUI Failed");
		goto release;
	}


	uint32_t and_mask 	= 0x00000000;
	uint32_t or_mask 	= 0xFEDCBAFE;
	uint64_t mask = ((uint64_t)or_mask << 32) | and_mask;
	uint8_t mask_buf[8] = {0};
	sys_put_le64(mask, mask_buf);
	dw_full_addr_masked_write(dev, EUI_64, sizeof(mask_buf), mask_buf);
 */
release:
	//release_device(dev);
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
		dw_configure(dev);
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

	//printk("DWT_CTRL: %08x\n", ARM_CM_DWT_CTRL);
	//printk("DEMCR: %08x\n", ARM_CM_DEMCR);

	/* Enabling dwt timing debug shenanigans */
	if (ARM_CM_DWT_CTRL != 0) {        // See if DWT is available. tbh this isnt really what this check does
		ARM_CM_DEMCR      |= 1 << 24;  // Enable DWT and ITM blocks
		ARM_CM_DWT_CYCCNT  = 0;
		ARM_CM_DWT_CTRL   |= 1 << 0;   // Enable cyccnt
	}

	printk("DW3xxx init\n");

	//k_poll_signal_init(&data->irq_signal);

	if (!spi_is_ready_dt(&cfg->bus)) {
		LOG_ERR("SPI bus not ready %s", cfg->bus.bus->name);
		return -ENODEV;
	}

	/* Setup the irq interrupt, to be enabled later */
	if (!device_is_ready(cfg->irq_gpio.port)) {
		LOG_ERR("GPIO device %s is not ready", cfg->irq_gpio.port->name);
		return -ENODEV;
	}

	k_work_init(&data->isr_work, dw3xxx_irq_handler);
	gpio_init_callback(&data->irq_callback, dw3xxx_hw_isr, BIT(cfg->irq_gpio.pin));
	rc = gpio_add_callback(cfg->irq_gpio.port, &data->irq_callback);
	if (rc < 0) {
		LOG_ERR("Failed to initialise data ready callback!");
		return rc;
	}
	gpio_pin_interrupt_configure_dt(&cfg->irq_gpio, GPIO_INT_DISABLE);

	/* Setup the wakeup pin */
	if (!device_is_ready(cfg->wkp_gpio.port)) {
		LOG_ERR("GPIO device %s is not ready", cfg->wkp_gpio.port->name);
		return -ENODEV;
	}
	rc = gpio_pin_configure_dt(&cfg->wkp_gpio, GPIO_OUTPUT_ACTIVE);
	if (rc < 0) {
		LOG_ERR("Failed to configure Wakeup gpio!");
		return rc;
	}

	/* Setup the reset pin */
	rc = gpio_pin_configure_dt(&cfg->rst_gpio, GPIO_INPUT);
	if (rc < 0) {
		LOG_ERR("Failed to configure reset gpio!");
		return rc;
	}

	return pm_device_driver_init(dev, dw3xxx_pm_control);
}

/*----------------------------------------------------------------------------*/

#define DW3XXX_SPI_OP_MODE SPI_OP_MODE_MASTER | SPI_WORD_SET(8) | SPI_TRANSFER_MSB

#define DW3XXX_INIT(n)                                                                      \
	struct dw3xxx_data dw3xxx_data_##n;                                                 \
	static const struct dw3xxx_config dw3xxx_config_##n = {                             \
		.bus = SPI_DT_SPEC_INST_GET(n, DW3XXX_SPI_OP_MODE, 0),                      \
		.irq_gpio = GPIO_DT_SPEC_INST_GET(n, irq_gpios),                            \
		.wkp_gpio = GPIO_DT_SPEC_INST_GET(n, wakeup_gpios),				\
		.rst_gpio = GPIO_DT_SPEC_INST_GET(n, reset_gpios),				\
	};                                                                                  \
	PM_DEVICE_DT_INST_DEFINE(n, dw3xxx_pm_control);                                     \
	DEVICE_DT_INST_DEFINE(n, dw3xxx_init, PM_DEVICE_DT_INST_GET(n), &dw3xxx_data_##n,   \
			      &dw3xxx_config_##n, POST_KERNEL, CONFIG_IEEE802154_DW3XXX_INIT_PRIO, \
			      NULL);

DT_INST_FOREACH_STATUS_OKAY(DW3XXX_INIT)

/*----------------------------------------------------------------------------*/
