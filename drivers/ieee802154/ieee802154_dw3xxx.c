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

/* Type Definitions ----------------------------------------------------------*/

typedef enum
{
        CMD_TXRXOFF             = 0x81 | (0x00 << 1),
        CMD_TX                  = 0x81 | (0x01 << 1),
        CMD_RX                  = 0x81 | (0x02 << 1),
        CMD_DTX                 = 0x81 | (0x03 << 1),
        FAST_CMD_DRX            = 0x81 | (0x04 << 1),
        CMD_DTX_TS              = 0x81 | (0x05 << 1),
        CMD_DRX_TS              = 0x81 | (0x06 << 1),
        CMD_DTX_RS              = 0x81 | (0x07 << 1),
        CMD_DRX_RS              = 0x81 | (0x08 << 1),
        CMD_DTX_REF             = 0x81 | (0x09 << 1),
        CMD_DRX_REF             = 0x81 | (0x0A << 1),
        CMD_CCA_TX              = 0x81 | (0x0B << 1),
        CMD_TX_W4R              = 0x81 | (0x0C << 1),
        CMD_DTX_W4R             = 0x81 | (0x0D << 1),
        CMD_DTX_TS_W4R          = 0x81 | (0x0E << 1),
        CMD_DTX_RS_W4R          = 0x81 | (0x0F << 1),
        CMD_DTX_REF_W4R         = 0x81 | (0x10 << 1),
        CMD_CCA_TX_W4R          = 0x81 | (0x11 << 1),
        CMD_CLR_IRQS            = 0x81 | (0x12 << 1),
        CMD_DB_TOGGLE           = 0x81 | (0x13 << 1),
} dw_fast_command_packet_e;

typedef enum{
        GEN_CFG_AES_0           = (0x00 << 1),
        GEN_CFG_AES_1           = (0x01 << 1),
        STS_CFG                 = (0x02 << 1),
        RX_TUNE                 = (0x03 << 1),
        EXT_SYNC                = (0x04 << 1),
        GPIO_CTRL               = (0x05 << 1),
        CMD_DRX                 = (0x06 << 1),
        RF_CONF                 = (0x07 << 1),
        RF_CAL                  = (0x08 << 1),
        FS_CTRL                 = (0X09 << 1),
        AON                     = (0X0A << 1),
        OTP_IF                  = (0X0B << 1),
        CIA_C                   = (0X0C << 1),
        CIA_D                   = (0X0D << 1),
        CIA_E                   = (0X0E << 1),
        DIG_DIAG                = (0X0F << 1),
        PMSC                    = (0X11 << 1),
        RX_BUFFER_0             = (0X12 << 1),
        RX_BUFFER_1             = (0X13 << 1),
        TX_BUFFER               = (0X14 << 1),
        ACC_MEM                 = (0X15 << 1),
        SCRATCH_RAM             = (0X16 << 1),
        AES_RAM                 = (0X17 << 1),
        SET_1_SET_2             = (0X18 << 1),
        INDIRECT_PTR_A          = (0X1D << 1),
        INDIRECT_PTR_B          = (0X1E << 1),
        IN_PTR_CFG              = (0X1F << 1),
} dw_base_addr_e;

typedef enum{
        DEV_ID                  = ((0x40 | GEN_CFG_AES_0) << 8) | (0x00 << 2),
        EUI_64                  = ((0x40 | GEN_CFG_AES_0) << 8) | (0x04 << 2),
        PANADR                  = ((0x40 | GEN_CFG_AES_0) << 8) | (0x0C << 2),
	SYS_CFG			= ((0x40 | GEN_CFG_AES_0) << 8) | (0x10 << 2),
	FF_CFG			= ((0x40 | GEN_CFG_AES_0) << 8) | (0x14 << 2),
	SPI_RD_CRC		= ((0x40 | GEN_CFG_AES_0) << 8) | (0x18 << 2),
	SYS_TIME		= ((0x40 | GEN_CFG_AES_0) << 8) | (0x1C << 2),
	TX_FCTRL		= ((0x40 | GEN_CFG_AES_0) << 8) | (0x24 << 2),
	DX_TIME			= ((0x40 | GEN_CFG_AES_0) << 8) | (0x2C << 2),
	DREF_TIME		= ((0x40 | GEN_CFG_AES_0) << 8) | (0x30 << 2),
	RX_FWTO			= ((0x40 | GEN_CFG_AES_0) << 8) | (0x34 << 2),
	SYS_CTRL		= ((0x40 | GEN_CFG_AES_0) << 8) | (0x38 << 2),
	SYS_ENABLE		= ((0x40 | GEN_CFG_AES_0) << 8) | (0x3C << 2),
	SYS_STATUS		= ((0x40 | GEN_CFG_AES_0) << 8) | (0x44 << 2),
	RX_FINFO		= ((0x40 | GEN_CFG_AES_0) << 8) | (0x4C << 2),
	RX_TIME			= ((0x40 | GEN_CFG_AES_0) << 8) | (0x64 << 2),
	TX_TIME			= ((0x40 | GEN_CFG_AES_0) << 8) | (0x74 << 2),
	TX_RAWST		= ((0x40 | GEN_CFG_AES_1) << 8) | (0x00 << 2),
	TX_ANTD			= ((0x40 | GEN_CFG_AES_1) << 8) | (0x04 << 2),
	ACK_RESP_T		= ((0x40 | GEN_CFG_AES_1) << 8) | (0x08 << 2),
	TX_POWER		= ((0x40 | GEN_CFG_AES_1) << 8) | (0x0C << 2),
	CHAN_CTRL		= ((0x40 | GEN_CFG_AES_1) << 8) | (0x14 << 2),
	LE_PEND_01		= ((0x40 | GEN_CFG_AES_1) << 8) | (0x18 << 2),
	LE_PEND_23		= ((0x40 | GEN_CFG_AES_1) << 8) | (0x1C << 2),
	SPI_COLLISION		= ((0x40 | GEN_CFG_AES_1) << 8) | (0x20 << 2),
	RDB_STATUS		= ((0x40 | GEN_CFG_AES_1) << 8) | (0x24 << 2),
	RDB_DIAG		= ((0x40 | GEN_CFG_AES_1) << 8) | (0x28 << 2),
	AES_CFG			= ((0x40 | GEN_CFG_AES_1) << 8) | (0x30 << 2),
	AES_IV0			= ((0x40 | GEN_CFG_AES_1) << 8) | (0x34 << 2),
	AES_IV1			= ((0x40 | GEN_CFG_AES_1) << 8) | (0x38 << 2),
	AES_IV2			= ((0x40 | GEN_CFG_AES_1) << 8) | (0x3C << 2),
	AES_IV3			= ((0x40 | GEN_CFG_AES_1) << 8) | (0x40 << 2),
	DMA_CFG			= ((0x40 | GEN_CFG_AES_1) << 8) | (0x44 << 2),
	AES_START		= ((0x40 | GEN_CFG_AES_1) << 8) | (0x4C << 2),
	AES_STS			= ((0x40 | GEN_CFG_AES_1) << 8) | (0x50 << 2),
	AES_KEY			= ((0x40 | GEN_CFG_AES_1) << 8) | (0x54 << 2),
	// Not comprehensive. Only GEN_CFG_AES registers included so far 
} dw_full_addr_e;

/* Function Declarations -----------------------------------------------------*/

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
	int rc;
	const struct dw3xxx_config *cfg = dev->config;

	const struct spi_buf tx_buf = {
		.buf = (uint8_t *)hdr_buf,
		.len = hdr_len
	};
	const struct spi_buf_set tx = {
		.buffers = &tx_buf,
		.count = 1
	};
	struct spi_buf rx_buf[2] = {
		{
			.buf = NULL,
			.len = hdr_len,
		},
		{
			.buf = (uint8_t *)data,
			.len = data_len,
		},
	};
	const struct spi_buf_set rx = {
		.buffers = rx_buf,
		.count = ARRAY_SIZE(rx_buf),
	};

	LOG_DBG("spi read, header length %u, data length %u", (uint16_t)hdr_len, (uint32_t)data_len);
	LOG_HEXDUMP_DBG(hdr_buf, (uint16_t)hdr_len, "rd: header");

	acquire_device(dev);
	rc = spi_transceive_dt(&cfg->bus, &tx, &rx);
	if (rc < 0) {
		LOG_ERR("SPI transfer failed");
		goto release_read;
	}

	LOG_HEXDUMP_DBG(data, (uint32_t)data_len, "rd: data");
release_read:
	release_device(dev);
	return rc;
}

/*----------------------------------------------------------------------------*/

int dw_spi_write(const struct device *dev, uint16_t hdr_len, const uint8_t *hdr_buf, uint32_t data_len, const uint8_t *data)
{
	int rc;
        size_t cnt = (data_len) ? 2 : 1;

	const struct dw3xxx_config *cfg = dev->config;
	struct spi_buf buf[2] = {
		{.buf = (uint8_t *)hdr_buf, .len = hdr_len},
		{.buf = (uint8_t *)data, .len = data_len}
	};
	struct spi_buf_set buf_set = {.buffers = buf, .count = cnt};

	LOG_DBG("spi write, header length %u, data length %u", (uint16_t)hdr_len, (uint32_t)data_len);
	LOG_HEXDUMP_DBG(hdr_buf, (uint16_t)hdr_len, "wr: header");
	LOG_HEXDUMP_DBG(data, (uint32_t)data_len, "wr: data");

	acquire_device(dev);
	rc = spi_write_dt(&cfg->bus, &buf_set);
	if (rc < 0) {
		LOG_ERR("SPI read failed");
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
	struct dw3xxx_data *data = dev->data;
	k_work_submit(&data->isr_work);
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
	gpio_pin_interrupt_configure_dt(&cfg->irq_gpio, GPIO_INT_EDGE_RISING);

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
