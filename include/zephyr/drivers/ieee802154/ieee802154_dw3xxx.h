
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>



struct dw3xxx_config {
	struct spi_dt_spec bus;
	struct gpio_dt_spec irq_gpio;
    struct gpio_dt_spec wkp_gpio; // better to implement kconfig to specify whether to use spi wakeup or wakeup pin
	struct gpio_dt_spec rst_gpio;
};

struct dw3xxx_data {
	struct gpio_callback irq_callback;
    struct k_work isr_work;
	struct k_poll_signal irq_signal;
};




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



int dw_spi_read(const struct device *dev, uint16_t hdr_len, const uint8_t *hdr_buf, uint32_t data_len, uint8_t *data);
int dw_spi_write(const struct device *dev, uint16_t hdr_len, const uint8_t *hdr_buf, uint32_t data_len,	const uint8_t *data);
void dw_wakeup(const struct device *dev);
void dw_reset(const struct device *dev);
void dw_enable_irq(const struct device* dev);
void dw_disable_irq(const struct device* dev);
