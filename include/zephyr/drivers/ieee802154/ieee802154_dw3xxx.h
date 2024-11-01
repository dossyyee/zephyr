
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/ieee802154/deca_device_api.h>
#include <zephyr/drivers/ieee802154/deca_interface.h>

/* Unit conversion macros. From Microseconds to relevant register units */
#define US_TO_UUS(us) ((us) * 40 / 39)          // RX_FWTO register in units of 512/499.2
#define US_TO_CUS(us) ((us) * 128 / 125)        // ACK_RESP register in units of 128 system clock cycles @ 125MHz = 1.024us
#define US_TO_DLY_UNIT(us) ((us) * 1248 / 5)    // DX_TIME register in units of 1 / (499.2Mhz / 2) ~= 4.006ns

#define TX_ANT_DLY 16385
#define RX_ANT_DLY 16385

/* Time periods to tune */
#define PREAMBLE_HUNT 400        // Desired time for preamble hunt, can be minimised to zero
#define TX_CFG_TIME 200           // Time of dw3000 to process TX start + a buffer period
#define R_CPU_PROCESSING 398    // Time from interrupt reception to start tx command
#define I_CPU_PROCESSING 442    // Time from interrupt reception to start tx command

#define RX_TIMEOUT US_TO_UUS(500)          // Time after rx wakeup to call timout error
#define PREAMBLE_TIMEOUT 4      // How mand PACs to wait and flag a timout

#define PREAMBLE_DURATION 65
#define STARUP_PLL 20

#define RESPONDER_DETERMINISTIC_DELAY US_TO_DLY_UNIT(PREAMBLE_DURATION + R_CPU_PROCESSING + TX_CFG_TIME)
#define INITIATOR_DETERMINISTIC_DELAY US_TO_DLY_UNIT(PREAMBLE_DURATION + I_CPU_PROCESSING + TX_CFG_TIME)

#define INITIATOR_RX_WAKEUP_DELAY US_TO_CUS(PREAMBLE_DURATION + R_CPU_PROCESSING + TX_CFG_TIME - STARUP_PLL - PREAMBLE_HUNT)
#define RESPONDER_RX_WAKEUP_DELAY US_TO_CUS(PREAMBLE_DURATION + I_CPU_PROCESSING + TX_CFG_TIME - STARUP_PLL - PREAMBLE_HUNT)


#define HRP_UWB_PHY_CHANNEL_5 5
#define HRP_UWB_PHY_CHANNEL_9 9

#define RX_EVENT_OK	        1
#define RX_EVENT_ERROR	        -1
#define RX_EVENT_ERROR_TO       -2
#define RX_EVENT_BAD_STS        -3
#define ERROR_POLL_TIMEOUT      -4
#define UNKNOWN_ERROR           -5

struct dw_isr_callbacks {
	void (*cbTxDone)(const dwt_cb_data_t*);
	void (*cbRxOk)(const dwt_cb_data_t*);
	void (*cbRxTo)(const dwt_cb_data_t*);
	void (*cbRxErr)(const dwt_cb_data_t*);
	void (*cbSPIErr)(const dwt_cb_data_t*);
	void (*cbSPIRdy)(const dwt_cb_data_t*);
};

/* Timestamp struct */
struct dw_dstwr_ts_t {
        uint64_t ts1;
        uint64_t ts2;
        uint64_t ts3;
};

// typedef enum {
//         RANGING_NONE,
//         RANGING_ENABLED = 0b10,
//         RANGING_INITIATOR = RANGING_ENABLED | 0b00,
//         RANGING_RESPONDER = RANGING_ENABLED | 0b01,
//         TIME_SYNC_NONE = 0,
//         TIME_SYNC_ENABLED = 0b10 << 2,
//         TIME_SYNC_CONTROLER = TIME_SYNC_ENABLED | (0b00 <<2),
//         TIME_SYNC_LISTNER = TIME_SYNC_ENABLED | (0b01 <<2),
// } dw_configrole_e __packed;
typedef enum {
        RANGING_INITIATOR,
        RANGING_RESPONDER,
        TIME_SYNC_CONTROLER,
        TIME_SYNC_LISTNER,
} dw_configrole_e;

struct dw3xxx_config {
	struct spi_dt_spec bus;
	struct gpio_dt_spec irq_gpio;
        struct gpio_dt_spec wkp_gpio; // better to implement kconfig to specify whether to use spi wakeup or wakeup pin
	struct gpio_dt_spec rst_gpio;
        struct gpio_dt_spec sync_gpio;
};

struct dw3xxx_data {
	struct gpio_callback irq_callback;
        struct k_work isr_work;
	// struct k_poll_signal irq_signal;
        struct k_poll_signal rx_sig;
        struct k_poll_event rng_event[1]; /* Ranging event */
        struct k_poll_signal radio_wakeup;

        dwt_config_t phy_cfg;
        dwt_txconfig_t tx_pwr_cfg;
        struct dw_isr_callbacks cbs;
        dw_configrole_e role;
        uint32_t irq_mask_hi;
        uint32_t irq_mask_lo;

        struct dw_dstwr_ts_t timestamps;
        uint8_t rx_count;
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

static const uint8_t tx_pwr_lut_ch5[64] = {

};
// make a python script which creates the lookup table which is optimised for the lowest possible coarse grain for a 
// given input




int dw_spi_read(const struct device *dev, uint16_t hdr_len, const uint8_t *hdr_buf, uint32_t data_len, uint8_t *data);
int dw_spi_write(const struct device *dev, uint16_t hdr_len, const uint8_t *hdr_buf, uint32_t data_len,	const uint8_t *data);
void dw_wakeup(const struct device *dev);
void dw_reset(const struct device *dev);
void dw_enable_irq(const struct device* dev);
void dw_disable_irq(const struct device* dev);
int dw3xxx_configure_device(const struct device* dev, dw_configrole_e role, uint8_t channel);
void dw3xxx_set_sts_key(const struct device *dev, uint32_t *key);
void dw3xxx_set_sts_iv(const struct device *dev, uint32_t *iv);
void dw3xxx_set_sts_counter(const struct device *dev, uint32_t iv);
void dw3xxx_get_timestamp(const struct device *dev, uint64_t *ts);
uint32_t dw3xxx_get_sts_counter(const struct device *dev);
void dw3xxx_get_sts_iv(const struct device *dev, uint32_t *iv);
void dw3xxx_get_sts_key(const struct device *dev, uint32_t *key);

// Placeholder functions for demo
void run_initiator_forever(const struct device* dev);
void run_responder_forever(const struct device* dev);

int run_responder(const struct device* dev, k_timeout_t timeout);
int run_initiator(const struct device* dev);