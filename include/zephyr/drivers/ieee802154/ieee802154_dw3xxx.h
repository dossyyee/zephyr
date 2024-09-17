
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


int dw_spi_read(const struct device *dev, uint16_t hdr_len, const uint8_t *hdr_buf, uint32_t data_len, uint8_t *data);
int dw_spi_write(const struct device *dev, uint16_t hdr_len, const uint8_t *hdr_buf, uint32_t data_len,	const uint8_t *data);
void dw_wakeup(const struct device *dev);
void dw_reset(const struct device *dev);


