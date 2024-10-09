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
#include <zephyr/drivers/counter.h>
// #include <zephyr/timing/timing.h>

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

/* Requires timer0 to be enabled in device tree. Node label is specific to nrf5340 at this time.*/
static const struct device *const timer0 = DEVICE_DT_GET(DT_NODELABEL(timer0));

int main(void)
{
	printk("Entering Main\n");
	// timing_init();
	// timing_start();

	/* Grab the clock driver */
	if (!device_is_ready(timer0)) {
		printk("%s: device not ready.\n", timer0->name);
		return 0;
	}

	int ret;
	if (!gpio_is_ready_dt(&led)) {
		return 0;
	}	
	ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_INACTIVE);
	if (ret < 0) {
		return 0;
	}

	dw3xxx_configure_device(uwb, RANGING_INITIATOR, HRP_UWB_PHY_CHANNEL_9);

	dw_enable_irq(uwb);
	run_initiator_forever(uwb);


	LOG_ERR("Device not run");
	return 0;
}
