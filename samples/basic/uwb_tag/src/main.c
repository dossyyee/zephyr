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

/* The devicetree node identifier for the "led0" alias. */
#define LED0_NODE DT_ALIAS(led0)

static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
static const struct device *uwb = DEVICE_DT_GET(DT_INST(0, qorvo_dw3xxx));

int main(void)
{
	printk("Entering Main\n");
	
	/* Set up LED for debugging */
	int ret;
	if (!gpio_is_ready_dt(&led)) {
		return 0;
	}	
	ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) {
		return 0;
	}

	/* Check uwb device is ready */
	if (!device_is_ready(uwb)) {
		return 0;
	}
	
	dw3xxx_configure_device(uwb, RANGING_RESPONDER, HRP_UWB_PHY_CHANNEL_9);
	dw_enable_irq(uwb);
	while (1) {
		run_responder(uwb);
	}

	LOG_ERR("Device not run");
	return 0;
}
