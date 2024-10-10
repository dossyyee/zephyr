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
#include <zephyr/timing/timing.h>

#include <zephyr/drivers/ieee802154/deca_device_api.h>
#include <zephyr/drivers/ieee802154/deca_interface.h>
#include <zephyr/drivers/ieee802154/deca_types.h>
#include <zephyr/drivers/ieee802154/deca_version.h>

#include <zephyr/drivers/ieee802154/ieee802154_dw3xxx.h>


LOG_MODULE_REGISTER(app);

/* 1000 msec = 1 sec */
#define DELAY_US   1000

#define ALARM_CHANNEL_0 0

#define  ARM_CM_DEMCR      (*(uint32_t *)0xE000EDFC)
#define  ARM_CM_DWT_CTRL   (*(uint32_t *)0xE0001000)
#define  ARM_CM_DWT_CYCCNT (*(uint32_t *)0xE0001004)

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
struct counter_alarm_cfg alarm_cfg;

uint64_t log_time1;
uint64_t log_time2;

uint32_t  start;
uint32_t  stop;
uint32_t  delta;

static void counter_interrupt_fn(const struct device *counter_dev,
				      uint8_t chan_id, uint32_t ticks,
				      void *user_data)
{
	log_time2 = timing_counter_get();
	stop  = ARM_CM_DWT_CYCCNT;
	LOG_INF("Log Time: %llu us", timing_cycles_to_ns(log_time2 - log_time1)/1000);
	LOG_INF("t1: %llu\tt2: %llu", log_time1, log_time2);
	LOG_INF("t1: %u\tt2: %u", start, stop);
}

int main(void)
{
	if (ARM_CM_DWT_CTRL != 0) {        // See if DWT is available
		ARM_CM_DEMCR      |= 1 << 24;  // Set bit 24
		ARM_CM_DWT_CYCCNT  = 0;
		ARM_CM_DWT_CTRL   |= 1 << 0;   // Set bit 0
	}

	int ret;
	printk("Entering Main\n");
	/* Timing for debug and performance measuring */
	timing_init();
	timing_start();

	LOG_INF("Timing freq: %lld", timing_freq_get());

	/* Grab the clock driver */
	if (!device_is_ready(timer0)) {
		LOG_ERR("%s: device not ready.", timer0->name);
		return 0;
	}

	counter_start(timer0);

	/* configure alarm */
	alarm_cfg.flags = 0;	/* No flags set */
	alarm_cfg.ticks = counter_us_to_ticks(timer0, DELAY_US);
	LOG_INF("%d", counter_us_to_ticks(timer0, DELAY_US));

	alarm_cfg.callback = counter_interrupt_fn;
	alarm_cfg.user_data = &alarm_cfg;

	while (1) {
		
		/* Set the counter in motion. If configured */
		ret = counter_set_channel_alarm(timer0, ALARM_CHANNEL_0, &alarm_cfg);
		log_time1 = timing_counter_get();
		start = ARM_CM_DWT_CYCCNT;

		if (ret  < 0 ) {
			LOG_ERR("Error setting counter: %d", ret);
		}

		//k_msleep(1000);
		k_busy_wait(1000000);
	}


	
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
