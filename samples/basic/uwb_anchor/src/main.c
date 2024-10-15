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
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>

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

/* 
 * The anchor will be in a peripheral role which only maintains one active connection to a tag.
 * The anchor uses extended advertising to 
 */

/* ------------------------------------ Function Prototypes --------------------------------------- */
static void connected(struct bt_conn *conn, uint8_t cnxn_err);
static void disconnected(struct bt_conn *conn, uint8_t reason);
static void start_adv(struct k_work *work);

/* ------------------------------------ Bluetooth Vairables --------------------------------------- */
static K_WORK_DEFINE(start_adv_worker, start_adv);

/* Connection */
static struct bt_conn *my_conn = NULL;
static struct bt_conn_cb connection_callbacks = {
	.connected = connected,
	.disconnected = disconnected,
};

/* Advertising */
static uint8_t mfg_data[] = { 0xff, 0xff, 0x00 };
static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL|BT_LE_AD_NO_BREDR)),
	BT_DATA_BYTES(BT_DATA_UUID16_ALL, BT_UUID_16_ENCODE(BT_UUID_BAS_VAL)), /* Change this to the custom ranging servive when implemented*/
};
static const struct bt_data sd[] = {
	BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME, sizeof(CONFIG_BT_DEVICE_NAME) - 1),
	BT_DATA(BT_DATA_MANUFACTURER_DATA, mfg_data, sizeof(mfg_data)), /* Placeholder for anchor location/gps coords */
};

/* Periodic advertising roughly every 211 ms */
static const struct bt_le_adv_param adv_param[] = {BT_LE_ADV_PARAM_INIT(
	(BT_LE_ADV_OPT_USE_IDENTITY | BT_LE_ADV_OPT_CONNECTABLE ), 
	0x0150, 0x0154, NULL
)};


/* ------------------------------------ Bluetooth Functions --------------------------------------- */
static void start_adv(struct k_work *work)
{
	int err;
	err = bt_le_adv_start(adv_param, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
	if (err) {
		LOG_ERR("Failed to start advertising");
	} else {
		LOG_INF("Starting advertising");
	}
}

static void connected(struct bt_conn *conn, uint8_t cnxn_err) 
{
	int err;
	if (cnxn_err) {
		LOG_ERR("Connection error: 0x%02x", cnxn_err);
	} else {
		LOG_INF("Connected");
		my_conn = bt_conn_ref(conn);

		err = bt_le_adv_stop();
		if (err) {
			LOG_ERR("Failed to stop periodic advertising");
		}
	}
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	LOG_INF("Disconnected. Reason: 0x%02x", reason);
	bt_conn_unref(conn);
	k_work_submit(&start_adv_worker);
}

int main(void)
{
	printk("Entering Main\n");
	
	int err;
	if (!gpio_is_ready_dt(&led)) {
		return 0;
	}	
	err = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
	if (err < 0) {
		return 0;
	}

	/* Enable bluetooth */
	err = bt_enable(NULL);
	if (err < 0) {
		LOG_ERR("Bluetooth enable failed.");
		return 0;
	}
	bt_conn_cb_register(&connection_callbacks);
	
	k_work_submit(&start_adv_worker);

	dw3xxx_configure_device(uwb, RANGING_RESPONDER, HRP_UWB_PHY_CHANNEL_9);
	/* Very important the the dw3000 irq is enabled after configuration */
	dw_enable_irq(uwb);

	while (1) {
		run_responder(uwb);
	}

	LOG_ERR("Device not run");
	return 0;
}
