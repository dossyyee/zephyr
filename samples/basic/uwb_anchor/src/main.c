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
#include <zephyr/bluetooth/gatt.h>

#include <zephyr/drivers/ieee802154/deca_device_api.h>
#include <zephyr/drivers/ieee802154/deca_interface.h>
#include <zephyr/drivers/ieee802154/deca_types.h>
#include <zephyr/drivers/ieee802154/deca_version.h>

#include <zephyr/drivers/ieee802154/ieee802154_dw3xxx.h>
#include <zephyr/bluetooth/services/rs.h>


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
static void start_ranging(void);
static void key_changed(void);
static void iv_changed(void);

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
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_RS_VAL),
};
static const struct bt_data sd[] = {
	BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME, sizeof(CONFIG_BT_DEVICE_NAME) - 1),
	BT_DATA(BT_DATA_MANUFACTURER_DATA, mfg_data, sizeof(mfg_data)), /* Placeholder for anchor location/gps coords */
};

/* Periodic advertising roughly every 62 ms */
static const struct bt_le_adv_param adv_param[] = {BT_LE_ADV_PARAM_INIT(
	(BT_LE_ADV_OPT_CONNECTABLE | BT_LE_ADV_OPT_SCANNABLE), 
	// (BT_LE_ADV_OPT_USE_IDENTITY | BT_LE_ADV_OPT_CONNECTABLE | BT_LE_ADV_OPT_SCANNABLE), 
	0x0064, 0x0068, NULL
)};

/* Ranging Service */
static struct bt_rs_cb rs_callbacks = {
	.start_ranging = start_ranging,
	.key_changed = key_changed,
	.iv_changed = iv_changed,
};


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

static void start_ranging(void)
{
	LOG_INF("Start Ranging");
	uint32_t key[STS_U32_LEN];
	uint32_t iv[STS_U32_LEN];
	uint64_t timestamp[3];

	int err;

	bt_rs_get_key(key);
	bt_rs_get_iv(iv);
	// printk("IV: 0x%08X 0x%08X 0x%08X 0x%08X\n", iv[0], iv[1], iv[2], iv[3]);
	// printk("KEY: 0x%08X 0x%08X 0x%08X 0x%08X\n", key[0], key[1], key[2], key[3]);
	/* Temporarily the key and iv should be updated here. It should already be consistant with the 
	 * bluetooth service, but in case the callbacks have not been implemented or the key and iv have
	* not been initiated yet */

	dw3xxx_set_sts_key(uwb, key);
	dw3xxx_set_sts_iv(uwb, iv);


	// uint32_t tmp[STS_U32_LEN];
	// dw3xxx_get_sts_iv(uwb, tmp);
	// printk("IV: 0x%08X 0x%08X 0x%08X 0x%08X\n", tmp[0], tmp[1], tmp[2], tmp[3]);

	// dw3xxx_get_sts_key(uwb, tmp);
	// printk("KEY: 0x%08X 0x%08X 0x%08X 0x%08X\n", tmp[0], tmp[1], tmp[2], tmp[3]);

	err = run_responder(uwb, K_MSEC(10));
	if (err) {
		memset(timestamp, 0x00, sizeof(timestamp));
		bt_rs_set_timestamp(timestamp);
		bt_rs_indicate_timestamp(my_conn);
	} else {
		LOG_INF("Ranging Successful");
		dw3xxx_get_timestamp(uwb, timestamp);
		bt_rs_set_timestamp(timestamp);
		bt_rs_indicate_timestamp(my_conn);
	}
}

static void key_changed(void)
{
	/* Update the dw3000 device key*/
	uint32_t key[STS_U32_LEN];
	bt_rs_get_key(key);
	dw3xxx_set_sts_key(uwb, key);
}

static void iv_changed(void)
{
	/* Update the dw3000 device iv*/
	uint32_t iv[STS_U32_LEN];
	bt_rs_get_iv(iv);
	dw3xxx_set_sts_iv(uwb, iv);
}

void mtu_updated(struct bt_conn *conn, uint16_t tx, uint16_t rx)
{
	printk("Updated MTU: TX: %d RX: %d bytes\n", tx, rx);
}

static struct bt_gatt_cb gatt_callbacks = {
	.att_mtu_updated = mtu_updated,
};

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
	/* Register Callbacks */
	bt_conn_cb_register(&connection_callbacks);
	bt_gatt_cb_register(&gatt_callbacks);
	bt_rs_cb_init(&rs_callbacks);
	
	k_work_submit(&start_adv_worker);

	dw3xxx_configure_device(uwb, RANGING_RESPONDER, HRP_UWB_PHY_CHANNEL_9);
	/* Very important the the dw3000 irq is enabled after configuration */
	dw_enable_irq(uwb);

	// LOG_ERR("Device not run");
	return 0;
}
