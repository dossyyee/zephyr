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

#include <zephyr/drivers/ieee802154/deca_device_api.h>
#include <zephyr/drivers/ieee802154/deca_interface.h>
#include <zephyr/drivers/ieee802154/deca_types.h>
#include <zephyr/drivers/ieee802154/deca_version.h>

#include <zephyr/drivers/ieee802154/ieee802154_dw3xxx.h>


LOG_MODULE_REGISTER(app);

#define LED0_NODE DT_ALIAS(led0)

static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
static const struct device *uwb = DEVICE_DT_GET(DT_INST(0, qorvo_dw3xxx));

/* 
 * The tag will be in a central role which will scan for a short scanning period once every couple
 * minutes or more frequently during times of movement, during this scan period, a graded list of 
 * anchors discovered is generated. 
 */

/* ------------------------------------ Function Prototypes --------------------------------------- */
static void connected(struct bt_conn *conn, uint8_t cnxn_err);
static void disconnected(struct bt_conn *conn, uint8_t reason);
static void start_scan(struct k_work *work);
/* ------------------------------------ Bluetooth Vairables --------------------------------------- */

static struct bt_conn *curr_conn = NULL;
static struct bt_conn_cb connection_callbacks = {
	.connected = connected,
	.disconnected = disconnected,
};
static K_WORK_DEFINE(start_scan_worker, start_scan);

/* ------------------------------------ Bluetooth Functions --------------------------------------- */
static void device_found(const bt_addr_le_t *addr, int8_t rssi, uint8_t type, struct net_buf_simple *ad)
{
	char addr_str[BT_ADDR_LE_STR_LEN];
	int err;
	/* 
	 * Process should be to:
	 * 1. Record the advertising devices nearby.
	 * 2. Filter those devices and record the ones who are anchors
	 * 3. If during a scan/discovery period:
	 *    Generate an evolving list of candidate anchors based on rssi, time of last advertisement
	 *    and last distance measurement to the device, number of failed ranging attempts. Develop a 
	 *    heuristic. Attempting to connect to anchors whose advertisments are close together but not 
	 *    too close would be ideal.
	 * 4. If during a ranging period:
	 *    Establish a connection with the anchor and schedule a ranging attempt
	 * 
	 */

	/* Only interested in connectable events */
	if (type != BT_GAP_ADV_TYPE_ADV_IND &&
	    type != BT_GAP_ADV_TYPE_ADV_DIRECT_IND) {
		return;
	}

	/* Only interested in anchors (advertised with ranging service )*/
	// TODO: implement this check

	/* Apply the heuristic to the anchor and slot it in the list*/
	// TODO: create a heuristic

	/* If the device found is highly rated, then attempt to create a connection to the device and */

	// bt_addr_le_to_str(addr, addr_str, sizeof(addr_str));
	// printk("Device found: %s (RSSI %d)\n", addr_str, rssi);

	// /* connect only to devices in close proximity */
	// if (rssi < -50) {
	// 	return;
	// }

	// if (bt_le_scan_stop()) {
	// 	return;
	// }

	// err = bt_conn_le_create(addr, BT_CONN_LE_CREATE_CONN,
	// 			BT_LE_CONN_PARAM_DEFAULT, &default_conn);
	// if (err) {
	// 	printk("Create conn to %s failed (%d)\n", addr_str, err);
	// 	start_scan();
	// }
}

static void start_scan(struct k_work *work)
{
	int err;

	err = bt_le_scan_start(BT_LE_SCAN_ACTIVE_CONTINUOUS, device_found);
	if (err) {
		LOG_ERR("Scanning failed to start (err %d)", err);
		return;
	}

	LOG_INF("Scanning successfully started\n");
}

// static void connected(struct bt_conn *conn, uint8_t err)
// {
// 	char addr[BT_ADDR_LE_STR_LEN];

// 	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

// 	if (err) {
// 		printk("Failed to connect to %s %u %s\n", addr, err, bt_hci_err_to_str(err));

// 		bt_conn_unref(curr_conn);
// 		curr_conn = NULL;

// 		start_scan();
// 		return;
// 	}

// 	if (conn != default_conn) {
// 		return;
// 	}

// 	printk("Connected: %s\n", addr);

// 	bt_conn_disconnect(conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
// }

// static void disconnected(struct bt_conn *conn, uint8_t reason)
// {
// 	char addr[BT_ADDR_LE_STR_LEN];

// 	if (conn != default_conn) {
// 		return;
// 	}

// 	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

// 	printk("Disconnected: %s, reason 0x%02x %s\n", addr, reason, bt_hci_err_to_str(reason));

// 	bt_conn_unref(default_conn);
// 	default_conn = NULL;

// 	start_scan();
// }

// BT_CONN_CB_DEFINE(conn_callbacks) = {
// 	.connected = connected,
// 	.disconnected = disconnected,
// };

int main(void)
{
	printk("Entering Main\n");
	
	/* Set up LED for debugging */
	int err;
	if (!gpio_is_ready_dt(&led)) {
		return 0;
	}
	err = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
	if (err < 0) {
		return 0;
	}

	/* Check uwb device is ready */
	if (!device_is_ready(uwb)) {
		return 0;
	}

	/* Enable Bluetooth */
	err = bt_enable(NULL);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return 0U;
	}

	bt_conn_cb_register(&connection_callbacks);
	k_work_submit(&start_adv_worker);

	
	dw3xxx_configure_device(uwb, RANGING_RESPONDER, HRP_UWB_PHY_CHANNEL_9);
	dw_enable_irq(uwb);
	while (1) {
		run_responder(uwb);
	}

	LOG_ERR("Device not run");
	return 0;
}
