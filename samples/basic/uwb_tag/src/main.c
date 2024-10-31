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
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>

#include <zephyr/bluetooth/services/rs.h>

#include <zephyr/drivers/ieee802154/deca_device_api.h>
#include <zephyr/drivers/ieee802154/deca_interface.h>
#include <zephyr/drivers/ieee802154/deca_types.h>
#include <zephyr/drivers/ieee802154/deca_version.h>
#include <zephyr/drivers/ieee802154/ieee802154_dw3xxx.h>


LOG_MODULE_REGISTER(app);

#define LED0_NODE 		DT_ALIAS(led0)
#define LONG_SCAN_INTERVAL_MS	(15 * 60 * 1000)	/* 15 Minutes */
#define SHORT_SCAN_INTERVAL_MS	(1 * 60 * 1000)		/* 1 Minute */
#define RANGING_INTERVAL_MS	(5 * 1000)		/* 5 Seconds */
#define LONG_SCAN_DURATION_MS	(2000) 			/* 2 Second Active Scan */
#define SHORT_SCAN_DURATION_MS	(500) 			/* 0.5 second Passive Scan */

#define MAX_RANGING_ATTEMPTS	5

#define ANCHOR_LIST_LEN		15	
#define ANCHOR_SHORTLIST_LEN	6

/* 
 * The tag will be in a central role which will scan for a short scanning period once every couple
 * minutes or more frequently during times of movement, during this scan period, a graded list of 
 * anchors discovered is generated. 
 */

/* ------------------------------------ Function Prototypes --------------------------------------- */
static void connected(struct bt_conn *conn, uint8_t cnxn_err);
static void disconnected(struct bt_conn *conn, uint8_t reason);
// static void start_scan(struct k_work *work);

static void long_scan_handler(struct k_work *work); /* Prime the list of devices. */
static void stop_long_scan_handler(struct k_work *work); /* Reschedule the list priming */
static void short_scan_handler(struct k_work *work); /* Short scan prior to ranging */
static void stop_short_scan_handler(struct k_work *work); /* Reschedule the list priming */
static void do_ranging_handler(struct k_work *work);
static void do_ranging_thread(void *arg1, void *arg2, void *arg3);
// static void prune_anchors_handler(struct k_work *work);/* Get rid of unheard from anchors */

/* A long scan uses active scanning and is used to get a comprehensive list of all the nearby devices
and to populate their information such as gps coordinates. It is run periodically at a large time 
interval. A short scan is run more frequently and is used to update the score of the devices in the 
shortlist. When in a ranging mode, this shortlist is used to decide whether to attempt to connect to a 
device or to wait until a better candidate is found. Since the reliability of a ranging attempt is not 
100% this method is proposed as a way to minimise the number of unsuccessful connection attempts and 
thus lower power consumption. */

/* ----------------------------------------- Work Items ------------------------------------------- */

static K_WORK_DELAYABLE_DEFINE(long_scan_work, long_scan_handler);
static K_WORK_DELAYABLE_DEFINE(stop_long_scan_work, stop_long_scan_handler);
static K_WORK_DELAYABLE_DEFINE(short_scan_work, short_scan_handler);
static K_WORK_DELAYABLE_DEFINE(stop_short_scan_work, stop_short_scan_handler);
static K_WORK_DELAYABLE_DEFINE(do_ranging_work, do_ranging_handler);
// static K_WORK_DELAYABLE_DEFINE(prune_anchors_work, prune_anchors_handler);

#define RANGING_STACK_SIZE	1024
#define RANGING_PRIORITY	10

K_THREAD_DEFINE(ranging_thread_id, RANGING_STACK_SIZE,
		do_ranging_thread, NULL, NULL, NULL,
		RANGING_PRIORITY, 0, 0);


/* ------------------------------------ Bluetooth Vairables --------------------------------------- */

static struct bt_conn *curr_conn = NULL;
static struct bt_conn_cb connection_callbacks = {
	.connected = connected,
	.disconnected = disconnected,
};
struct bt_conn_le_create_param conn_create_param[] = {{
	.options = BT_CONN_LE_OPT_NONE,
	.interval = BT_GAP_SCAN_FAST_INTERVAL,
	.window = BT_GAP_SCAN_FAST_WINDOW,
	.interval_coded = 0, /* Same as 1M */
	.window_coded = 0, /* Same as 1M */
	.timeout = 70 /* Connection initiation timeout (N * 10 MS). The reliability of achieveing a connection will depend on advertising interval and this */
}};
/* ----------------------------------- Application Vairables -------------------------------------- */
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
static const struct device *uwb = DEVICE_DT_GET(DT_INST(0, qorvo_dw3xxx));

/* operational mode of the application */
enum op_mode {
	IDLE,
	SHORT_SCAN,
	LONG_SCAN,
	RANGING,
};

/* Struct for gps coordinates */
struct gps_coord {
	int32_t latitude;
	int32_t longitude;
	int32_t height;
};
/* struct for storing known anchors */
struct anchor {
	bt_addr_le_t addr;
	int8_t rssi; // Dont know the usefulness of this at this point in time. could store last distance instead
	int reliability; // A record of past ranging attempts
	int last_heard; // Change this to use Date time. Used to drop devices from the list when no longer around.
	struct gps_coord coord;
	bool exists; // temporary measure for knowing if a anchor in the list exists or not
};
struct quick_anchor {
	bt_addr_le_t addr;
	int score;
};

static struct anchor anchor_list[ANCHOR_LIST_LEN] = {0}; /* A record of the nearby anchors. */
static struct quick_anchor shortlist[ANCHOR_SHORTLIST_LEN]; /* Passive scan updated, Ranked list of achors to range with */
enum op_mode app_state = IDLE;
static atomic_t CONNECTED = ATOMIC_INIT(0);
struct k_poll_signal conn_sig;
struct k_poll_signal write_complete_sig;
struct k_poll_event conn_event[1];
struct k_poll_event rng_event[1];

static struct bt_uuid_128 rs_uuid = BT_UUID_INIT_128(BT_UUID_RS_VAL);
static struct bt_uuid_128 count_char_uuid = BT_UUID_INIT_128(BT_UUID_RS_COUNTER_CHAR_VAL);
static struct bt_uuid_128 key_char_uuid = BT_UUID_INIT_128(BT_UUID_RS_KEY_CHAR_VAL);
static struct bt_uuid_128 iv_char_uuid = BT_UUID_INIT_128(BT_UUID_RS_IV_CHAR_VAL);
static struct bt_uuid_128 ts_char_uuid = BT_UUID_INIT_128(BT_UUID_RS_TS_CHAR_VAL);
static struct bt_uuid_128 rc_char_uuid = BT_UUID_INIT_128(BT_UUID_RS_RNGCMD_CHAR_VAL);
static struct bt_uuid_16 ccc_char_uuid = BT_UUID_INIT_16(BT_UUID_GATT_CCC_VAL);

/* ------------------------------------ Bluetooth Functions --------------------------------------- */
/** @brief Check the status of the atomic bet CONNECTED which mirrors the bluetooth connection status
 *
 *  @return true if CONNECTED == 1, otherwise false.
 */
static inline bool is_device_connected(void)
{
	return atomic_get(&CONNECTED) == 1;
}

/** @brief Callback used with bt_data_parse to check the advertisement packet contains a UUID matching the 
 * Ranging service uuid.
 *
 *  @param data Data to check
 *  @param user_data Boolean value used to store the outcome of the check
 *
 *  @return true if next data in advertisement should be checked, otherwise false.
 */
static bool is_anchor(struct bt_data *data, void *user_data)
{
	bool *found = user_data;
	// char rs_uuid_str[BT_UUID_STR_LEN] = {0};
	// char uuid_str[BT_UUID_STR_LEN] = {0};
	// struct bt_uuid_128 t = {0};
	// struct bt_uuid *u = (struct bt_uuid *)(&t);
	/* check if the data is a UUID. Expecting only one UUID to be advertised */
	if (data->type == BT_DATA_UUID128_ALL) {
		/* Check that the uuid matches the Ranging Service uuid */
		// bt_uuid_create(u, data->data, 16);
		// bt_uuid_to_str(&uuid, uuid_str, BT_UUID_STR_LEN);
		// bt_uuid_to_str(BT_UUID_RS, rs_uuid_str, BT_UUID_STR_LEN);
		// printk("UUID128 adv. UUID: %s\tRS UUID: %s\n", uuid_str, rs_uuid_str);
		// if (!bt_uuid_cmp(&rs_uuid.uuid, u)) {
		if (!memcmp(rs_uuid.val, data->data, 16)) {	/* TODO: Check that this will work as expected! */
			*found = true;
			/* return false to indicate that no more advertisement data should be parsed */
			return false;
		}
	} else if (data->type == BT_DATA_UUID128_SOME) {
		/* Could add code here for if there is a circumstance when more than 1 uuid is advertised */
		LOG_INF("Unexpected. More than one 128 bit uuid observed.");
	}
	return true;
}

/** @brief Callback used with bt_data_parse to parse the data from the advertisement of a bluetooth device 
 * which has already been established that it is an anchor.
 *
 *  @param data Data to parse
 *  @param user_data Pointer to an anchor struct.
 *
 *  @return true
 */
static bool parse_anchor_data(struct bt_data *data, void *user_data)
{
	struct anchor *a = user_data;

	switch (data->type) {
		/* Add more cases as they become relevant. */
	case BT_DATA_MANUFACTURER_DATA:
		ARG_UNUSED(a);
		/* TODO: this should hold a struct that contains the gps coordinates. 
		 Yet to be implemented on anchor appilation side. */
	}
	return true;
}

/** @brief Check if an anchor is stored in a list of anchors by using addr.
 *
 *  @param addr Anchor device address to check
 *  @param a_list List of stored known anchors
 *  @param len Length of anchor list
 *
 *  @return list index of anchor if @a addr is in @a a_list, else -1
 */
static int anchor_is_known(const bt_addr_le_t *addr, struct anchor *a_list, int len)
{
	for (int i = 0; i < len; i++) {
		if (bt_addr_le_eq(addr, &a_list[i].addr)) {
			return i;
		}
	}
	return -1;
}

/** @brief Function to find where a new anchor should be placed within the list of known anchors. This
 * will either be an empty slot or will be to replace an anchor with worse rssi or a very long unheard 
 * from time. If the new device has bad rssi, then there is a chance that it is rejected.
 *
 *  @param rssi RSSI of the new device
 *  @param a_list List of stored known anchors
 *  @param len Length of anchor list
 *
 *  @return a value from 0 to @a len - 1 if a slot is allocated, else -1
 */
static int get_new_anchor_index(int8_t rssi, struct anchor *a_list, int len)
{
	/* TODO: Add consideration for the last_heard value so that anchors that have not been heard from
	get dropped. */
	int candidate_index = -1;
	int8_t worst_rssi = 127;

	for (int i = 0; i < len; i++) {
		/* check if the slot in the list is empty */
		if (!a_list[i].exists) {
			return i;
		}

		if (a_list[i].rssi < worst_rssi) {
			candidate_index = i;
			worst_rssi = a_list[i].rssi;
		}
	}

	if (worst_rssi > rssi) {
		/* Reject the new device because it has a worse rssi */
		return -1;
	}
	return candidate_index;
}

static void device_found(const bt_addr_le_t *addr, int8_t rssi, uint8_t type, struct net_buf_simple *ad)
{
	char addr_str[BT_ADDR_LE_STR_LEN];
	bool anchor_found = false;

	/* Only interested in connectable events */
	if (type != BT_GAP_ADV_TYPE_ADV_IND &&
	    type != BT_GAP_ADV_TYPE_ADV_DIRECT_IND) {
		return;
	}

	/* Check that the device is an anchor */
	bt_data_parse(ad, is_anchor, &anchor_found);
	if (!anchor_found) {
		/* Device found was not an anchor. */
		return;
	}

	bt_addr_le_to_str(addr, addr_str, sizeof(addr_str));
	LOG_INF("Anchor found. Addr: %s", addr_str);

	switch (app_state) {
	case IDLE:
		/* what are you doing here?? */
		return;
	case SHORT_SCAN:
		/* Apply the heuristic to the anchor */
		/* Add to short list if it is better than existing anchors or list is not full */
		/* Update the anchor if it is already in the list */
		break;
	case LONG_SCAN:

		int ind = anchor_is_known(addr, anchor_list, ANCHOR_LIST_LEN);
		if (ind >= 0) {
			/* Update the parameters: rssi, last heard */
			anchor_list[ind].rssi = rssi;
			/* Apply the heuristic to the device, add to the list and sort. */
		} else {
			int ind = get_new_anchor_index(rssi, anchor_list, ANCHOR_LIST_LEN);

			if (ind < 0) {
				/* Reject the anchor */
				break;
			}
			
			bt_addr_le_copy(&anchor_list[ind].addr, addr);
			anchor_list[ind].rssi = rssi;
			anchor_list[ind].exists = true;
			bt_data_parse(ad, parse_anchor_data, &anchor_list[ind]);
		}
		break;
	case RANGING:
		/* Placeholder. Scanning should be disabled during connection attempts but there
		may be a use for starting a short scan during ranging. */
		break;
	}

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
	 */
}

static void long_scan_handler(struct k_work *work)
{
	int err;

	if (app_state != IDLE) {
		/* Another scan or ranging is taking place. Reschedule.*/
		k_work_reschedule(&long_scan_work, K_MSEC(1187));
		return;
	}

	app_state = LONG_SCAN;
	err = bt_le_scan_start(BT_LE_SCAN_ACTIVE, device_found);
	if (err) {
		LOG_ERR("Long Scan failed to start (err %d)", err);
		app_state = IDLE;
	} else {
		LOG_INF("Start Long scanning");
		k_work_reschedule(&stop_long_scan_work, K_MSEC(LONG_SCAN_DURATION_MS));
	}
	k_work_reschedule(&long_scan_work, K_MSEC(LONG_SCAN_INTERVAL_MS));
}

static void stop_long_scan_handler(struct k_work *work)
{
	int err;

	if (app_state != LONG_SCAN) {
		LOG_ERR("Unexpected state: Should be LONG_SCAN but isnt");
	}

	err = bt_le_scan_stop();
	if (err) {
		LOG_ERR("Couldnt stop Long Scan");
		k_work_reschedule(&stop_long_scan_work, K_MSEC(1000));
	} else {
		app_state = IDLE;
	}
}

static void short_scan_handler(struct k_work *work)
{
	int err;

	if (app_state != IDLE) {
		/* Another scan or ranging is taking place. Reschedule.*/
		k_work_reschedule(&short_scan_work, K_MSEC(1187));
		return;
	}

	app_state = SHORT_SCAN;
	err = bt_le_scan_start(BT_LE_SCAN_PASSIVE, device_found);
	if (err) {
		LOG_ERR("Short Scan failed to start (err %d)", err);
		app_state = IDLE;
	} else {
		LOG_INF("Start Short scanning");
		k_work_reschedule(&stop_short_scan_work, K_MSEC(SHORT_SCAN_DURATION_MS));
	}
	k_work_reschedule(&short_scan_work, K_MSEC(SHORT_SCAN_INTERVAL_MS));
}

static void stop_short_scan_handler(struct k_work *work)
{
	int err;

	if (app_state != SHORT_SCAN) {
		LOG_ERR("Unexpected state: Should be SHORT_SCAN but isnt");
	}

	err = bt_le_scan_stop();
	if (err) {
		LOG_ERR("Couldnt stop Short Scan");
		k_work_reschedule(&stop_short_scan_work, K_MSEC(1000));
	} else {
		app_state = IDLE;
	}
}

static void do_ranging_handler(struct k_work *work)
{
	int err;
	/* Reschedule the next ranging event regardless of current system state */
	k_work_schedule(&do_ranging_work, K_MSEC(RANGING_INTERVAL_MS));

	if (app_state != IDLE) {
		/* Scanning is taking place. Do nothing.*/
		return;
	}

	app_state = RANGING;

	/* Attempt to connect to each of the devices in the shortlist in series until the
	 * number of successfull range attempts reaches 3 or 4, or untill the end of the 
	 * list is reached. */

	/* Currently using the main list of anchors, not the ranked shortlist of anchors */

	for (int i = 0; i < ANCHOR_LIST_LEN; i++) {
		/* check if the anchor exists field is true. */
		if (!anchor_list[i].exists) {
			continue;
		}
		
		/* 1. Start a connection */
		err = bt_conn_le_create(&anchor_list[i].addr, conn_create_param, BT_LE_CONN_PARAM_DEFAULT, &curr_conn);
		if (err) {
			char addr_str[BT_ADDR_LE_STR_LEN];
			bt_addr_le_to_str(&anchor_list[i].addr, addr_str, sizeof(addr_str));
			printk("Create conn to %s failed (%d). %s\n", addr_str, err, bt_hci_err_to_str(err));
			break;
		}
		/* Wait until the device is connected */
		// printk("Connection Status: %s\n", is_device_connected() == true ? "connected" : "disconnected");
		err = k_poll(conn_event, 1, K_FOREVER);
		// k_poll_signal_check(&conn_sig, &signaled, &result);
		k_poll_signal_reset(&conn_sig);
		conn_event[0].state = K_POLL_STATE_NOT_READY;
		// printk("Connection Status: %s\n", is_device_connected() == true ? "connected" : "disconnected");

		for (int j = 0; j < MAX_RANGING_ATTEMPTS; j++) {
			/* Write to the counter characteristic */
			
			/* Write to the uwb counter */
			/* Send a ranging command */
			/* Read from the timestamp characteristic */
		}
		k_msleep(1000);

		bt_conn_disconnect(curr_conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);

		// printk("Attempt disconnect\n");
		// err = k_poll(conn_event, 1, K_FOREVER);
		// k_poll_signal_check(&conn_sig, &signaled, &result);
		// k_poll_signal_reset(&conn_sig);
		// conn_event[0].state = K_POLL_STATE_NOT_READY;

		/* Wait until the device has disconnected. */
		// while(is_device_connected()) {};
	}

	LOG_INF("Ranging attempted");
	app_state = IDLE;
}

static void gatt_write_cb(struct bt_conn *conn, uint8_t err, struct bt_gatt_write_params *params)
{
	// LOG_INF("Write complete. Handle = %d", params->handle); // Is it really complete here?

	/* Send signal to indicate range command has been sent */
	/* TODO: remove hardcoded reference to range command handle */
	if (err) {
		LOG_ERR("Gatt write failed. %s", bt_gatt_err_to_str(err));
	} else {
		LOG_INF("Gatt write succeeded");
	}
	if (params->handle == 24U) {
		// k_poll_signal_raise(&write_complete_sig,1);
	}

	k_poll_signal_raise(&write_complete_sig,1);
}

static uint8_t gatt_read_cb(struct bt_conn *conn, uint8_t err, struct bt_gatt_read_params *params,
			    const void *data, uint16_t length)
{
	if (err) {
		LOG_ERR("Gatt read failed. %s", bt_gatt_err_to_str(err));
	}
	/* TODO: parse the data here */
	LOG_INF("Read complete");
	return BT_GATT_ITER_STOP;
}
static uint8_t notify_func(struct bt_conn *conn, struct bt_gatt_subscribe_params *params,
			   const void *data, uint16_t length)
{
	if (!data) {
		printk("[UNSUBSCRIBED]\n");
		params->value_handle = 0U;
		return BT_GATT_ITER_STOP;
	}

	printk("[NOTIFICATION] data %p length %u\n", data, length);

	return BT_GATT_ITER_CONTINUE;
}
static void do_ranging_thread(void *arg1, void *arg2, void *arg3)
{
	int err;
	/* Reschedule the next ranging event regardless of current system state */
	struct bt_gatt_write_params write_params = {
		.func = gatt_write_cb,
		.handle = 0, 	// Temporary handle. To be changed when addresssing a specific characteristic
		.offset = 0, 	// ???
		.data = NULL, 	// Change to a pointer to data when sending
		.length = 0	// Length of data being sent (bytes?)
	};

	struct bt_gatt_read_params read_params = {
		.func = gatt_read_cb,
		.handle_count = 1,	// Only one characteristic will be read at this stage
		.single.handle = 0,	// Temporary handle. To be changed when addresssing a specific characteristic
		.single.offset = 0
	};

	uint32_t uwb_counter = 0;
	uint8_t rng_cmd = 0x01; // TODO: Change this to a defined constant in the rs.h file

	/* Initiate the key and counter on the dw3000 */

	while (1) {
		/* TODO: decide whether to skip the ranging attempt or wait until scanning is complete. */
		if (app_state == IDLE) {

			app_state = RANGING;

			/* Attempt to connect to each of the devices in the shortlist in series until the
			* number of successfull range attempts reaches 3 or 4, or untill the end of the 
			* list is reached. */

			/* Currently using the main list of anchors, not the ranked shortlist of anchors */

			for (int i = 0; i < ANCHOR_LIST_LEN; i++) {
				/* check if the anchor exists field is true. */
				if (!anchor_list[i].exists) {
					continue;
				}
				
				/* 1. Start a connection */
				err = bt_conn_le_create(&anchor_list[i].addr, conn_create_param, BT_LE_CONN_PARAM_DEFAULT, &curr_conn);
				if (err) {
					char addr_str[BT_ADDR_LE_STR_LEN];
					bt_addr_le_to_str(&anchor_list[i].addr, addr_str, sizeof(addr_str));
					printk("Create conn to %s failed (%d). %s\n", addr_str, err, bt_hci_err_to_str(err));
					break;
				}
				/* Wait until the device is connected */
				err = k_poll(conn_event, 1, K_FOREVER); // TODO: add a timeout and a return value to indicate connection success
				// k_poll_signal_check(&conn_sig, &signaled, &result);
				k_poll_signal_reset(&conn_sig);
				conn_event[0].state = K_POLL_STATE_NOT_READY;

				for (int j = 0; j < MAX_RANGING_ATTEMPTS; j++) {
					/* Create new counter and synchronise between devices */
					uwb_counter = 0x12345678;
					dw3xxx_update_sts_counter(uwb, uwb_counter);
					uwb_counter = sys_cpu_to_be32(uwb_counter);

					/* TODO: Create more than one write param instead of waiting for write complete signal */
					/* Write to the counter characteristic. Attribute handle = 18 (check this with discovery) */
					write_params.handle = 18U;
					write_params.data = &uwb_counter;
					write_params.length = sizeof(uwb_counter);
					bt_gatt_write(curr_conn, &write_params);
					/* Wait for write to complete */
					k_poll(rng_event, 1, K_FOREVER); 
					k_poll_signal_reset(&write_complete_sig);
					rng_event[0].state = K_POLL_STATE_NOT_READY;
					
					/* Send a ranging command. Attribute Handle = 27 */
					write_params.handle = 27U;
					write_params.data = &rng_cmd;
					write_params.length = sizeof(rng_cmd);
					bt_gatt_write(curr_conn, &write_params);
					/* Wait for write to complete */
					err = k_poll(rng_event, 1, K_FOREVER); // TODO: add timeout and error handling
					k_poll_signal_reset(&write_complete_sig);
					rng_event[0].state = K_POLL_STATE_NOT_READY;

					k_msleep(1);
					/* DW3000 Run initiator after command has been sent */
					run_initiator(uwb);

					/* Sleep until ranging should be complete */
					// k_msleep(2);

					/* Read from the timestamp characteristic. Attribute handle = 24 */
					// read_params.single.handle = 24U;
					// bt_gatt_read(curr_conn, &read_params);

					break; // Temporary break. Add check to see if ranging was successful.
				}
				k_msleep(5000);
				bt_conn_disconnect(curr_conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);

				/* Wait until the device has disconnected. */
				err = k_poll(conn_event, 1, K_FOREVER);
				// k_poll_signal_check(&conn_sig, &signaled, &result);
				k_poll_signal_reset(&conn_sig);
				conn_event[0].state = K_POLL_STATE_NOT_READY;
			}

			LOG_INF("Ranging attempted");
			app_state = IDLE;
		}
		k_msleep(RANGING_INTERVAL_MS);
	}
}

static struct bt_gatt_discover_params discover_params;
static struct bt_gatt_subscribe_params subscribe_params;

static uint8_t discover_func(struct bt_conn *conn,
			     const struct bt_gatt_attr *attr,
			     struct bt_gatt_discover_params *params)
{
	int err;

	if (!attr) {
		printk("Discover complete\n");
		(void)memset(params, 0, sizeof(*params));
		return BT_GATT_ITER_STOP;
	}

	// printk("[ATTRIBUTE] handle %u\n", attr->handle);

	if (!bt_uuid_cmp(discover_params.uuid, BT_UUID_RS_COUNTER_CHAR)) {
		LOG_INF("Counter characteristic ATTR val handle:  %u", bt_gatt_attr_value_handle(attr));
		discover_params.uuid = &key_char_uuid.uuid;
		discover_params.start_handle = attr->handle + 1;
		err = bt_gatt_discover(conn, &discover_params);
		if (err) {
			printk("Discover failed (err %d)\n", err);
		}
	} else if (!bt_uuid_cmp(discover_params.uuid, BT_UUID_RS_KEY_CHAR)) {
		LOG_INF("Key characteristic ATTR val handle: %u", bt_gatt_attr_value_handle(attr));
		discover_params.uuid = &iv_char_uuid.uuid;
		discover_params.start_handle = attr->handle + 1;
		err = bt_gatt_discover(conn, &discover_params);
		if (err) {
			printk("Discover failed (err %d)\n", err);
		}
	} else if (!bt_uuid_cmp(discover_params.uuid, BT_UUID_RS_IV_CHAR)) {
		LOG_INF("IV characteristic ATTR val handle: %u", bt_gatt_attr_value_handle(attr));
		discover_params.uuid = &ts_char_uuid.uuid;
		discover_params.start_handle = attr->handle + 1;
		err = bt_gatt_discover(conn, &discover_params);
		if (err) {
			printk("Discover failed (err %d)\n", err);
		}
	} else if (!bt_uuid_cmp(discover_params.uuid, BT_UUID_RS_TS_CHAR)) {
		LOG_INF("Timestamp characteristic ATTR val handle:%u", bt_gatt_attr_value_handle(attr));
		discover_params.uuid = &ccc_char_uuid.uuid;
		discover_params.start_handle = attr->handle + 1;
		discover_params.type = BT_GATT_DISCOVER_DESCRIPTOR;
		err = bt_gatt_discover(conn, &discover_params);
		if (err) {
			printk("Discover failed (err %d)\n", err);
		}
	} else if (!bt_uuid_cmp(discover_params.uuid, BT_UUID_GATT_CCC)) {
		LOG_INF("GATT CCC ATTR handle: %u", attr->handle);
		discover_params.uuid = &rc_char_uuid.uuid;
		discover_params.type = BT_GATT_DISCOVER_CHARACTERISTIC;
		err = bt_gatt_discover(conn, &discover_params);
		if (err) {
			printk("Discover failed (err %d)\n", err);
		}
		
	} else if (!bt_uuid_cmp(discover_params.uuid, BT_UUID_RS_RNGCMD_CHAR)) {
		LOG_INF("Range Command characteristic ATTR val handle: %u", bt_gatt_attr_value_handle(attr));
		// discover_params.uuid = &ccc_char_uuid.uuid;
		// discover_params.start_handle = attr->handle + 1;
		// discover_params.type = BT_GATT_DISCOVER_DESCRIPTOR;
		// err = bt_gatt_discover(conn, &discover_params);
		// if (err) {
		// 	printk("Discover failed (err %d)\n", err);
		// }
		return BT_GATT_ITER_STOP;
	} else {
		return BT_GATT_ITER_CONTINUE;
	}
	return BT_GATT_ITER_STOP;
}

static void mtu_exchange_cb(struct bt_conn *conn, uint8_t err,
			    struct bt_gatt_exchange_params *params)
{
	printk("%s: MTU exchange %s (%u)\n", __func__,
	       err == 0U ? "successful" : "failed",
	       bt_gatt_get_mtu(conn));
}

static struct bt_gatt_exchange_params mtu_exchange_params = {
	.func = mtu_exchange_cb
};

static int mtu_exchange(struct bt_conn *conn)
{
	int err;

	printk("%s: Current MTU = %u\n", __func__, bt_gatt_get_mtu(conn));

	printk("%s: Exchange MTU...\n", __func__);
	err = bt_gatt_exchange_mtu(conn, &mtu_exchange_params);
	if (err) {
		printk("%s: MTU exchange failed (err %d)", __func__, err);
	}

	return err;
}
static void connected(struct bt_conn *conn, uint8_t err)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	if (err) {
		LOG_ERR("Failed to connect to %s %u %s", addr, err, bt_hci_err_to_str(err));

		bt_conn_unref(curr_conn);
		curr_conn = NULL;
		return;
	}

	if (conn != curr_conn) {
		LOG_ERR("What funky stuff went down here?");
		return;
	}
	atomic_set(&CONNECTED, 1);
	(void)mtu_exchange(conn);
	LOG_INF("Connected: %s", addr);

	/* TODO: Implement a cached handle system where every time a new device is connected to, the handles of 
	each gatt attribute is stored. This discovery connection should be done prior to a ranging connection to 
	minimise latency, not necessary but a nice to have. */

	/* Discovery */
	// discover_params.uuid = &count_char_uuid.uuid;
	// discover_params.func = discover_func;
	// discover_params.start_handle = BT_ATT_FIRST_ATTRIBUTE_HANDLE;
	// discover_params.end_handle = BT_ATT_LAST_ATTRIBUTE_HANDLE;
	// discover_params.type = BT_GATT_DISCOVER_CHARACTERISTIC;

	// err = bt_gatt_discover(conn, &discover_params);
	// if (err) {
	// 	printk("Discover failed(err %d)\n", err);
	// 	return;
	// }

	/* Subscribe to the read value. */
	subscribe_params.value_handle = 24U;
	subscribe_params.notify = notify_func;
	subscribe_params.value = BT_GATT_CCC_INDICATE;
	subscribe_params.ccc_handle = 25U;

	err = bt_gatt_subscribe(conn, &subscribe_params);
	if (err) {
		if (err == -EALREADY) {
			printk("[ALREADY SUBSCRIBED]\n");
		} else {
			printk("Subscribe failed (err %d)\n", err);
		}
	} else {
		printk("[SUBSCRIBED]\n");
	}

	k_poll_signal_raise(&conn_sig,1);
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	char addr[BT_ADDR_LE_STR_LEN];

	if (conn != curr_conn) {
		return;
	}

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	printk("Disconnected: %s, reason 0x%02x %s\n", addr, reason, bt_hci_err_to_str(reason));

	bt_conn_unref(curr_conn);
	curr_conn = NULL;
	atomic_set(&CONNECTED, 0);
	k_poll_signal_raise(&conn_sig,1);
}

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
		return 0;
	}
	/* Register connection callbacks */
	bt_conn_cb_register(&connection_callbacks);

	/* Configure UWB chip and driver */
	dw3xxx_configure_device(uwb, RANGING_INITIATOR, HRP_UWB_PHY_CHANNEL_9);
	dw_enable_irq(uwb);

	/* TODO: Change this so that the disconnect has a dedicated signal. */
	k_poll_signal_init(&conn_sig);
	k_poll_signal_init(&write_complete_sig);
	k_poll_event_init(&conn_event[0], K_POLL_TYPE_SIGNAL, K_POLL_MODE_NOTIFY_ONLY, &conn_sig);
	k_poll_event_init(&rng_event[0], K_POLL_TYPE_SIGNAL, K_POLL_MODE_NOTIFY_ONLY, &write_complete_sig);

	/* Start and schedule */
	k_work_schedule(&long_scan_work, K_NO_WAIT);
	k_work_schedule(&short_scan_work, K_NO_WAIT);
	// k_work_schedule(&do_ranging_work, K_MSEC(RANGING_INTERVAL_MS));

	return 0;
}
