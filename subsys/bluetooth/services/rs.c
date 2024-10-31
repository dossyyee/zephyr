/** @file
 *  @brief GATT Ranging Service
 */

#include <zephyr/types.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <zephyr/kernel.h>
#include <zephyr/init.h>
 
#include <zephyr/settings/settings.h>
 
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
 
#include <zephyr/sys/byteorder.h>
#include <zephyr/bluetooth/byteorder.h>
#include <zephyr/sys/util.h>  /* For hex2bin */

#include <zephyr/bluetooth/services/rs.h>
#include <zephyr/drivers/ieee802154/deca_device_api.h>
 
#define LOG_LEVEL CONFIG_BT_SERVICE_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(bt_rs);
 
/* STS Key and IV (Key: 128 bits, IV: 96 bits + 32-bit counter) */
// static uint32_t sts_key[STS_UINT_LEN];
// static uint32_t sts_iv[STS_UINT_LEN]; /* First 12 bytes for IV, last 4 bytes for counter */

union {
	uint32_t u32[4];
	uint8_t u8[16];
} sts_key, sts_iv;

/* TODO: revisit the size of the timestamp. Mathematically, only the lower digits are necessary
 * for a correct distance calculation. Therefore this can be more space efficient. */
union {
	uint64_t u64[3];
	uint8_t u8[24];
} timestamp = {.u64 = {0,0,0}};


/* callbacks */
typedef void (*bt_rs_range_cb_t)(void);
static const struct bt_rs_cb *rs_cb;

/* Wizardry */
static const struct bt_gatt_attr *timestamp_attr = NULL;
static struct bt_gatt_indicate_params ind_params;

 
/* Read and write functions for the counter */
static ssize_t read_counter(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf,
			    uint16_t len, uint16_t offset)
{
	return bt_gatt_attr_read(conn, attr, buf, len, offset, &sts_iv.u32[3], sizeof(uint32_t));
}
 
static ssize_t write_counter(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf,
			     uint16_t len, uint16_t offset, uint8_t flags)
{
	if (offset != 0 || len != sizeof(uint32_t)) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}

	const uint32_t *value = buf;
	sts_iv.u32[3] = *value;

	/* Trigger the IV changed Callback if it is implemented */
	if (rs_cb->iv_changed) {
		rs_cb->iv_changed();
	}
	return len;
}
 
/* Read and write functions for the key */
static ssize_t read_key(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf,
                        uint16_t len, uint16_t offset)
{
	return bt_gatt_attr_read(conn, attr, buf, len, offset, sts_key.u8, sizeof(sts_key));
}
 
static ssize_t write_key(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf,
                         uint16_t len, uint16_t offset, uint8_t flags)
{
	if (offset != 0 || len != sizeof(sts_key)) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}

	memcpy(sts_key.u8, (uint8_t *)buf, sizeof(sts_key));

	/* Trigger the key changed Callback if it is implemented */
	if (rs_cb->key_changed) {
		rs_cb->key_changed();
	}
	return len;
}
 
/* Read and write functions for the IV */
static ssize_t read_iv(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf,
                       uint16_t len, uint16_t offset)
{
	return bt_gatt_attr_read(conn, attr, buf, len, offset, sts_iv.u8, sizeof(sts_iv));
}
 
static ssize_t write_iv(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf,
                        uint16_t len, uint16_t offset, uint8_t flags)
{
	if (offset != 0 || len != sizeof(sts_iv)) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}

	memcpy(sts_iv.u8, (uint8_t *)buf, sizeof(sts_iv));

	/* Trigger the IV changed Callback if it is implemented */
	if (rs_cb->iv_changed) {
		rs_cb->iv_changed();
	}
	return len;
}

/* Read function for the timestamp */
static ssize_t read_ts(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf,
                       uint16_t len, uint16_t offset)
{
	return bt_gatt_attr_read(conn, attr, buf, len, offset, timestamp.u8, sizeof(timestamp));
}

/* Write function for start ranging */
static ssize_t write_range_cmd(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf,
                        uint16_t len, uint16_t offset, uint8_t flags)
{
	if (offset != 0 || len != 1) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}

	const uint8_t *value = buf;

	if (*value  == 0x01) {
		/* Start ranging if callback is implemented */
		if (rs_cb->start_ranging) {
			rs_cb->start_ranging();
		}
	} else {
		LOG_WRN("Invalid  value for ranging characteristic");
	}
	return len;
}

static void indicate_cb(struct bt_conn *conn, struct bt_gatt_indicate_params *params, uint8_t err)
{
	if (err) {
		LOG_ERR("Indication Failed: %d", err);
	} else {
		LOG_INF("Indication Successful");
	}
}

static void indicate_destroy_cb(struct bt_gatt_indicate_params *params) 
{
	return;
}

static void timestamp_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	if (value & BT_GATT_CCC_INDICATE) {
		LOG_INF("Timestamp indications enabled");
	} else {
		LOG_INF("Timestamp indications disabled");
	}
}
 
/* Ranging Service Declaration */
BT_GATT_SERVICE_DEFINE(
	rs_svc, BT_GATT_PRIMARY_SERVICE(BT_UUID_RS),
	/* 32-bit counter characteristic */
	BT_GATT_CHARACTERISTIC(BT_UUID_RS_COUNTER_CHAR,
				BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
				BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
				read_counter, write_counter, NULL),
	/* Key characteristic */
	BT_GATT_CHARACTERISTIC(BT_UUID_RS_KEY_CHAR,
				BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
				BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
				read_key, write_key, NULL),
	/* IV characteristic */
	BT_GATT_CHARACTERISTIC(BT_UUID_RS_IV_CHAR,
				BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
				BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
				read_iv, write_iv, NULL),
	/* Timestamp characteristic */
	BT_GATT_CHARACTERISTIC(BT_UUID_RS_TS_CHAR,
				BT_GATT_CHRC_READ | BT_GATT_CHRC_INDICATE,
				BT_GATT_PERM_READ,
				read_ts, NULL, NULL),
	BT_GATT_CCC(timestamp_ccc_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
	/* Ranging Command characteristic */
	BT_GATT_CHARACTERISTIC(BT_UUID_RS_RNGCMD_CHAR,
				BT_GATT_CHRC_WRITE,
				BT_GATT_PERM_WRITE,
				NULL, write_range_cmd, NULL),
);
 
/* Get and set functions for key, IV, and counter */
void bt_rs_set_key(const uint32_t *key)
{
	memcpy(sts_key.u32, key, sizeof(sts_key));
}
 
void bt_rs_get_key(uint32_t *key)
{
	memcpy(key, sts_key.u32, sizeof(sts_key));
}
 
void bt_rs_set_iv_upper96(const uint32_t *iv)
{
	memcpy(sts_iv.u32, iv, 3* sizeof(uint32_t));
}
 
void bt_rs_get_iv(uint32_t *iv)
{
	memcpy(iv, sts_iv.u32, sizeof(sts_iv));
}
 
void bt_rs_set_iv_counter(uint32_t counter)
{
	sts_iv.u32[3] = counter;
}
 
uint32_t bt_rs_get_iv_counter(void)
{
	return sts_iv.u32[3];
}

void bt_rs_set_timestamp(uint64_t *ts)
{
	memcpy(timestamp.u64, ts, sizeof(timestamp));
}

int bt_rs_indicate_timestamp(struct bt_conn *conn)
{
	// if (!bt_gatt_is_subscribed(conn, ind_params.attr, BT_GATT_CCC_INDICATE)) {
	// 	LOG_ERR("Not subscribed to indications");
	// 	return -1;
	// }

	int err = bt_gatt_indicate(NULL, &ind_params);

	if (err) {
		LOG_ERR("Failed to send indication: %s", bt_gatt_err_to_str(err));
	}
	return err;
}

int bt_rs_notify_timestamp(struct bt_conn *conn)
{
	int err = bt_gatt_notify(conn, timestamp_attr, timestamp.u8, sizeof(timestamp));
	if (err) {
		LOG_ERR("Failed to send indication: %s", bt_gatt_err_to_str(err));
	}
	return err;
}

int bt_rs_cb_init(const struct bt_rs_cb *cb)
{
	__ASSERT(cb == NULL, "Ranging service needs valid `struct bt_rs_cb` callback");
	__ASSERT(cb->start_ranging == NULL,
		 "`start_ranging` callback api is required for functioning of RS");
	if (!cb || !cb->start_ranging) {
		return -EINVAL;
	}
	rs_cb = cb;
	return 0;
}
 
/* Initialize the STS key and IV with values from Kconfig */
static int bt_rs_init()
{
    	/* Parse the key from CONFIG_BT_RS_KEY128 */
    	const char *key_str = CONFIG_BT_RS_KEY128;
    	size_t key_len = strlen(key_str);
    	if (key_len != 32) {
    	    	LOG_ERR("Invalid key length in CONFIG_BT_RS_KEY128");
    	    	return -EINVAL;
    	}
    	if (hex2bin(key_str, key_len, sts_key.u8, sizeof(sts_key)) != sizeof(sts_key)) {
    	    	LOG_ERR("Failed to parse key from CONFIG_BT_RS_KEY128");
		memset(sts_key.u8, 0x00, sizeof(sts_key));
    	    	return -EINVAL;
    	}

    	/* Parse the IV from CONFIG_BT_RS_IV_UPPER96 */
    	const char *iv_str = CONFIG_BT_RS_IV_UPPER96;
    	size_t iv_len = strlen(iv_str);
    	if (iv_len != 24) {
		LOG_ERR("Invalid IV length in CONFIG_BT_RS_IV_UPPER96");
		return -EINVAL;
    	}
    	if (hex2bin(iv_str, iv_len, sts_iv.u8, 12) != 12) {
		LOG_ERR("Failed to parse IV from CONFIG_BT_RS_IV_UPPER96");
		memset(sts_iv.u8, 0x00, 12);
		return -EINVAL;
    	}
	
    	/* Get the counter from CONFIG_BT_RS_IV_COUNT32 */
    	sts_iv.u32[3] = CONFIG_BT_RS_IV_COUNT32;

	/* Setup the timestamp indication parameters */
	for (int i = 0; i < rs_svc.attr_count; i++) {
		/* Iterate through rs_svc and find the attribute with matching UUID */
		if (!bt_uuid_cmp((&rs_svc.attrs[i])->uuid, BT_UUID_RS_TS_CHAR)) {
			timestamp_attr = &rs_svc.attrs[i];
			break;
		}
	}

	/* Set up the indication parameters for the timestamp */
	if (!timestamp_attr) {
		LOG_ERR("Failed to find Timestamp Attribute");
		return -EINVAL;
	}

	// printk("Permissions: %X\n", timestamp_attr->perm);

	ind_params.uuid = NULL;//BT_UUID_RS_TS_CHAR;
	ind_params.attr = timestamp_attr;
	ind_params.data = timestamp.u8;
	ind_params.len = sizeof(timestamp);
	ind_params.func = indicate_cb;
	ind_params.destroy = indicate_destroy_cb;

    	return 0;
}
 
SYS_INIT(bt_rs_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);