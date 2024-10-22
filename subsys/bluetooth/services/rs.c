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
static uint32_t sts_key[STS_UINT_LEN];
static uint32_t sts_iv[STS_UINT_LEN]; /* First 12 bytes for IV, last 4 bytes for counter */

/* TODO: revisit the implementation of the Key and IV. A union useful for dealing with the
bytes from the bluetooth and Kconfig side and uint32_t from the application side. */
union {
	dwt_sts_cp_key_t a;
	uint32_t b[4];
} sts_key_things;



typedef void (*bt_rs_range_cb_t)(void);
static const struct bt_rs_cb *rs_cb;
 
/* UUID declarations */
#define BT_UUID_RS_VAL			BT_UUID_128_ENCODE(0x49aa9800, 0x34c9, 0x40ca, 0x95db, 0x4eea1a31a229)
#define BT_UUID_RS			BT_UUID_DECLARE_128(BT_UUID_RS_VAL)
 
#define BT_UUID_RS_COUNTER_CHAR_VAL	BT_UUID_128_ENCODE(0x49aa9801, 0x34c9, 0x40ca, 0x95db, 0x4eea1a31a229)
#define BT_UUID_RS_COUNTER_CHAR		BT_UUID_DECLARE_128(BT_UUID_RS_COUNTER_CHAR_VAL)
 
#define BT_UUID_RS_KEY_CHAR_VAL		BT_UUID_128_ENCODE(0x49aa9802, 0x34c9, 0x40ca, 0x95db, 0x4eea1a31a229)
#define BT_UUID_RS_KEY_CHAR		BT_UUID_DECLARE_128(BT_UUID_RS_KEY_CHAR_VAL)
 
#define BT_UUID_RS_IV_CHAR_VAL		BT_UUID_128_ENCODE(0x49aa9803, 0x34c9, 0x40ca, 0x95db, 0x4eea1a31a229)
#define BT_UUID_RS_IV_CHAR		BT_UUID_DECLARE_128(BT_UUID_RS_IV_CHAR_VAL)

#define BT_UUID_RS_RANGE_CHAR_VAL	BT_UUID_128_ENCODE(0x49aa9804, 0x34c9, 0x40ca, 0x95db, 0x4eea1a31a229)
#define BT_UUID_RS_RANGE_CHAR		BT_UUID_DECLARE_128(BT_UUID_RS_RANGE_CHAR_VAL)
 
/* Read and write functions for the counter */
static ssize_t read_counter(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf,
			    uint16_t len, uint16_t offset)
{
	uint32_t counter_be = sys_cpu_to_be32(sts_iv[3]);
	return bt_gatt_attr_read(conn, attr, buf, len, offset, &counter_be, sizeof(counter_be));
}
 
static ssize_t write_counter(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf,
			     uint16_t len, uint16_t offset, uint8_t flags)
{
	if (offset != 0 || len != sizeof(uint32_t)) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}
	sts_iv[3] = sys_be32_to_cpu(*(uint32_t *)buf);
	if (rs_cb->iv_changed) {
		rs_cb->iv_changed();
	}
	return len;
}
 
/* Read and write functions for the key */
static ssize_t read_key(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf,
                        uint16_t len, uint16_t offset)
{
	uint8_t key_bytes[16];
	for (int i = 0; i < 4; i++) {
		sys_put_be32(sts_key[i], &key_bytes[i*4]);
	}
	return bt_gatt_attr_read(conn, attr, buf, len, offset, key_bytes, sizeof(key_bytes));
}
 
static ssize_t write_key(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf,
                         uint16_t len, uint16_t offset, uint8_t flags)
{
	if (offset != 0 || len != sizeof(sts_key)) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}
	const uint8_t *key_bytes = buf;
	for (int i = 0; i < 4; i++) {
		sts_key[i] = sys_get_be32(&key_bytes[i*4]);
	}
	if (rs_cb->key_changed) {
		rs_cb->key_changed();
	}
	return len;
}
 
/* Read and write functions for the IV */
static ssize_t read_iv(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf,
                       uint16_t len, uint16_t offset)
{
	uint8_t iv_bytes[16];
	for (int i = 0; i < 4; i++) {
		sys_put_be32(sts_iv[i], &iv_bytes[i*4]);
	}
	return bt_gatt_attr_read(conn, attr, buf, len, offset, iv_bytes, sizeof(iv_bytes));
}
 
static ssize_t write_iv(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf,
                        uint16_t len, uint16_t offset, uint8_t flags)
{
	if (offset != 0 || len != sizeof(sts_iv)) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}
	const uint8_t *iv_bytes = buf;
	for (int i = 0; i < 4; i++) {
		sts_iv[i] = sys_get_be32(&iv_bytes[i*4]);
	}
	if (rs_cb->iv_changed) {
		rs_cb->iv_changed();
	}
	return len;
}

/* Write function for start ranging */
static ssize_t write_ranging(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf,
                        uint16_t len, uint16_t offset, uint8_t flags)
{
	if (offset != 0 || len != 1) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}

	const uint8_t *value = buf;

	if (*value  == 0x01) {
		if (rs_cb->start_ranging) {
			rs_cb->start_ranging();
		}
	} else {
		LOG_WRN("Invalid  value for ranging characteristic");
	}
	return len;
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
	/* IV characteristic */
	BT_GATT_CHARACTERISTIC(BT_UUID_RS_RANGE_CHAR,
				BT_GATT_CHRC_WRITE,
				BT_GATT_PERM_WRITE,
				NULL, write_ranging, NULL),
);
 
/* Get and set functions for key, IV, and counter */
void bt_rs_set_key(const uint32_t *key)
{
	memcpy(sts_key, key, 4 * sizeof(uint32_t));
}
 
void bt_rs_get_key(uint32_t *key)
{
	memcpy(key, sts_key, 4 * sizeof(uint32_t));
}
 
void bt_rs_set_iv_upper96(const uint32_t *iv)
{
	memcpy(sts_iv, iv, 3 * sizeof(uint32_t));
}
 
void bt_rs_get_iv(uint32_t *iv)
{
	memcpy(iv, sts_iv, 4 * sizeof(uint32_t));
}
 
void bt_rs_set_iv_counter(uint32_t counter)
{
	sts_iv[3] = counter;
}
 
uint32_t bt_rs_get_iv_counter(void)
{
	return sts_iv[3];
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
	uint8_t key_bytes[16];
    	if (hex2bin(key_str, key_len, key_bytes, sizeof(key_bytes)) != sizeof(key_bytes)) {
    	    	LOG_ERR("Failed to parse key from CONFIG_BT_RS_KEY128");
    	    	return -EINVAL;
    	}
	for (int i = 0; i < 4; i++) {
		sts_key[i] = sys_get_be32(&key_bytes[i*4]);
	}
	
    	/* Parse the IV from CONFIG_BT_RS_IV_UPPER96 */
    	const char *iv_str = CONFIG_BT_RS_IV_UPPER96;
    	size_t iv_len = strlen(iv_str);
    	if (iv_len != 24) {
		LOG_ERR("Invalid IV length in CONFIG_BT_RS_IV_UPPER96");
		return -EINVAL;
    	}
	uint8_t iv_bytes[12];
    	if (hex2bin(iv_str, iv_len, iv_bytes, sizeof(iv_bytes)) != sizeof(iv_bytes)) {
		LOG_ERR("Failed to parse IV from CONFIG_BT_RS_IV_UPPER96");
		return -EINVAL;
    	}
	for (int i = 0; i < 3; i++) {
		sts_iv[i] = sys_get_be32(&iv_bytes[i*4]);
	}
	
    	/* Get the counter from CONFIG_BT_RS_IV_COUNT32 */
    	sts_iv[3] = CONFIG_BT_RS_IV_COUNT32;
    	return 0;
}
 
SYS_INIT(bt_rs_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);