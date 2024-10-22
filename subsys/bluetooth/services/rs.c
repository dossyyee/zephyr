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
 
#define LOG_LEVEL CONFIG_BT_SERVICE_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(bt_rs);
 
/* STS Key and IV (Key: 128 bits, IV: 96 bits + 32-bit counter) */
static uint8_t sts_key[16];
static uint8_t sts_iv[16]; /* First 12 bytes for IV, last 4 bytes for counter */
 
/* UUID declarations */
#define BT_UUID_RS_VAL			BT_UUID_128_ENCODE(0x49aa9800, 0x34c9, 0x40ca, 0x95db, 0x4eea1a31a229)
#define BT_UUID_RS			BT_UUID_DECLARE_128(BT_UUID_RS_VAL)
 
#define BT_UUID_RS_COUNTER_CHAR_VAL	BT_UUID_128_ENCODE(0x49aa9801, 0x34c9, 0x40ca, 0x95db, 0x4eea1a31a229)
#define BT_UUID_RS_COUNTER_CHAR		BT_UUID_DECLARE_128(BT_UUID_RS_COUNTER_CHAR_VAL)
 
#define BT_UUID_RS_KEY_CHAR_VAL		BT_UUID_128_ENCODE(0x49aa9802, 0x34c9, 0x40ca, 0x95db, 0x4eea1a31a229)
#define BT_UUID_RS_KEY_CHAR		BT_UUID_DECLARE_128(BT_UUID_RS_KEY_CHAR_VAL)
 
#define BT_UUID_RS_IV_CHAR_VAL		BT_UUID_128_ENCODE(0x49aa9803, 0x34c9, 0x40ca, 0x95db, 0x4eea1a31a229)
#define BT_UUID_RS_IV_CHAR		BT_UUID_DECLARE_128(BT_UUID_RS_IV_CHAR_VAL)
 
/* Read and write functions for the counter */
static ssize_t read_counter(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf,
			    uint16_t len, uint16_t offset)
{
	const uint8_t *counter = attr->user_data;
	uint32_t counter_le = sys_get_le32(counter);
	return bt_gatt_attr_read(conn, attr, buf, len, offset, &counter_le, sizeof(counter_le));
}
 
static ssize_t write_counter(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf,
			     uint16_t len, uint16_t offset, uint8_t flags)
{
	uint8_t *counter = attr->user_data;
	if (offset != 0 || len != sizeof(uint32_t)) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}
	memcpy(counter, buf, sizeof(uint32_t));
	return len;
}
 
/* Read and write functions for the key */
static ssize_t read_key(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf,
                        uint16_t len, uint16_t offset)
{
	const uint8_t *key = attr->user_data;
	return bt_gatt_attr_read(conn, attr, buf, len, offset, key, sizeof(sts_key));
}
 
static ssize_t write_key(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf,
                         uint16_t len, uint16_t offset, uint8_t flags)
{
	uint8_t *key = attr->user_data;
	if (offset != 0 || len != sizeof(sts_key)) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}
	memcpy(key, buf, sizeof(sts_key));
	return len;
}
 
/* Read and write functions for the IV */
static ssize_t read_iv(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf,
                       uint16_t len, uint16_t offset)
{
	const uint8_t *iv = attr->user_data;
	return bt_gatt_attr_read(conn, attr, buf, len, offset, iv, 12); /* Only upper 96 bits */
}
 
static ssize_t write_iv(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf,
                        uint16_t len, uint16_t offset, uint8_t flags)
{
	uint8_t *iv = attr->user_data;
	if (offset != 0 || len != 12) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}
	memcpy(iv, buf, 12);
	return len;
}
 
/* Ranging Service Declaration */
BT_GATT_SERVICE_DEFINE(
    rs_svc, BT_GATT_PRIMARY_SERVICE(BT_UUID_RS),
    /* 32-bit counter characteristic */
    BT_GATT_CHARACTERISTIC(BT_UUID_RS_COUNTER_CHAR,
			   BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
			   BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
			   read_counter, write_counter, &sts_iv[12]),
    /* Key characteristic */
    BT_GATT_CHARACTERISTIC(BT_UUID_RS_KEY_CHAR,
			   BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
			   BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
			   read_key, write_key, sts_key),
    /* IV characteristic */
    BT_GATT_CHARACTERISTIC(BT_UUID_RS_IV_CHAR,
			   BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
			   BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
			   read_iv, write_iv, sts_iv),
);
 
/* Get and set functions for key, IV, and counter */
void bt_rs_set_key(const uint8_t *key)
{
	memcpy(sts_key, key, sizeof(sts_key));
}
 
void bt_rs_get_key(uint8_t *key)
{
	memcpy(key, sts_key, sizeof(sts_key));
}
 
void bt_rs_set_iv(const uint8_t *iv)
{
	memcpy(sts_iv, iv, 12);
}
 
void bt_rs_get_iv(uint8_t *iv)
{
	memcpy(iv, sts_iv, 12);
}
 
void bt_rs_set_counter(uint32_t counter)
{
	sys_put_le32(counter, &sts_iv[12]);
}
 
uint32_t bt_rs_get_counter(void)
{
	return sys_get_le32(&sts_iv[12]);
}
 
/* Initialize the STS key and IV with values from Kconfig */
static int bt_rs_init()
{
    	// ARG_UNUSED(dev);
	
    	/* Parse the key from CONFIG_BT_RS_KEY128 */
    	const char *key_str = CONFIG_BT_RS_KEY128;
    	size_t key_len = strlen(key_str);
    	if (key_len != 32) {
    	    	LOG_ERR("Invalid key length in CONFIG_BT_RS_KEY128");
    	    	return -EINVAL;
    	}
    	if (hex2bin(key_str, key_len, sts_key, sizeof(sts_key)) != sizeof(sts_key)) {
    	    	LOG_ERR("Failed to parse key from CONFIG_BT_RS_KEY128");
    	    	return -EINVAL;
    	}
	
    	/* Parse the IV from CONFIG_BT_RS_IV_UPPER96 */
    	const char *iv_str = CONFIG_BT_RS_IV_UPPER96;
    	size_t iv_len = strlen(iv_str);
    	if (iv_len != 24) {
		LOG_ERR("Invalid IV length in CONFIG_BT_RS_IV_UPPER96");
		return -EINVAL;
    	}
    	if (hex2bin(iv_str, iv_len, sts_iv, 12) != 12) {
		LOG_ERR("Failed to parse IV from CONFIG_BT_RS_IV_UPPER96");
		return -EINVAL;
    	}
	
    	/* Get the counter from CONFIG_BT_RS_IV_COUNT32 */
    	uint32_t counter = CONFIG_BT_RS_IV_COUNT32;
    	sys_put_le32(counter, &sts_iv[12]);
	
    	return 0;
}
 
SYS_INIT(bt_rs_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);