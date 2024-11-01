
#include <stdint.h>
#include <zephyr/bluetooth/uuid.h>

#define STS_U32_LEN	4

/* UUID declarations */
#define BT_UUID_RS_VAL			BT_UUID_128_ENCODE(0x49aa9800, 0x34c9, 0x40ca, 0x95db, 0x4eea1a31a229)
#define BT_UUID_RS			BT_UUID_DECLARE_128(BT_UUID_RS_VAL)
 
#define BT_UUID_RS_COUNTER_CHAR_VAL	BT_UUID_128_ENCODE(0x49aa9801, 0x34c9, 0x40ca, 0x95db, 0x4eea1a31a229)
#define BT_UUID_RS_COUNTER_CHAR		BT_UUID_DECLARE_128(BT_UUID_RS_COUNTER_CHAR_VAL)
 
#define BT_UUID_RS_KEY_CHAR_VAL		BT_UUID_128_ENCODE(0x49aa9802, 0x34c9, 0x40ca, 0x95db, 0x4eea1a31a229)
#define BT_UUID_RS_KEY_CHAR		BT_UUID_DECLARE_128(BT_UUID_RS_KEY_CHAR_VAL)
 
#define BT_UUID_RS_IV_CHAR_VAL		BT_UUID_128_ENCODE(0x49aa9803, 0x34c9, 0x40ca, 0x95db, 0x4eea1a31a229)
#define BT_UUID_RS_IV_CHAR		BT_UUID_DECLARE_128(BT_UUID_RS_IV_CHAR_VAL)

#define BT_UUID_RS_TS_CHAR_VAL		BT_UUID_128_ENCODE(0x49aa9804, 0x34c9, 0x40ca, 0x95db, 0x4eea1a31a229)
#define BT_UUID_RS_TS_CHAR		BT_UUID_DECLARE_128(BT_UUID_RS_TS_CHAR_VAL)

#define BT_UUID_RS_RNGCMD_CHAR_VAL	BT_UUID_128_ENCODE(0x49aa9805, 0x34c9, 0x40ca, 0x95db, 0x4eea1a31a229)
#define BT_UUID_RS_RNGCMD_CHAR		BT_UUID_DECLARE_128(BT_UUID_RS_RNGCMD_CHAR_VAL)

/** @brief Ranging callback structure */
struct bt_rs_cb {
	void (*start_ranging)(void);
	void (*iv_changed)(void);
	void (*key_changed)(void);
};

void bt_rs_set_key(const uint32_t *key);
void bt_rs_get_key(uint32_t *key);
void bt_rs_set_iv_upper96(const uint32_t *iv);
void bt_rs_get_iv(uint32_t *iv);
void bt_rs_set_iv_counter(uint32_t counter);
uint32_t bt_rs_get_iv_counter(void);
void bt_rs_set_timestamp(uint64_t *ts);
int bt_rs_indicate_timestamp(struct bt_conn *conn);
int bt_rs_notify_timestamp(struct bt_conn *conn);
int bt_rs_cb_init(const struct bt_rs_cb *cb);