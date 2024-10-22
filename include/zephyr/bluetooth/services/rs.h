
#include <stdint.h>

#define STS_UINT_LEN	4

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
int bt_rs_cb_init(const struct bt_rs_cb *cb);