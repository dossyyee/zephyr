/** @file
 *  @brief GATT Device Information Service
 */

/*
 * Copyright (c) 2019 Demant
 * Copyright (c) 2018 Nordic Semiconductor ASA
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
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

#define LOG_LEVEL CONFIG_BT_SERVICE_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(bt_rs);

static uint32_t sts_key[4] = {
	(CONFIG_BT_RS_KEY128 >> 96) & 0xFFFFFFFF,
	(CONFIG_BT_RS_KEY128 >> 64) & 0xFFFFFFFF,
	(CONFIG_BT_RS_KEY128 >> 32) & 0xFFFFFFFF,
	CONFIG_BT_RS_KEY128 & 0xFFFFFFFF
};

static uint32_t sts_iv[4] = {
	(CONFIG_BT_RS_IV_UPPER96 >> 96) & 0xFFFFFFFF,
	(CONFIG_BT_RS_IV_UPPER96 >> 64) & 0xFFFFFFFF,
	(CONFIG_BT_RS_IV_UPPER96 >> 32) & 0xFFFFFFFF,
	CONFIG_BT_RS_IV_COUNT32 & 0xFFFFFFFF
}

/* To restore the key and IV to defaults if necessary */
#define BT_RS_STS_KEY_DEFAULT		sts_key
#define BT_RS_STS_IV_DEFAULT		sts_iv

/* UUID declaration */
#define BT_UUID_RS_VAL			BT_UUID_128_ENCODE(0x49aa9800,0x34c9,0x40ca,0x95db,0x4eea1a31a229)
#define BT_UUID_RS			BT_UUID_DECLARE_128(BT_UUID_RS_VAL)
#define BT_UUID_RS_COUNTER_CHAR_VAL	BT_UUID_128_ENCODE(0x49aa9801,0x34c9,0x40ca,0x95db,0x4eea1a31a229)
#define BT_UUID_RS_COUNTER_CHAR		BT_UUID_DECLARE_128(BT_UUID_RS_COUNTER_CHAR_VAL)

/* TODO: writefunc for counter */

static ssize_t read_counter(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf,
			 uint16_t len, uint16_t offset)
{
	uint32_t counter = sts_iv[3];
	return bt_gatt_attr_read(conn, attr, buf, len, offset, &counter, sizeof(counter));
}

static ssize_t write_counter(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf,
			uint16_t len, uint16_t offset, uint8_t flags)
{
	/* check the length of the buf */
	/* save the buf to the sts_iv value */
	memcpy(&ct_time, buf, sizeof(ct_time));
}

/* Ranging Service Declaration */
BT_GATT_SERVICE_DEFINE(
	rs_svc, BT_GATT_PRIMARY_SERVICE(BT_UUID_RS),
	/* 32 Bit counter characteristic */
	BT_GATT_CHARACTERISTIC( BT_UUID_RS_COUNTER_CHAR, 
				BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE, 
				BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
			        read_counter, write_counter, NULL),
);

/* TODO: create a get and set key, IV and counter functions. */

