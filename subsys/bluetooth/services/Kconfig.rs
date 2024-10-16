# GATT Device Information service

# Copyright (c) 2018 Nordic Semiconductor ASA
# SPDX-License-Identifier: Apache-2.0

menuconfig BT_RS
	bool "GATT Ranging service"

if BT_RS

config BT_RS_IV_UPPER96
	hex "Upper 96 bits of the STS Input Vector"
	default 0x1F9A3DE4D37EC3CAC44FA8FB
	range 0x0 0xFFFFFFFFFFFFFFFFFFFFFFFF
	help
	  The upper 96 bits of the encryption initialisation vector
	  used for sts transmission. This is typically not changed
	  during operation.

config BT_RS_IV_COUNT32
	hex "Lower 32 bits Counter portion of the STS Input Vector"
	default 0x362EEB34
	range 0x0 0xFFFFFFFF
	help
	  The lower 32 bits of the encryption input vector
	  used for sts transmission. Usually referred to as the counter

config BT_RS_KEY128
	hex "128 bit STS encryption key"
	default 0x14EB220FF86050A8D1D336AA14148674 
	help
	  The 128 bit encryption key used to encrypt the STS input vector

endif # BT_RS
