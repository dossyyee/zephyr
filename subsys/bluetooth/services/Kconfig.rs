# GATT Device Information service

# Copyright (c) 2018 Nordic Semiconductor ASA
# SPDX-License-Identifier: Apache-2.0

menuconfig BT_RS
	bool "GATT Ranging service"

if BT_RS

# config USE_DEFAULT_STS_KEY_IV
# 	bool "Use default sts key and iv (not recommended)"
# 	default y
# 	help
# 	  The default key and iv for the sts of ieee802.15.4z specification

config BT_RS_IV_UPPER96
	string "Upper 96 bits of the STS Input Vector"
	default "1F9A3DE4D37EC3CAC44FA8FB"
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
	string "128 bit STS encryption key"
	default "14EB220FF86050A8D1D336AA14148674" 
	help
	  The 128 bit encryption key used to encrypt the STS input vector

endif # BT_RS
