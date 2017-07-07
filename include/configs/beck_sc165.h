/*
 * Copyright (C) 2017 Beck IPC GmbH
 * Copyright (C) 2017 kernel concepts GmbH
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */
#ifndef __BECK_SC165_CONFIG_H
#define __BECK_SC165_CONFIG_H

#define PHYS_SDRAM_SIZE		SZ_512M
#define BOARD_STR		"sc165"
#define FSSIZE_STR		"0xff70000"
#define FLASHSIZE_STR           "0x10000000"

#ifdef CONFIG_FSL_QSPI
#define FSL_QSPI_FLASH_SIZE	SZ_256M
#endif

#include "beck_sc1x5.h"

#endif

