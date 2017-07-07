/*
 * Copyright (C) 2017 Beck IPC GmbH
 * Copyright (C) 2017 kernel concepts GmbH
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */
#ifndef __BECK_SC145_CONFIG_H
#define __BECK_SC145_CONFIG_H

#define PHYS_SDRAM_SIZE		SZ_128M
#define BOARD_STR		"sc145"
#define FSSIZE_STR		"0x3f70000"
#define FLASHSIZE_STR           "0x04000000"

#ifdef CONFIG_FSL_QSPI
#define FSL_QSPI_FLASH_SIZE	SZ_64M
#endif
	
#include "beck_sc1x5.h"

#endif

