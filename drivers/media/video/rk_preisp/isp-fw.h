/*
 * Copyright (C) Fuzhou Rockchip Electronics Co.Ltd
 * Author: Tusson <dusong@rock-chips.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __ISP_FIRMWARE_H__
#define __ISP_FIRMWARE_H__

#include <linux/types.h>
#include <linux/firmware.h>
#include "spi2apb.h"

#define RKL_MAX_SECTION_NUM 10

struct rkl_section {
	union {
		uint32_t offset;
		uint32_t wait_value;
	};
	uint32_t size;
	union {
		uint32_t load_addr;
		uint32_t wait_addr;
	};
	uint16_t wait_time;
	uint16_t timeout;
	uint16_t crc_16;
	uint8_t  flag;
	uint8_t  type;
};

struct rkl_header {
	char version[32];
	uint32_t header_size;
	uint32_t section_count;
	struct rkl_section sections[RKL_MAX_SECTION_NUM];
};

#define BOOT_FLAG_CRC		   (1 << 0)
#define BOOT_FLAG_EXE		   (1 << 1)
#define BOOT_FLAG_LOAD_PMEM	   (1 << 2)
#define BOOT_FLAG_ACK		   (1 << 3)
#define BOOT_FLAG_READ_WAIT	   (1 << 4)
#define BOOT_FLAG_BOOT_REQUEST (1 << 5)

struct rkl_boot_request {
	uint32_t flag;
	uint32_t load_addr;
	uint32_t boot_len;
	uint8_t status;
	uint8_t dummy[2];
	uint8_t cmd;
};

#define BOOT_REQUEST_ADDR 0x18000010
#define BOOT_REQUEST_WAIT_DELAY_MS 1
#define BOOT_REQUEST_TIMEOUT 10

#define DSP_HEAD_ADDR 0x60000000

#define RKL_DEFAULT_FW_NAME "preisp.rkl"

/**
 * preisp_spi_download_fw: - rk preisp firmware download through spi
 *
 * @spi: spi device
 * @fw_name: name of firmware file, NULL for default firmware name
 * Context: can sleep
 *
 * It returns zero on success, else a negative error code.
 **/
int preisp_spi_download_fw(struct spi_device *spi, const char *fw_name);

#endif
