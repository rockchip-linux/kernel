/* SPDX-License-Identifier: ((GPL-2.0+ WITH Linux-syscall-note) OR MIT)
 *
 * Copyright (C) 2021 Rockchip Electronics Co., Ltd.
 */

#ifndef _UAPI_FEC_CONFIG_H
#define _UAPI_FEC_CONFIG_H

#include <linux/types.h>
#include <linux/v4l2-controls.h>

#define FEC_API_VERSION		KERNEL_VERSION(1, 0, 0)
struct fec_config {
	__u32 mesh_density;
	__u32 src_width;
	__u32 src_height;
	__u32 dst_width;
	__u32 dst_height;
	__u32 mesh_size;
	__s32 buf_fd;
	__u32 fec_bic_mode;
} __attribute__ ((packed));

struct fec_params_cfg {
	__u32 module_en_update;
	__u32 module_ens;
	__u32 module_cfg_update;

	__u32 frame_id;
	struct fec_config fec_cfg;
} __attribute__ ((packed));

#endif
