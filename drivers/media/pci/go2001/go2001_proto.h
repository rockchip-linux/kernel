/*
 *  go2001 - GO2001 codec driver.
 *
 *  Author : Pawel Osciak <posciak@chromium.org>
 *
 *  Copyright (C) 2015 Google, Inc.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
#ifndef _MEDIA_PCI_GO2001_GO2001_PROTO_H_
#define _MEDIA_PCI_GO2001_GO2001_PROTO_H_

struct go2001_msg_ring_desc {
	u32 msg_size;
	u32 start_off;
	u32 end_off;
	u32 wr_off;
	u32 rd_off;
} __attribute__((packed));

#define GO2001_CTRL_SESSION_ID	0
struct go2001_msg_hdr {
	u32 size;
	u32 session_id;
	u32 sequence_id;
	u32 type;
	u32 status;
} __attribute__((packed));

#define GO2001_MAX_MSG_PAYLOAD_SIZE	SZ_256
struct go2001_msg_payload {
	struct go2001_msg_hdr hdr;
	u8 param[GO2001_MAX_MSG_PAYLOAD_SIZE - sizeof(struct go2001_msg_hdr)];
} __attribute__((packed));

#define go2001_calc_payload_size(param_size) \
	(sizeof(struct go2001_msg_hdr) + (param_size))

enum go2001_msg_status {
	GO2001_STATUS_OK = 0,
	GO2001_STATUS_FAIL = 1,
	GO2001_STATUS_RES_NA = 2,
	GO2001_STATUS_INVALID_PARAM = 3,
	GO2001_STATUS_NOT_IMPLEMENTED = 4,
	GO2001_STATUS_NEW_PICTURE_SIZE = 5,
	GO2001_STATUS_WAITING_PICTURE_SIZE_CHANGED = 6,
	GO2001_STATUS_STREAM_ERROR = 7,
	GO2001_STATUS_NO_OUTPUT = 8,
	GO2001_STATUS_INFO_NOT_READY = 9,
	GO2001_STATUS_RESTART_SYS = 10,
};

enum go2001_msg_type {
	GO2001_VM_INIT_DECODER = 0x100,
	GO2001_VM_INIT_ENCODER = 0x101,
	GO2001_VM_INIT_PASSTHROUGH = 0x102,
	GO2001_VM_GET_VERSION = 0x104,
	GO2001_VM_SET_MMAP = 0x105,
	GO2001_VM_RELEASE_MMAP = 0x106,
	GO2001_VM_EMPTY_BUFFER = 0x107,
	GO2001_VM_GET_INFO = 0x108,
	GO2001_VM_SET_CTRL = 0x109,
	GO2001_VM_GET_CTRL = 0x10a,
	GO2001_VM_RELEASE = 0x10b,
	GO2001_VM_RELEASE_ALL = 0x10c,
	GO2001_VM_EVENT_ASSERT = 0x200,
	GO2001_VM_EVENT_LOG = 0x201,
	GO2001_VM_SET_LOG_LEVEL = 0x300,
};

#define GO2001_MSG_RING_MEM_OFFSET	0x401000
#define GO2001_MSG_RING_MEM_SIZE	0x2000
#define GO2001_RX_RING_DESC_OFFSET	0x0
#define GO2001_TX_RING_DESC_OFFSET	0x100

#define GO2001_FMT_NV12			0x20001
#define GO2001_FMT_NV12_TILED_8X4	0x20002
#define GO2001_FMT_ARGB			0x20003

enum go2001_hw_format_coded {
	GO2001_FMT_VP8 = 1,
	GO2001_FMT_VP9 = 2,
};

enum go2001_hw_format_raw {
	GO2001_FMT_YUV420_PLANAR = 0,
	GO2001_FMT_YUV420_SEMIPLANAR = 1,
	GO2001_FMT_YUV420_SEMIPLANAR_VU = 2,
	GO2001_FMT_YUV422_INTERLEAVED_YUYV = 3,
	GO2001_FMT_YUV422_INTERLEAVED_UYVY = 4,
	GO2001_FMT_RGB565 = 5,
	GO2001_FMT_BGR565 = 6,
	GO2001_FMT_RGB555 = 7,
	GO2001_FMT_BGR555 = 8,
	GO2001_FMT_RGB444 = 9,
	GO2001_FMT_BGR444 = 10,
	GO2001_FMT_RGB888 = 11,
	GO2001_FMT_BGR888 = 12,
	GO2001_FMT_RGB101010 = 13,
	GO2001_FMT_BGR101010 = 14,
};

struct go2001_init_decoder_param {
	u32 coded_fmt;
	u32 concealment;
} __attribute__((packed));

struct go2001_init_decoder_reply {
	u32 session_id;
	u32 input_address;
	u32 input_buffer_size;
	u32 output_address;
} __attribute__((packed));

struct go2001_init_encoder_param {
	u32 session_id;
	u32 num_ref_frames;
	u32 width;
	u32 height;
	u32 orig_width;
	u32 orig_height;
	u32 framerate_num;
	u32 framerate_denom;
	u32 raw_fmt;
} __attribute__((packed));

struct go2001_init_encoder_reply {
} __attribute__((packed));

struct go2001_get_info_reply {
	u32 vpx_version;
	u32 vpx_profile;
	u32 visible_width;
	u32 visible_height;
	u32 coded_width;
	u32 coded_height;
	u32 scaled_width;
	u32 scaled_height;
} __attribute__((packed));

struct go2001_get_version_reply {
	u32 hw_ver;
	u32 sw_ver;
	u32 vp8dec_hw_ver;
	u32 vp8dec_sw_ver;
	u32 vp8enc_hw_ver;
	u32 vp8enc_sw_ver;
	u32 vp9dec_hw_ver;
	u32 vp9dec_sw_ver;
} __attribute__((packed));

struct go2001_enc_coding_ctrl_area {
	u32 enabled;
	u32 top;
	u32 left;
	u32 bottom;
	u32 right;
} __attribute__((packed));

#define GO2001_CODING_CTRL_INTERP_FILTER_BICUBIC	0x0
#define GO2001_CODING_CTRL_INTERP_FILTER_BILINEAR	0x1
#define GO2001_CODING_CTRL_INTERP_FILTER_NONE		0x2

#define GO2001_CODING_CTRL_DEBLOCK_FILTER_NORMAL	0x0
#define GO2001_CODING_CTRL_DEBLOCK_FILTER_SIMPLE	0x1

#define GO2001_CODING_CTRL_MV_DISABLED		0x0
#define GO2001_CODING_CTRL_MV_ADAPTIVE		0x1
#define GO2001_CODING_CTRL_MV_ENABLED		0x2

#define GO2001_CODING_CTRL_QM_PSNR	0x0
#define GO2001_CODING_CTRL_QM_SSIM	0x1
struct go2001_enc_coding_ctrl {
	u32 interp_filter_type;
	u32 deblock_filter_type;
	u32 deblock_filter_level;
	u32 deblock_filter_sharpness;
	u32 num_dct_parts;
	u32 error_resilient;
	u32 split_mv;
	u32 quarter_pixel_mv;
	u32 cir_start;
	u32 cir_interval;
	struct go2001_enc_coding_ctrl_area intra_area;
	struct go2001_enc_coding_ctrl_area roi1_area;
	struct go2001_enc_coding_ctrl_area roi2_area;
	s32 roi1_delta_qp;
	s32 roi2_delta_qp;
	u32 deadzone_enabled;
	u32 max_num_passes;
	u32 quality_metric;
	s32 qp_delta[5];
	s32 adaptive_roi;
	s32 adaptive_roi_color;
} __attribute__((packed));

struct go2001_enc_rate_ctrl {
} __attribute__((packed));

struct go2001_enc_preprocess_ctrl {
} __attribute__((packed));

enum go2001_hw_ctrl_type {
	GO2001_HW_CTRL_TYPE_RATE = 1,
	GO2001_HW_CTRL_TYPE_CODING = 2,
	GO2001_HW_CTRL_TYPE_PREPROCESS = 3,
};

union go2001_hw_ctrl {
	struct go2001_enc_rate_ctrl rate_ctrl;
	struct go2001_enc_coding_ctrl coding_ctrl;
	struct go2001_enc_preprocess_ctrl prep_ctrl;
} __attribute__((packed));;

struct go2001_set_ctrl_param {
	u32 type;
	union go2001_hw_ctrl ctrl;
} __attribute__((packed));

struct go2001_set_ctrl_reply {
} __attribute__((packed));

#define G02001_EMPTY_BUF_DEC_FLAG_RES_CHANGE_DONE	0x1
struct go2001_empty_buffer_dec_param {
	u64 in_addr;
	u32 payload_size;
	u64 out_addr[2];
	u32 flags;
	u32 raw_fmt;
} __attribute__((packed));

struct go2001_empty_buffer_dec_reply {
	struct go2001_get_info_reply info;
	u32 intra_frame;
	u32 golden_frame;
	u32 num_error_mbs;
	u32 num_slice_rows;
} __attribute__((packed));

#define GO2001_EMPTY_BUF_ENC_FRAME_KEYFRAME	0x0
#define GO2001_EMPTY_BUF_ENC_FRAME_PRED		0x1

enum go2001_enc_frame_control {
	GO2001_FRM_CTRL_NO_REFRESH = 0,
	GO2001_FRM_CTRL_REFERENCE = 1,
	GO2001_FRM_CTRL_REFRESH = 2,
	GO2001_FRM_CTRL_REFERENCE_AND_REFRESH = 3,
};

struct go2001_empty_buffer_enc_param {
	u64 in_addr[3];
	u64 out_addr;
	u32 out_size;
	u32 frame_type;
	u32 time_increment;
	u64 stab_in_addr;
	u32 ipf_frame_ctrl;
	u32 grf_frame_ctrl;
	u32 arf_frame_ctrl;
	u32 layer_id;
	u32 bits_per_sec;
} __attribute__((packed));

#define VP8_MAX_NUM_PARTITIONS 9
struct go2001_empty_buffer_enc_reply {
	u32 frame_type;
	u32 partition_off[VP8_MAX_NUM_PARTITIONS];
	u32 partition_size[VP8_MAX_NUM_PARTITIONS];
	u32 payload_size;
	u32 ipf_frame_ctrl;
	u32 grf_frame_ctrl;
	u32 arf_frame_ctrl;
} __attribute__((packed));

struct go2001_mmap_list_desc {
	u64 first_entry_dma_addr;
	u32 entry_count;
	u64 mmap_list_addr;
} __attribute__((packed));

#define GO2001_VSM_DIR_IN	(1<<0)
#define GO2001_VSM_DIR_OUT	(1<<1)
#define GO2001_MMAP_MAX_ENTRIES	9
struct go2001_set_mmap_param {
	u32 dir;
	u32 count;
	struct go2001_mmap_list_desc mmap_list_desc[GO2001_MMAP_MAX_ENTRIES];
} __attribute__((packed));

struct go2001_mmap_list_entry {
	u64 dma_addr;
	u32 size;
} __attribute__((packed));

struct go2001_release_mmap_param {
	u32 dir;
	u32 count;
	u64 addr[GO2001_MMAP_MAX_ENTRIES];
} __attribute__((packed));

struct go2001_event_assert_reply {
	u8 filename[32];
	u8 funcname[32];
	u8 expr[32];
	u32 line_no;
} __attribute__((packed));

struct go2001_event_log_reply {
	u8 data[128];
} __attribute__((packed));

#define GO2001_LOG_LEVEL_MAX		5
#define GO2001_LOG_LEVEL_DISABLED	0xFFFFFFFF
struct go2001_set_log_level_param {
	u32 level;
} __attribute__((packed));

struct go2001_set_log_level_reply {
	u32 level;
} __attribute__((packed));

struct go2001_boot_hdr {
	u32 signature;
	u32 entry_addr;
	u32 size;
	u32 checksum;
} __attribute__((packed));

#define GO2001_FW_HDR_OFF	0x20afc0
#define GO2001_FW_STOP		0x212010
#define GO2001_FW_STOP_BIT	(1 << 20)

#define GO2001_BOOT_FW_NAME		"go2001-boot.fw"
#define GO2001_BOOT_FW_OFF		0x207000
#define GO2001_BOOT_FW_ENTRY_BASE	0xd0000000

#define GO2001_FW_NAME		"go2001.fw"
#define GO2001_FW_OFF		0
#define GO2001_FW_ENTRY_BASE	0

#define GO2001_FW_SIGNATURE \
(((u32)'P') | (((u32)'C') << 8) | (((u32)'I') << 16) | (((u32)'E') << 24))
#define GO2001_FW_DONE_SIGNATURE \
(((u32)'D') | (((u32)'O') << 8) | (((u32)'N') << 16) | (((u32)'E') << 24))

#define GO2001_FW_MAX_SIZE	SZ_1M

#endif /* _MEDIA_PCI_GO2001_GO2001_PROTO_H_ */
