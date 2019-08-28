/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Fundamental types and constants relating to WFA NAN
 * (Neighbor Awareness Networking)
 *
 * Copyright (C) 1999-2018, Broadcom Corporation
 * 
 *      Unless you and Broadcom execute a separate written software license
 * agreement governing use of this software, this software is licensed to you
 * under the terms of the GNU General Public License version 2 (the "GPL"),
 * available at http://www.broadcom.com/licenses/GPLv2.php, with the
 * following added to such license:
 * 
 *      As a special exception, the copyright holders of this software give you
 * permission to link this software with independent modules, and to copy and
 * distribute the resulting executable under terms of your choice, provided that
 * you also meet, for each linked independent module, the terms and conditions of
 * the license of that module.  An independent module is a module which is not
 * derived from this software.  The special exception does not apply to any
 * modifications of the software.
 * 
 *      Notwithstanding the above, under no circumstances may you combine this
 * software in any way with any other Broadcom software provided under a license
 * other than the GPL, without Broadcom's express prior written consent.
 *
 * <<Broadcom-WL-IPTag/Open:>>
 *
 * $Id: nan.h 581725 2015-08-25 10:43:26Z $
 */
#ifndef _NAN_H_
#define _NAN_H_

#include <typedefs.h>
#include <proto/802.11.h>


/* This marks the start of a packed structure section. */
#include <packed_section_start.h>

/* WiFi NAN OUI values */
#define NAN_OUI            WFA_OUI     /* WiFi OUI */
/* For oui_type field identifying the type and version of the NAN IE. */
#define NAN_OUI_TYPE       0x13        /* Type/Version */
/* IEEE 802.11 vendor specific information element. (Same as P2P_IE_ID.) */
#define NAN_IE_ID          0xdd

/* Same as P2P_PUB_AF_CATEGORY and DOT11_ACTION_CAT_PUBLIC */
#define NAN_PUB_AF_CATEGORY     0x04
/* IEEE 802.11 Public Action Frame Vendor Specific. (Same as P2P_PUB_AF_ACTION.) */
#define NAN_PUB_AF_ACTION       0x09
/* Number of octents in hash of service name. (Same as P2P_WFDS_HASH_LEN.) */
#define NAN_SVC_HASH_LEN    6
/* Size of fixed length part of nan_pub_act_frame_t before attributes. */
#define NAN_PUB_ACT_FRAME_FIXED_LEN 6
/* Number of octents in master rank value. */
#define NAN_MASTER_RANK_LEN     8
/* NAN public action frame header size */
#define NAN_PUB_ACT_FRAME_HDR_SIZE (OFFSETOF(nan_pub_act_frame_t, data))
/* NAN network ID */
#define NAN_NETWORK_ID		"\x51\x6F\x9A\x01\x00\x00"
/* Service Control Type length */
#define NAN_SVC_CONTROL_TYPE_LEN	2
/* Binding Bitmap length */
#define NAN_BINDING_BITMAP_LEN		2
/* Service Response Filter (SRF) control field masks */
#define NAN_SRF_BLOOM_MASK		0x01
#define NAN_SRF_INCLUDE_MASK		0x02
#define NAN_SRF_INDEX_MASK		0x0C
/* SRF Bloom Filter index shift */
#define NAN_SRF_BLOOM_SHIFT	2
#define NAN_SRF_INCLUDE_SHIFT	1
/* Mask for CRC32 output, used in hash function for NAN bloom filter */
#define NAN_BLOOM_CRC32_MASK	0xFFFF

/* Attribute TLV header size */
#define NAN_ATTR_ID_OFF		0
#define NAN_ATTR_LEN_OFF	1
#define NAN_ATTR_DATA_OFF	3

#define NAN_ATTR_ID_LEN		 1	/* ID field length */
#define NAN_ATTR_LEN_LEN	 2	/* Length field length */
#define NAN_ATTR_HDR_LEN	 3	/* ID + 2-byte length field */
#define NAN_ENTRY_CTRL_LEN       1      /* Entry control field length from FAM attribute */
#define NAN_MAP_ID_LEN           1	/* MAP ID length to signify band */
#define NAN_OPERATING_CLASS_LEN  1	/* operating class field length from NAN FAM */
#define NAN_CHANNEL_NUM_LEN      1	/* channel number field length 1 byte */

#define NAN_MAP_ID_2G   2  /* NAN Further Avail Map ID for band 2.4G */
#define NAN_MAP_ID_5G   5  /* NAN Further Avail Map ID for band 5G */
#define NAN_MAP_NUM_IDS 2  /* Max number of NAN Further Avail Map IDs supported */

#define NAN_AVAIL_ENTRY_LEN_RES0 7      /* Avail entry len in FAM attribute for resolution 16TU */
#define NAN_AVAIL_ENTRY_LEN_RES1 5      /* Avail entry len in FAM attribute for resolution 32TU */
#define NAN_AVAIL_ENTRY_LEN_RES2 4      /* Avail entry len in FAM attribute for resolution 64TU */

/* Vendor-specific public action frame for NAN */
typedef BWL_PRE_PACKED_STRUCT struct nan_pub_act_frame_s {
	/* NAN_PUB_AF_CATEGORY 0x04 */
	uint8 category_id;
	/* NAN_PUB_AF_ACTION 0x09 */
	uint8 action_field;
	/* NAN_OUI 0x50-6F-9A */
	uint8 oui[DOT11_OUI_LEN];
	/* NAN_OUI_TYPE 0x13 */
	uint8 oui_type;
	/* One or more NAN Attributes follow */
	uint8 data[1];
} BWL_POST_PACKED_STRUCT nan_pub_act_frame_t;

/* NAN attributes as defined in the nan spec */
enum {
	NAN_ATTR_MASTER_IND = 0,
	NAN_ATTR_CLUSTER = 1,
	NAN_ATTR_SVC_ID_LIST = 2,
	NAN_ATTR_SVC_DESCRIPTOR = 3,
	NAN_ATTR_CONN_CAP = 4,
	NAN_ATTR_INFRA = 5,
	NAN_ATTR_P2P = 6,
	NAN_ATTR_IBSS = 7,
	NAN_ATTR_MESH = 8,
	NAN_ATTR_FURTHER_NAN_SD = 9,
	NAN_ATTR_FURTHER_AVAIL = 10,
	NAN_ATTR_COUNTRY_CODE = 11,
	NAN_ATTR_RANGING = 12,
	NAN_ATTR_CLUSTER_DISC = 13,
	NAN_ATTR_VENDOR_SPECIFIC = 221
};

enum wifi_nan_avail_resolution {
	NAN_AVAIL_RES_16_TU = 0,
	NAN_AVAIL_RES_32_TU = 1,
	NAN_AVAIL_RES_64_TU = 2,
	NAN_AVAIL_RES_INVALID = 255
};

typedef BWL_PRE_PACKED_STRUCT struct wifi_nan_ie_s {
	uint8	id;		/* IE ID: NAN_IE_ID 0xDD */
	uint8	len;		/* IE length */
	uint8	oui[DOT11_OUI_LEN]; /* NAN_OUI 50:6F:9A */
	uint8	oui_type;	/* NAN_OUI_TYPE 0x13 */
	uint8	attr[1];	/* var len attributes */
} BWL_POST_PACKED_STRUCT wifi_nan_ie_t;

#define NAN_IE_HDR_SIZE	(OFFSETOF(wifi_nan_ie_t, attr))

/* master indication record  */
typedef BWL_PRE_PACKED_STRUCT struct wifi_nan_master_ind_attr_s {
	uint8	id;
	uint16	len;
	uint8	master_preference;
	uint8	random_factor;
} BWL_POST_PACKED_STRUCT wifi_nan_master_ind_attr_t;

/* cluster attr record  */
typedef BWL_PRE_PACKED_STRUCT struct wifi_nan_cluster_attr_s {
	uint8	id;
	uint16	len;
	uint8   amr[NAN_MASTER_RANK_LEN];
	uint8   hop_count;
	/* Anchor Master Beacon Transmission Time */
	uint32  ambtt;
} BWL_POST_PACKED_STRUCT wifi_nan_cluster_attr_t;

/*  container for service ID records  */
typedef BWL_PRE_PACKED_STRUCT struct wifi_nan_svc_id_attr_s {
	uint8	id;
	uint16	len;
	uint8	svcid[NAN_SVC_HASH_LEN]; /* 6*len of srvc IDs */
} BWL_POST_PACKED_STRUCT wifi_nan_svc_id_attr_t;

/* service_control bitmap for wifi_nan_svc_descriptor_attr_t below */
#define NAN_SC_PUBLISH 0x0
#define NAN_SC_SUBSCRIBE 0x1
#define NAN_SC_FOLLOWUP 0x2
/* Set to 1 if a Matching Filter field is included in descriptors. */
#define NAN_SC_MATCHING_FILTER_PRESENT 0x4
/* Set to 1 if a Service Response Filter field is included in descriptors. */
#define NAN_SC_SR_FILTER_PRESENT 0x8
/* Set to 1 if a Service Info field is included in descriptors. */
#define NAN_SC_SVC_INFO_PRESENT 0x10
/* range is close proximity only */
#define NAN_SC_RANGE_LIMITED 0x20
/* Set to 1 if binding bitamp is present in descriptors */
#define NAN_SC_BINDING_BITMAP_PRESENT 0x40

/* Service descriptor */
typedef BWL_PRE_PACKED_STRUCT struct wifi_nan_svc_descriptor_attr_s {
	/* Attribute ID - 0x03. */
	uint8 id;
	/* Length of the following fields in the attribute */
	uint16 len;
	/* Hash of the Service Name */
	uint8 svc_hash[NAN_SVC_HASH_LEN];
	/* Publish or subscribe instance id */
	uint8 instance_id;
	/* Requestor Instance ID */
	uint8 requestor_id;
	/* Service Control Bitmask. Also determines what data follows. */
	uint8 svc_control;
	/* Optional fields follow */
} BWL_POST_PACKED_STRUCT wifi_nan_svc_descriptor_attr_t;

/* IBSS attribute */
typedef BWL_PRE_PACKED_STRUCT struct wifi_nan_ibss_attr_s {
	/* Attribute ID - 0x07. */
	uint8 id;
	/* Length of the following fields in the attribute */
	uint16 len;
	/* BSSID of the ibss */
	struct ether_addr bssid;
	/*
	 map control:, bits:
	[0-3]: Id for associated further avail map attribute
	[4-5]: avail interval duration: 0:16ms; 1:32ms; 2:64ms; 3:reserved
	[6] : repeat : 0 - applies to next DW, 1: 16 intervals max? wtf?
	[7] : reserved
	*/
	uint8 map_ctrl;
	/* avail. intervals bitmap, var len  */
	uint8 avail_bmp[1];
} BWL_POST_PACKED_STRUCT wifi_nan_ibss_attr_t;

/* Further Availability MAP attr  */
typedef BWL_PRE_PACKED_STRUCT struct wifi_nan_favail_attr_s {
	/* Attribute ID - 0x0A. */
	uint8 id;
	/* Length of the following fields in the attribute */
	uint16 len;
	/* MAP id: val [0..15], values[16-255] reserved */
	uint8 map_id;
	/*  availibility entry, var len */
	uint8 avil_entry[1];
} BWL_POST_PACKED_STRUCT wifi_nan_favail_attr_t;

/* Further Availability MAP attr  */
typedef BWL_PRE_PACKED_STRUCT struct wifi_nan_avail_entry_s {
	/*
	 entry control
	 [0-1]: avail interval duration: 0:16ms; 1:32ms; 2:64ms;
	 [2:7] reserved
	*/
	uint8 entry_ctrl;
	/* operating class: freq band etc IEEE 802.11 */
	uint8 opclass;
	/* channel number */
	uint8 chan;
	/*  avail bmp, var len */
	uint8 avail_bmp[1];
} BWL_POST_PACKED_STRUCT wifi_nan_avail_entry_t;

/* Map control Field */
#define NAN_MAPCTRL_IDMASK	0x7
#define NAN_MAPCTRL_DURSHIFT	4
#define NAN_MAPCTRL_DURMASK	0x30
#define NAN_MAPCTRL_REPEAT	0x40
#define NAN_MAPCTRL_REPEATSHIFT	6

#define NAN_VENDOR_TYPE_RTT	0
#define NAN_VENDOR_TYPE_P2P	1

/* Vendor Specific Attribute */
typedef BWL_PRE_PACKED_STRUCT struct wifi_nan_vendor_attr_s {
	uint8	id;		/* 0xDD */
	uint16	len;		/* IE length */
	uint8	oui[DOT11_OUI_LEN]; /* 00-90-4C */
	uint8	type;		/* attribute type */
	uint8	attr[1];	/* var len attributes */
} BWL_POST_PACKED_STRUCT wifi_nan_vendor_attr_t;

#define NAN_VENDOR_HDR_SIZE	(OFFSETOF(wifi_nan_vendor_attr_t, attr))

/* p2p operation attribute */
typedef BWL_PRE_PACKED_STRUCT struct wifi_nan_p2p_op_attr_s {
	/* Attribute ID - 0x06. */
	uint8 id;
	/* Length of the following fields in the attribute */
	uint16 len;
	/* P2P device role */
	uint8 dev_role;
	/* BSSID of the ibss */
	struct ether_addr p2p_dev_addr;
	/*
	map control:, bits:
	[0-3]: Id for associated further avail map attribute
	[4-5]: avail interval duration: 0:16ms; 1:32ms; 2:64ms; 3:reserved
	[6] : repeat : 0 - applies to next DW, 1: 16 intervals max? wtf?
	[7] : reserved
	*/
	uint8 map_ctrl;
	/* avail. intervals bitmap */
	uint8 avail_bmp[1];
} BWL_POST_PACKED_STRUCT wifi_nan_p2p_op_attr_t;

/* ranging attribute */
#define NAN_RANGING_MAP_CTRL_ID_SHIFT 0
#define NAN_RANGING_MAP_CTRL_ID_MASK 0x0F
#define NAN_RANGING_MAP_CTRL_DUR_SHIFT 4
#define NAN_RANGING_MAP_CTRL_DUR_MASK 0x30
#define NAN_RANGING_MAP_CTRL_REPEAT_SHIFT 6
#define NAN_RANGING_MAP_CTRL_REPEAT_MASK 0x40
#define NAN_RANGING_MAP_CTRL_REPEAT_DW(_ctrl) (((_ctrl) & \
	NAN_RANGING_MAP_CTRL_DUR_MASK) ? 16 : 1)
#define NAN_RANGING_MAP_CTRL(_id, _dur, _repeat) (\
	(((_id) << NAN_RANGING_MAP_CTRL_ID_SHIFT) & \
		 NAN_RANGING_MAP_CTRL_ID_MASK) | \
	(((_dur) << NAN_RANGING_MAP_CTRL_DUR_SHIFT) & \
		NAN_RANGING_MAP_CTRL_DUR_MASK) | \
	(((_repeat) << NAN_RANGING_MAP_CTRL_REPEAT_SHIFT) & \
		 NAN_RANGING_MAP_CTRL_REPEAT_MASK))

enum {
	NAN_RANGING_PROTO_FTM = 0
};

typedef BWL_PRE_PACKED_STRUCT struct wifi_nan_ranging_attr_s {
	uint8 id;						/* 0x0C */
	uint16 len; 					/* length that follows */
	struct ether_addr dev_addr;		/* device mac address */

	/*
	map control:, bits:
	[0-3]: Id for associated further avail map attribute
	[4-5]: avail interval duration: 0:16ms; 1:32ms; 2:64ms; 3:reserved
	[6] : repeat : 0 - applies to next DW, 1: 16 intervals max? wtf?
	[7] : reserved
	*/
	uint8 map_ctrl;

	uint8 protocol;					/* FTM = 0 */
	uint32 avail_bmp;				/* avail interval bitmap */
} BWL_POST_PACKED_STRUCT wifi_nan_ranging_attr_t;

#define NAN_CONN_CAPABILITY_WFD		0x0001
#define NAN_CONN_CAPABILITY_WFDS		0x0002
#define NAN_CONN_CAPABILITY_TDLS		0x0004
#define NAN_CONN_CAPABILITY_INFRA	0x0008
#define NAN_CONN_CAPABILITY_IBSS		0x0010
#define NAN_CONN_CAPABILITY_MESH		0x0020

typedef BWL_PRE_PACKED_STRUCT struct wifi_nan_conn_cap_attr_s {
	/* Attribute ID - 0x04. */
	uint8 id;
	/* Length of the following fields in the attribute */
	uint16	len;
	uint16	conn_cap_bmp;	/* Connection capability bitmap */
} BWL_POST_PACKED_STRUCT wifi_nan_conn_cap_attr_t;

#define NAN_SLOT_RES_16TU 16
#define NAN_SLOT_RES_32TU 32
#define NAN_SLOT_RES_64TU 64

/* This marks the end of a packed structure section. */
#include <packed_section_end.h>

#endif /* _NAN_H_ */
