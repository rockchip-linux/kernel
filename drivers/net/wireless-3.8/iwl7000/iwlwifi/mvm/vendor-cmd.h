/******************************************************************************
 *
 * This file is provided under a dual BSD/GPLv2 license.  When using or
 * redistributing this file, you may do so under either license.
 *
 * GPL LICENSE SUMMARY
 *
 * Copyright(c) 2012 - 2014 Intel Corporation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of version 2 of the GNU General Public License as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110,
 * USA
 *
 * The full GNU General Public License is included in this distribution
 * in the file called COPYING.
 *
 * Contact Information:
 *  Intel Linux Wireless <ilw@linux.intel.com>
 * Intel Corporation, 5200 N.E. Elam Young Parkway, Hillsboro, OR 97124-6497
 *
 * BSD LICENSE
 *
 * Copyright(c) 2012 - 2014 Intel Corporation. All rights reserved.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *  * Neither the name Intel Corporation nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *****************************************************************************/

#define INTEL_OUI	0x001735

/**
 * enum iwl_mvm_vendor_cmd - supported vendor commands
 * @IWL_MVM_VENDOR_CMD_SET_LOW_LATENCY: set low-latency mode for the given
 *	virtual interface
 * @IWL_MVM_VENDOR_CMD_GET_LOW_LATENCY: query low-latency mode
 * @IWL_MVM_VENDOR_CMD_TCM_EVENT: TCM event
 */
enum iwl_mvm_vendor_cmd {
	IWL_MVM_VENDOR_CMD_SET_LOW_LATENCY,
	IWL_MVM_VENDOR_CMD_GET_LOW_LATENCY,
	IWL_MVM_VENDOR_CMD_TCM_EVENT,
};

/**
 * enum iwl_mvm_vendor_load - traffic load identifiers
 * @IWL_MVM_VENDOR_LOAD_LOW: low load: less than 10% airtime usage
 * @IWL_MVM_VENDOR_LOAD_MEDIUM: medium load: 10% or more, but less than 50%
 * @IWL_MVM_VENDOR_LOAD_HIGH: high load: 50% or more
 *
 * Traffic load is calculated based on the percentage of airtime used
 * (TX airtime is accounted as RTS+CTS+PPDU+ACK/BlockACK, RX airtime
 * is just the PPDU's time)
 */
enum iwl_mvm_vendor_load {
	IWL_MVM_VENDOR_LOAD_LOW,
	IWL_MVM_VENDOR_LOAD_MEDIUM,
	IWL_MVM_VENDOR_LOAD_HIGH,
};

/**
 * enum iwl_mvm_vendor_attr - attributes used in vendor commands
 * @__IWL_MVM_VENDOR_ATTR_INVALID: attribute 0 is invalid
 * @IWL_MVM_VENDOR_ATTR_LOW_LATENCY: low-latency flag attribute
 * @IWL_MVM_VENDOR_ATTR_VIF_ADDR: interface MAC address
 * @IWL_MVM_VENDOR_ATTR_VIF_LL: vif-low-latency (u8, 0/1)
 * @IWL_MVM_VENDOR_ATTR_LL: global low-latency (u8, 0/1)
 * @IWL_MVM_VENDOR_ATTR_VIF_LOAD: vif traffic load (u8, see load enum)
 * @IWL_MVM_VENDOR_ATTR_LOAD: global traffic load (u8, see load enum)
 * @NUM_IWL_MVM_VENDOR_ATTR: number of vendor attributes
 * @MAX_IWL_MVM_VENDOR_ATTR: highest vendor attribute number
 */
enum iwl_mvm_vendor_attr {
	__IWL_MVM_VENDOR_ATTR_INVALID,
	IWL_MVM_VENDOR_ATTR_LOW_LATENCY,
	IWL_MVM_VENDOR_ATTR_VIF_ADDR,
	IWL_MVM_VENDOR_ATTR_VIF_LL,
	IWL_MVM_VENDOR_ATTR_LL,
	IWL_MVM_VENDOR_ATTR_VIF_LOAD,
	IWL_MVM_VENDOR_ATTR_LOAD,

	NUM_IWL_MVM_VENDOR_ATTR,
	MAX_IWL_MVM_VENDOR_ATTR = NUM_IWL_MVM_VENDOR_ATTR - 1,
};
