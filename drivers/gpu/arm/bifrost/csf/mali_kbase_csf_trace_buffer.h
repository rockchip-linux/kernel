/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */
/*
 *
 * (C) COPYRIGHT 2018-2022 ARM Limited. All rights reserved.
 *
 * This program is free software and is provided to you under the terms of the
 * GNU General Public License version 2 as published by the Free Software
 * Foundation, and any use by you of this program is subject to the terms
 * of such GNU license.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, you can access it online at
 * http://www.gnu.org/licenses/gpl-2.0.html.
 *
 */

#ifndef _KBASE_CSF_TRACE_BUFFER_H_
#define _KBASE_CSF_TRACE_BUFFER_H_

#include <linux/types.h>

#define CSF_FIRMWARE_TRACE_ENABLE_INIT_MASK_MAX (4)
#define FIRMWARE_LOG_BUF_NAME "fwlog"

/* Forward declarations */
struct firmware_trace_buffer;
struct kbase_device;

/**
 * kbase_csf_firmware_trace_buffers_init - Initialize trace buffers
 *
 * @kbdev: Device pointer
 *
 * Allocate resources for trace buffers. In particular:
 * - One memory page of GPU-readable, CPU-writable memory is used for
 *   the Extract variables of all trace buffers.
 * - One memory page of GPU-writable, CPU-readable memory is used for
 *   the Insert variables of all trace buffers.
 * - A data buffer of GPU-writable, CPU-readable memory is allocated
 *   for each trace buffer.
 *
 * After that, firmware addresses are written with pointers to the
 * insert, extract and data buffer variables. The size and the trace
 * enable bits are not dereferenced by the GPU and shall be written
 * in the firmware addresses directly.
 *
 * This function relies on the assumption that the list of
 * firmware_trace_buffer elements in the device has already been
 * populated with data from the firmware image parsing.
 *
 * Return: 0 if success, or an error code on failure.
 */
int kbase_csf_firmware_trace_buffers_init(struct kbase_device *kbdev);

/**
 * kbase_csf_firmware_trace_buffers_term - Terminate trace buffers
 *
 * @kbdev: Device pointer
 */
void kbase_csf_firmware_trace_buffers_term(struct kbase_device *kbdev);

/**
 * kbase_csf_firmware_parse_trace_buffer_entry - Process a "trace buffer" section
 *
 * @kbdev:     Kbase device structure
 * @entry:     Pointer to the section
 * @size:      Size (in bytes) of the section
 * @updatable: Indicates whether config items can be updated with FIRMWARE_CONFIG_UPDATE
 *
 * Read a "trace buffer" section adding metadata for the related trace buffer
 * to the kbase_device:csf.firmware_trace_buffers list.
 *
 * Unexpected trace buffers will not be parsed and, as a consequence,
 * will not be initialized.
 *
 * Return: 0 if successful, negative error code on failure.
 */
int kbase_csf_firmware_parse_trace_buffer_entry(struct kbase_device *kbdev,
						const u32 *entry,
						unsigned int size,
						bool updatable);

/**
 * kbase_csf_firmware_reload_trace_buffers_data - Reload trace buffers data for firmware reboot
 *
 * @kbdev: Device pointer
 *
 * Helper function used when rebooting the firmware to reload the initial setup
 * for all the trace buffers which have been previously parsed and initialized.
 *
 * Almost all of the operations done in the initialization process are
 * replicated, with the difference that they might be done in a different order
 * and that the variables of a given trace buffer may be mapped to different
 * offsets within the same existing mappings.
 *
 * In other words, the re-initialization done by this function will be
 * equivalent but not necessarily identical to the original initialization.
 */
void kbase_csf_firmware_reload_trace_buffers_data(struct kbase_device *kbdev);

/**
 * kbase_csf_firmware_get_trace_buffer - Get a trace buffer
 *
 * @kbdev: Device pointer
 * @name:  Name of the trace buffer to find
 *
 * Return: handle to a trace buffer, given the name, or NULL if a trace buffer
 *         with that name couldn't be found.
 */
struct firmware_trace_buffer *kbase_csf_firmware_get_trace_buffer(
	struct kbase_device *kbdev, const char *name);

/**
 * kbase_csf_firmware_trace_buffer_get_trace_enable_bits_count - Get number of trace enable bits for a trace buffer
 *
 * @trace_buffer: Trace buffer handle
 *
 * Return: Number of trace enable bits in a trace buffer.
 */
unsigned int kbase_csf_firmware_trace_buffer_get_trace_enable_bits_count(
	const struct firmware_trace_buffer *trace_buffer);

/**
 * kbase_csf_firmware_trace_buffer_update_trace_enable_bit - Update a trace enable bit
 *
 * @trace_buffer: Trace buffer handle
 * @bit:          Bit to update
 * @value:        New value for the given bit
 *
 * Update the value of a given trace enable bit.
 *
 * Return: 0 if successful, negative error code on failure.
 */
int kbase_csf_firmware_trace_buffer_update_trace_enable_bit(
	struct firmware_trace_buffer *trace_buffer, unsigned int bit,
	bool value);

/**
 * kbase_csf_firmware_trace_buffer_is_empty - Empty trace buffer predicate
 *
 * @trace_buffer: Trace buffer handle
 *
 * Return: True if the trace buffer is empty, or false otherwise.
 */
bool kbase_csf_firmware_trace_buffer_is_empty(
	const struct firmware_trace_buffer *trace_buffer);

/**
 * kbase_csf_firmware_trace_buffer_read_data - Read data from a trace buffer
 *
 * @trace_buffer: Trace buffer handle
 * @data:         Pointer to a client-allocated where data shall be written.
 * @num_bytes:    Maximum number of bytes to read from the trace buffer.
 *
 * Read available data from a trace buffer. The client provides a data buffer
 * of a given size and the maximum number of bytes to read.
 *
 * Return: Number of bytes read from the trace buffer.
 */
unsigned int kbase_csf_firmware_trace_buffer_read_data(
	struct firmware_trace_buffer *trace_buffer, u8 *data, unsigned int num_bytes);

/**
 * kbase_csf_firmware_trace_buffer_get_active_mask64 - Get trace buffer active mask
 *
 * @tb: Trace buffer handle
 *
 * Return: Trace buffer active mask.
 */
u64 kbase_csf_firmware_trace_buffer_get_active_mask64(struct firmware_trace_buffer *tb);

/**
 * kbase_csf_firmware_trace_buffer_set_active_mask64 - Set trace buffer active mask
 *
 * @tb: Trace buffer handle
 * @mask: New active mask
 *
 * Return: 0 if successful, negative error code on failure.
 */
int kbase_csf_firmware_trace_buffer_set_active_mask64(struct firmware_trace_buffer *tb, u64 mask);

#endif /* _KBASE_CSF_TRACE_BUFFER_H_ */
