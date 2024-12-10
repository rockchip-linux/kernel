/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */
/*
 *
 * (C) COPYRIGHT 2019-2022 ARM Limited. All rights reserved.
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

#include <mali_kbase.h>

/**
 * kbase_device_get_list - get device list.
 * Get access to device list.
 *
 * Return: Pointer to the linked list head.
 */
const struct list_head *kbase_device_get_list(void);

/**
 * kbase_device_put_list - put device list.
 *
 * @dev_list: head of linked list containing device list.
 *
 * Put access to the device list.
 */
void kbase_device_put_list(const struct list_head *dev_list);

/**
 * kbase_increment_device_id - increment device id.
 *
 * Used to increment device id on successful initialization of the device.
 */
void kbase_increment_device_id(void);

/**
 * kbase_device_firmware_init_once - Initialize firmware and HWC
 *
 * @kbdev: An instance of the GPU platform device, allocated from the probe
 *         method of the driver.
 *
 * When a device file is opened for the first time,
 * load firmware and initialize hardware counter components.
 *
 * Return: 0 on success. An error code on failure.
 */
int kbase_device_firmware_init_once(struct kbase_device *kbdev);

/**
 * kbase_device_init - Device initialisation.
 *
 * @kbdev: The kbase device structure for the device (must be a valid pointer)
 *
 * This is called from device probe to initialise various other
 * components needed.
 *
 * Return: 0 on success and non-zero value on failure.
 */
int kbase_device_init(struct kbase_device *kbdev);

/**
 * kbase_device_term - Device termination.
 *
 * @kbdev: The kbase device structure for the device (must be a valid pointer)
 *
 * This is called from device remove to terminate various components that
 * were initialised during kbase_device_init.
 */
void kbase_device_term(struct kbase_device *kbdev);

/**
 * kbase_reg_write - write to GPU register
 * @kbdev:  Kbase device pointer
 * @offset: Offset of register
 * @value:  Value to write
 *
 * Caller must ensure the GPU is powered (@kbdev->pm.gpu_powered != false).
 */
void kbase_reg_write(struct kbase_device *kbdev, u32 offset, u32 value);

/**
 * kbase_reg_read - read from GPU register
 * @kbdev:  Kbase device pointer
 * @offset: Offset of register
 *
 * Caller must ensure the GPU is powered (@kbdev->pm.gpu_powered != false).
 *
 * Return: Value in desired register
 */
u32 kbase_reg_read(struct kbase_device *kbdev, u32 offset);

/**
 * kbase_is_gpu_removed() - Has the GPU been removed.
 * @kbdev:    Kbase device pointer
 *
 * When Kbase takes too long to give up the GPU, the Arbiter
 * can remove it.  This will then be followed by a GPU lost event.
 * This function will return true if the GPU has been removed.
 * When this happens register reads will be zero. A zero GPU_ID is
 * invalid so this is used to detect when GPU is removed.
 *
 * Return: True if GPU removed
 */
bool kbase_is_gpu_removed(struct kbase_device *kbdev);

/**
 * kbase_gpu_cache_flush_pa_range_and_busy_wait() - Start a cache physical range flush
 * and busy wait
 *
 * @kbdev:    kbase device to issue the MMU operation on.
 * @phys:     Starting address of the physical range to start the operation on.
 * @nr_bytes: Number of bytes to work on.
 * @flush_op: Flush command register value to be sent to HW
 *
 * Issue a cache flush physical range command, then busy wait an irq status.
 * This function will clear FLUSH_PA_RANGE_COMPLETED irq mask bit
 * and busy-wait the rawstat register.
 *
 * Return: 0 if successful or a negative error code on failure.
 */
#if MALI_USE_CSF
int kbase_gpu_cache_flush_pa_range_and_busy_wait(struct kbase_device *kbdev, phys_addr_t phys,
						 size_t nr_bytes, u32 flush_op);
#endif /* MALI_USE_CSF */

/**
 * kbase_gpu_cache_flush_and_busy_wait - Start a cache flush and busy wait
 * @kbdev: Kbase device
 * @flush_op: Flush command register value to be sent to HW
 *
 * Issue a cache flush command to hardware, then busy wait an irq status.
 * This function will clear CLEAN_CACHES_COMPLETED irq mask bit set by other
 * threads through kbase_gpu_start_cache_clean(), and wake them up manually
 * after the busy-wait is done. Any pended cache flush commands raised by
 * other thread are handled in this function.
 * hwaccess_lock must be held by the caller.
 *
 * Return: 0 if successful or a negative error code on failure.
 */
int kbase_gpu_cache_flush_and_busy_wait(struct kbase_device *kbdev,
					u32 flush_op);

/**
 * kbase_gpu_start_cache_clean - Start a cache clean
 * @kbdev: Kbase device
 * @flush_op: Flush command register value to be sent to HW
 *
 * Issue a given cache flush command to hardware.
 * This function will take hwaccess_lock.
 */
void kbase_gpu_start_cache_clean(struct kbase_device *kbdev, u32 flush_op);

/**
 * kbase_gpu_start_cache_clean_nolock - Start a cache clean
 * @kbdev: Kbase device
 * @flush_op: Flush command register value to be sent to HW
 *
 * Issue a given cache flush command to hardware.
 * hwaccess_lock must be held by the caller.
 */
void kbase_gpu_start_cache_clean_nolock(struct kbase_device *kbdev,
					u32 flush_op);

/**
 * kbase_gpu_wait_cache_clean - Wait for cache cleaning to finish
 * @kbdev: Kbase device
 *
 * This function will take hwaccess_lock, and may sleep.
 */
void kbase_gpu_wait_cache_clean(struct kbase_device *kbdev);

/**
 * kbase_gpu_wait_cache_clean_timeout - Wait for certain time for cache
 *                                      cleaning to finish
 * @kbdev: Kbase device
 * @wait_timeout_ms: Time in milliseconds, to wait for cache clean to complete.
 *
 * This function will take hwaccess_lock, and may sleep. This is supposed to be
 * called from paths (like GPU reset) where an indefinite wait for the
 * completion of cache clean operation can cause deadlock, as the operation may
 * never complete.
 *
 * Return: 0 if successful or a negative error code on failure.
 */
int kbase_gpu_wait_cache_clean_timeout(struct kbase_device *kbdev,
		unsigned int wait_timeout_ms);

/**
 * kbase_gpu_cache_clean_wait_complete - Called after the cache cleaning is
 *                                       finished. Would also be called after
 *                                       the GPU reset.
 * @kbdev: Kbase device
 *
 * Caller must hold the hwaccess_lock.
 */
void kbase_gpu_cache_clean_wait_complete(struct kbase_device *kbdev);

/**
 * kbase_clean_caches_done - Issue previously queued cache clean request or
 *                           wake up the requester that issued cache clean.
 * @kbdev: Kbase device
 *
 * Caller must hold the hwaccess_lock.
 */
void kbase_clean_caches_done(struct kbase_device *kbdev);

/**
 * kbase_gpu_interrupt - GPU interrupt handler
 * @kbdev: Kbase device pointer
 * @val:   The value of the GPU IRQ status register which triggered the call
 *
 * This function is called from the interrupt handler when a GPU irq is to be
 * handled.
 */
void kbase_gpu_interrupt(struct kbase_device *kbdev, u32 val);
