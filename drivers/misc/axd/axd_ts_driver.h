/*
 * Copyright (C) 2014 Imagination Technologies Ltd.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License version
 * 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * AXD timestamp driver interface.
 */
#include <linux/types.h>

/*
 * Called at the beginning of a stream to set the fixed reference point in time
 * for the stream. This should not be implemented if the reference point should
 * not be altered in this way, and is managed through other means.
 */
void __attribute__((weak)) axd_ts_reset(void);

/*
 * Adjusts the provided timestamp from the source to destination formats.
 * @return Non-zero for failure.
 */
int __attribute__((weak)) axd_ts_adjust(u64 *ts);
