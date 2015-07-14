/*
 * vpd_decode.c
 *
 * Google VPD decoding routines.
 *
 * Copyright 2015 Google Inc.
 *
 * This file is taken from the ChromiumOS VPD project and modified to
 * match the kernel coding style.
 *
 * The license is attached below and can be found in /LICENSE of the chromiumos
 * source repository.
 *
 * Copyright (c) 2006-2009 The Chromium OS Authors. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 *    * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 *    * Redistributions in binary form must reproduce the above
 * copyright notice, this list of conditions and the following disclaimer
 * in the documentation and/or other materials provided with the
 * distribution.
 *    * Neither the name of Google Inc. nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
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
 */

#include "vpd_decode.h"

#include <linux/export.h>

static int decode_len(const int32_t max_len, const uint8_t *in,
		int32_t *length, int32_t *decoded_len)
{
	uint8_t more;
	int i = 0;

	if (!length)
		return VPD_FAIL;

	if (!decoded_len)
		return VPD_FAIL;

	*length = 0;
	do {
		if (i >= max_len)
			return VPD_FAIL;
		more = in[i] & 0x80;
		*length <<= 7;
		*length |= in[i] & 0x7f;
		++i;
	} while (more);

	*decoded_len = i;

	return VPD_OK;
}

/* Sequentially decodes type, key, and value. */
int decode_vpd_string(const int32_t max_len, const uint8_t *input_buf,
		int32_t *consumed, vpd_decode_callback callback,
		void *callback_arg)
{
	int type;
	int32_t key_len, value_len;
	int32_t decoded_len;
	const uint8_t *key, *value;

	/* type */
	if (*consumed >= max_len)
		return VPD_FAIL;

	type = input_buf[*consumed];
	switch (type) {
	case VPD_TYPE_INFO:
	case VPD_TYPE_STRING:
		(*consumed)++;

		/* key */
		if (VPD_OK != decode_len(max_len - *consumed,
					&input_buf[*consumed], &key_len,
					&decoded_len) ||
				*consumed + decoded_len >= max_len) {
			return VPD_FAIL;
		}

		*consumed += decoded_len;
		key = &input_buf[*consumed];
		*consumed += key_len;

		/* value */
		if (VPD_OK != decode_len(max_len - *consumed,
					&input_buf[*consumed], &value_len,
					&decoded_len) ||
				*consumed + decoded_len > max_len) {
			return VPD_FAIL;
		}
		*consumed += decoded_len;
		value = &input_buf[*consumed];
		*consumed += value_len;

		if (type == VPD_TYPE_STRING)
			return callback(key, key_len, value, value_len,
					callback_arg);

		return VPD_OK;

	default:
		return VPD_FAIL;
	}
	return VPD_OK;
}
EXPORT_SYMBOL(decode_vpd_string);
