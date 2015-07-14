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

#ifndef __VPD_DECODE_H
#define __VPD_DECODE_H

#include <linux/types.h>

enum {
	VPD_OK = 0,
	VPD_FAIL,
};

enum {
	VPD_TYPE_TERMINATOR = 0,
	VPD_TYPE_STRING,
	VPD_TYPE_INFO                = 0xfe,
	VPD_TYPE_IMPLICIT_TERMINATOR = 0xff,
};

/* Callback for decode_vpd_string to invoke. */
typedef int vpd_decode_callback(const uint8_t *key, int32_t key_len,
		const uint8_t *value, int32_t value_len, void *arg);

/* Given the encoded string, this function invokes callback with extracted
 * (key, value). The *consumed will be plused the number of bytes consumed in
 * this function.
 *
 * The input_buf points to the first byte of the input buffer.
 *
 * The *consumed starts from 0, which is actually the next byte to be decoded.
 * It can be non-zero to be used in multiple calls.
 *
 * If one entry is successfully decoded, sends it to callback and returns the
 * result.
 */
int decode_vpd_string(const int32_t max_len, const uint8_t *input_buf,
		int32_t *consumed, vpd_decode_callback callback,
		void *callback_arg);

#endif  /* __VPD_DECODE_H */
