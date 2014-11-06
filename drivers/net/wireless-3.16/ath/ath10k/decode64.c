/*
 * Copyright (c) 2013 The Linux Foundation. All rights reserved.
 * Copyright (c) 2014, The Chromium OS Authors
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#include "decode64.h"

static char revkey[128] = {
	-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
	-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
	-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 62, -1, -1, -1, 63,
	52, 53, 54, 55, 56, 57, 58, 59, 60, 61, -1, -1, -1, -1, -1, -1,
	-1, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14,
	15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, -1, -1, -1, -1, -1,
	-1, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40,
	41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, -1, -1, -1, -1, -1,
};

int
decode64(unsigned char *src, unsigned char *src_end, unsigned char *dst)
{
	unsigned char *dst_end = dst;

	while (&src[3] < src_end) {
		int x;
		int t[4];
		int i;

		if (src[0] == '=' || src[1] == '=' ||
		    (src[2] == '=' && src[3] != '=')) {
			return -1;
		}

		for (i = 0; i < 4; i++) {
			if (src[i] == '=') {
				t[i] = 0;
			} else {
				if (src[i] >= 128 ||
				    ((t[i] = revkey[src[i]]) < 0)) {
					return -1;
				}
			}
		}

		x = (t[0] << 18) + (t[1] << 12) + (t[2] << 6) + t[3];

		*dst_end++ = (x >> 16) & 0xff;
		if (src[2] != '=')
			*dst_end++ = (x >> 8) & 0xff;
		if (src[3] != '=')
			*dst_end++ = x & 0xff;

		src += 4;
	}

	if (src != src_end)
		return -1;

	return dst_end - dst;
}

int
strip_nl(unsigned char *src, unsigned char *src_end, unsigned char *dst)
{
	unsigned char *dst_end = dst;

	while (src < src_end) {
		if (*src != '\n')
			*dst_end++ = *src;
		src++;
	}

	return dst_end - dst;
}
