/*
 *
 * Copyright (c) 2016, Fuzhou Rockchip Electronics Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef	__TYPE_DEF_H
#define	__TYPE_DEF_H

#ifndef	NULL
#define	NULL	0
#endif
#define	OK	0
#define	ERROR	(-1)

#define	FTL_ERROR	ERROR
#define	FTL_OK		OK
#define	FTL_NO_FLASH	-2
#define	FTL_NO_IDB	-3

#define	FALSE		0
#define	TRUE		(!FALSE)

#define	INVALID_UINT8	((u8)0xFF)
#define	INVALID_UINT16	((u16)0xFFFF)
#define	INVALID_UINT32	((u32)0xFFFFFFFFL)


#define	MIN(x, y) ((x) < (y) ? (x) : (y))
#define	MAX(x, y) ((x) > (y) ? (x) : (y))


void *ftl_malloc(int n_size);
void *ftl_memset(void *s, int c, unsigned int n);
void *ftl_memcpy(void *pv_to,
		 const void *pv_from,
		 unsigned int size);
#define PRINT_E	pr_err
void rknand_print_hex(char *s, void *buf, int width, int len);
#endif  /*__TYPEDEF_H */
