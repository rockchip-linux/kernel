/*
 *  drivers/input/touchscreen/gsl_point_id.h
 *
 *  Copyright (c) 2012 Shanghai Basewin
 *  Guan Yuwei<guanyuwei@basewin.com>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */

#ifndef _GSL_POINT_ID_H
#define _GSL_POINT_ID_H

unsigned int gsl_mask_tiaoping(void);
unsigned int gsl_version_id(void);
void gsl_alg_id_main(struct gsl_touch_info *cinfo);
void gsl_DataInit(int *ret);

#endif /* _GSL_POINT_ID_H */
