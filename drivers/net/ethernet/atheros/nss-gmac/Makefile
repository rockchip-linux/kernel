# Copyright (c) 2013, The Linux Foundation. All rights reserved.
#
# Permission to use, copy, modify, and/or distribute this software for any
# purpose with or without fee is hereby granted, provided that the above
# copyright notice and this permission notice appear in all copies.
#
#
# THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
# WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF MERCHANTABILITY
# AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY SPECIAL, DIRECT
# INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM
# LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE
# OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR
# PERFORMANCE OF THIS SOFTWARE.

###################################################
# Makefile for the NSS GMAC driver
###################################################

obj ?= .

obj-m += qca-nss-gmac.o

qca-nss-gmac-objs +=	nss_gmac_dev.o \
			nss_gmac_ctrl.o \
			nss_gmac_init.o \
			nss_gmac_ethtool.o \
			nss_gmac_tx_rx_offload.o \
			nss_gmac_mdiobus.o

GMAC_INCLUDE = -I$(obj)/include

# pick up nss_api_if.h
GMAC_INCLUDE += -I$(src)/../nss-drv/exports

# NSS_GMAC_DEBUG_LEVEL - 0: none, 1: warning, 2: info, 3: trace

ccflags-y += $(GMAC_INCLUDE) -DNSS_GMAC_DEBUG_LEVEL=0 -DNSS_MACSEC_SUPPORT=0 -DNSS_GMAC_DT_SUPPORT=1

