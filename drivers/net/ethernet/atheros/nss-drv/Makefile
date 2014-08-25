##########################################################################
# Copyright (c) 2013-2014, The Linux Foundation. All rights reserved.
# Permission to use, copy, modify, and/or distribute this software for
# any purpose with or without fee is hereby granted, provided that the
# above copyright notice and this permission notice appear in all copies.
# THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
# WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
# MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
# ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
# WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
# ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT
# OF OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
##########################################################################

# ###################################################
# # Makefile for the NSS driver
# ###################################################

obj-m += qca-nss-drv.o

#
# List the files that belong to the driver in alphabetical order.
#
qca-nss-drv-objs := \
			nss_cmn.o \
			nss_core.o \
			nss_crypto.o \
			nss_if.o \
			nss_init.o \
			nss_ipsec.o \
			nss_ipv4.o \
			nss_ipv6.o \
			nss_lag.o \
			nss_phys_if.o \
			nss_pm.o \
			nss_stats.o \
			nss_tun6rd.o \
			nss_tunipip6.o \
			nss_virt_if.o \
			nss_shaper.o

#
# TODO: Deprecated files should be removed before merge
#
qca-nss-drv-objs += \
			nss_tx_rx_crypto.o \
			nss_tx_rx_eth_rx.o \
			nss_tx_rx_generic.o \
			nss_tx_rx_ipv4.o \
			nss_tx_rx_ipv6.o \
			nss_tx_rx_ipsec.o \
			nss_tx_rx_n2h.o \
			nss_tx_rx_pppoe.o \
			nss_tx_rx_phys_if.o \
			nss_tx_rx_virt_if.o

ifeq ("$(KERNELVERSION)","3.4.0")
obj-m += qca-nss-connmgr-ipv4.o
obj-m += qca-nss-connmgr-ipv6.o
obj-m += qca-nss-tunipip6.o
ifeq "$(CONFIG_IPV6_SIT_6RD)" "y"
obj-m += qca-nss-tun6rd.o
qca-nss-tun6rd-objs := nss_connmgr_tun6rd.o
ccflags-y += -DNSS_TUN6RD_DEBUG_LEVEL=0
endif
obj-m += qca-nss-qdisc.o
qca-nss-connmgr-ipv4-objs := nss_connmgr_ipv4.o nss_connmgr_lag.o
qca-nss-connmgr-ipv6-objs := nss_connmgr_ipv6.o
qca-nss-tunipip6-objs := nss_connmgr_tunipip6.o
qca-nss-qdisc-objs := nss_qdisc.o
endif

ccflags-y += -I$(obj)/nss_hal/include -I$(obj)/exports -DNSS_DEBUG_LEVEL=0 -DNSS_EMPTY_BUFFER_SIZE=1792 -DNSS_PKT_STATS_ENABLED=0 -DNSS_DT_SUPPORT=1 -DNSS_PPP_SUPPORT=0 -DNSS_FW_DBG_SUPPORT=0 -DNSS_PM_SUPPORT=0
ccflags-y += -DNSS_CONNMGR_DEBUG_LEVEL=0 -DNSS_CONNMGR_PPPOE_SUPPORT=0
ccflags-y += -DNSS_TUNIPIP6_DEBUG_LEVEL=0
ccflags-y += -DNSS_PM_DEBUG_LEVEL=0
ccflags-y += -I$(TOPDIR)/qca/src/linux/net/bridge -DNSSQDISC_DEBUG_LEVEL=0 -I$(obj)

ifeq ("$(NSS_PM_SUPPORT)","1")
qca-nss-drv-objs += nss_tx_rx_freq.o
endif

obj ?= .

qca-nss-drv-objs += nss_hal/ipq806x/nss_hal_pvt.o
ccflags-y += -I$(obj)/nss_hal/ipq806x
