obj-$(CONFIG_DW_DMAC_CORE)	+= dw_dmac_core.o
dw_dmac_core-objs	:= core.o

ifneq ($(CONFIG_DW_DMAC_PRIVATE),y)
	dw_dmac_core-objs	+= private.o
endif

obj-$(CONFIG_DW_DMAC)		+= dw_dmac.o
dw_dmac-objs		:= platform.o

obj-$(CONFIG_DW_DMAC_PCI)	+= dw_dmac_pci.o
dw_dmac_pci-objs	:= pci.o
