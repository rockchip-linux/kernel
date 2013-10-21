# common
obj-$(CPTCFG_IWLWIFI)	+= iwlwifi.o
iwlwifi-objs		+= iwl-io.o
iwlwifi-objs		+= iwl-drv.o
iwlwifi-objs		+= iwl-debug.o
iwlwifi-objs		+= iwl-notif-wait.o
iwlwifi-objs		+= iwl-eeprom-read.o iwl-eeprom-parse.o
iwlwifi-objs		+= iwl-phy-db.o iwl-nvm-parse.o

# Bus
iwlwifi-$(CPTCFG_IWLWIFI_PCIE)	+= pcie/drv.o pcie/rx.o pcie/tx.o pcie/trans.o
iwlwifi-$(CPTCFG_IWLWIFI_PCIE)-$(CPTCFG_IWLMVM) += iwl-7000.o

iwlwifi-objs += $(iwlwifi-y-y) $(iwlwifi-y-m)

iwlwifi-$(CPTCFG_IWLWIFI_DEVICE_TRACING) += iwl-devtrace.o
iwlwifi-$(CPTCFG_IWLWIFI_DEVICE_TESTMODE) += iwl-tm-gnl.o

ccflags-y += -D__CHECK_ENDIAN__ -I$(src)

obj-$(CPTCFG_IWLMVM)	+= mvm/

CFLAGS_iwl-devtrace.o := -I$(src)
