dtb-$(CONFIG_ARCH_VEXPRESS) += rtsm_ve-aemv8a.dtb foundation-v8.dtb
dtb-$(CONFIG_ARCH_XGENE) += apm-mustang.dtb
dtb-$(CONFIG_ARCH_TEGRA) += tegra132-norrin.dtb

targets += dtbs
targets += $(dtb-y)

dtbs: $(addprefix $(obj)/, $(dtb-y))

clean-files := *.dtb
