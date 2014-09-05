dtb-$(CONFIG_ARCH_VEXPRESS) += rtsm_ve-aemv8a.dtb foundation-v8.dtb
dtb-$(CONFIG_ARCH_XGENE) += apm-mustang.dtb
dtb-$(CONFIG_ARCH_TEGRA) += tegra132-norrin.dtb
dtb-$(CONFIG_ARCH_TEGRA) += tegra132-ryu-p0.dtb
dtb-$(CONFIG_ARCH_TEGRA) += tegra132-ryu-p1.dtb

targets += dtbs dtbs_install
targets += $(dtb-y)

dtbs: $(addprefix $(obj)/, $(dtb-y))

clean-files := *.dtb
