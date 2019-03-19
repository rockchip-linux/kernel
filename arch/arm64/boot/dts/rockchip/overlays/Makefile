# Overlays for the ROCK Pi platform

dtbo-$(CONFIG_ARCH_ROCKCHIP) += \
	console-on-ttyS4.dtbo \
	console-on-ttyS2.dtbo \
	two-color-led.dtbo \
	at24c02.dtbo \
	devspi.dtbo

targets += dtbs dtbs_install
targets += $(dtbo-y)

always		:= $(dtbo-y)
clean-files	:= *.dtbo
