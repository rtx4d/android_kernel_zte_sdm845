dtb-$(CONFIG_ARCH_QCOM)	+= apq8016-sbc.dtb msm8916-mtp.dtb
dtb-$(CONFIG_ARCH_QCOM)	+= msm8996-mtp.dtb
dtb-$(CONFIG_ARCH_QCOM)	+= apq8096-db820c.dtb
#ZTE_MODIFY delete all qualcomm project dtbs,add zte dtbs

ZTE_BOARD_NAME := akershus
ZTE_DTB = $(notdir $(patsubst %.dts,%.dtb,$(wildcard $(srctree)/arch/arm64/boot/dts/qcom/*$(ZTE_BOARD_NAME).dts)))
ifeq ($(CONFIG_BUILD_ARM64_DT_OVERLAY),y)
ZTE_DTBO = $(patsubst %.dtb,%.dtbo, $(ZTE_DTB))
dtbo-y += $(ZTE_DTBO)
ZTE_DTBO_BASE = $(ZTE_DTBO)-base
$(ZTE_DTBO_BASE) := sdm845-v2.1.dtb
else
dtb-y += $(ZTE_DTB)
endif
$(warning "ZTE_BOARD_NAME= "$(ZTE_BOARD_NAME))
$(warning "ZTE_DTB= "$(ZTE_DTB))
$(warning "ZTE_DTBO= "$(ZTE_DTBO))
$(warning "ZTE_DTBO_BASE= "$(ZTE_DTBO_BASE))

always		:= $(dtb-y)
subdir-y	:= $(dts-dirs)
clean-files	:= *.dtb
