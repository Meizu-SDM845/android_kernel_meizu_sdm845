ifeq ($(CONFIG_MEIZU_QCOM),y)
dtb-$(CONFIG_MEIZU_M1882) += m1882-v2.1.dtb
dtb-$(CONFIG_MEIZU_M1892) += m1892-v2.1.dtb
ifeq ($(CONFIG_BUILD_ARM64_DT_OVERLAY),y)
dtbo-$(CONFIG_MEIZU_M1882) += m1882-v2.1-mtp-overlay.dtbo
m1882-v2.1-mtp-overlay.dtbo-base := m1882-v2.1.dtb
dtbo-$(CONFIG_MEIZU_M1892) += m1892-v2.1-mtp-overlay.dtbo
m1892-v2.1-mtp-overlay.dtbo-base := m1892-v2.1.dtb
endif
endif

always		:= $(dtb-y)
subdir-y	:= $(dts-dirs)
clean-files	:= *.dtb
