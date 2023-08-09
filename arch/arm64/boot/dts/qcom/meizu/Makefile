ifeq ($(CONFIG_MEIZU_QCOM),y)
dtb-$(CONFIG_MEIZU_M1882) += meizu-m1882.dtb
dtb-$(CONFIG_MEIZU_M1892) += meizu-m1892.dtb
ifeq ($(CONFIG_BUILD_ARM64_DT_OVERLAY),y)
dtbo-$(CONFIG_MEIZU_M1882) += meizu-m1882-overlay.dtbo
meizu-m1882-overlay.dtbo-base := meizu-m1882.dtb
dtbo-$(CONFIG_MEIZU_M1892) += meizu-m1892-overlay.dtbo
meizu-m1892-overlay.dtbo-base := meizu-m1892.dtb
endif
endif

always		:= $(dtb-y)
subdir-y	:= $(dts-dirs)
clean-files	:= *.dtb
