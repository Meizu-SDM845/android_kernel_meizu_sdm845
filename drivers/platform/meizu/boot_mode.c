/*
 * Copyright (C) 2023 transaero21 <transaero21@elseboot.ru>
 *
 * SPDX-License-Identifier: GPL-2.0
 */

#include <linux/meizu/boot_mode.h>

#include <linux/init.h>
#include <linux/printk.h>
#include <linux/string.h>

static int boot_mode = 0;

int get_boot_mode(void) {
	return boot_mode;
}

static int __init boot_mode_setup(char *mode)
{
	if (!strcmp(mode, "normal")) {
		boot_mode = 1;
	} else if (!strcmp(mode, "recovery")) {
		boot_mode = 2;
	} else if (!strcmp(mode, "charger")) {
		boot_mode = 3;
	}
	pr_info("%s: boot_mode = %d\n", __func__, boot_mode);

	return 0;
}
__setup("androidboot.mode=", boot_mode_setup);
