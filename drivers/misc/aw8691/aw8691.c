/*
 * aw8691.c   aw8691 haptic module
 *
 * Version: v1.0.0
 *
 * Copyright (c) 2017 AWINIC Technology CO., LTD
 * Copyright (C) 2023 transaero21 <transaero21@elseboot.ru>
 *
 *  Author: Nick Li <liweilei@awinic.com.cn>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/firmware.h>
#include <linux/slab.h>
#include <linux/version.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/debugfs.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <linux/syscalls.h>
#include <linux/power_supply.h>

#include "aw8691.h"
#include "aw8691_reg.h"

/******************************************************
 *
 * Marco
 *
 ******************************************************/
#define AW8691_I2C_NAME "aw8691_haptic"
#define AW8691_HAPTIC_NAME "aw8691_haptic"

#define AW8691_VERSION "v1.0.0"

/* #define AWINIC_I2C_REGMAP */
#define AWINIC_RAM_UPDATE_DELAY

#define AW_I2C_RETRIES 2
#define AW_I2C_RETRY_DELAY 2
#define AW_READ_CHIPID_RETRIES 5
#define AW_READ_CHIPID_RETRY_DELAY 2

/******************************************************
 *
 * variable
 *
 ******************************************************/
static char *aw8691_ram_name = "aw_170hz.bin";
static char *aw8691_rtp_name = "sms_ring01.bin";

static struct vibrator_combin aw8691_vibrator_combin[] = {
	{0, {19, 19, 19, 19}, {255, 255}},
	{20120, {8, 0, 0, 0}, {0, 0}},
	{20200, {13, 0, 0, 0}, {0, 0}},
	{20300, {20, 18, 15, 0}, {8, 0}},
	{21000, {11, 0, 0, 0}, {0, 0}},
	{21020, {8, 0, 0, 0}, {0, 0}},
	{21230, {9, 0, 0, 0}, {0, 0}},
	{21240, {9, 0, 0, 0}, {0, 0}},
	{21251, {0, 0, 0, 0}, {0, 0}},
	{21600, {4, 0, 0, 0}, {0, 0}},
	{21900, {9, 0, 0, 0}, {0, 0}},
	{22140, {0, 0, 0, 0}, {0, 0}},
	{22160, {0, 0, 0, 0}, {0, 0}},
	{22500, {12, 0, 0, 0}, {0, 0}},
	{22502, {12, 0, 0, 0}, {0, 0}},
	{22507, {12, 0, 0, 0}, {0, 0}},
	{22509, {12, 0, 0, 0}, {0, 0}},
	{22510, {4, 0, 0, 0}, {0, 0}},
	{22520, {11, 0, 0, 0}, {0, 0}},
	{22550, {12, 0, 0, 0}, {0, 0}},
	{22560, {12, 0, 0, 0}, {0, 0}},
	{25160, {4, 0, 0, 0}, {0, 0}},
	{25550, {4, 0, 0, 0}, {0, 0}},
	{25688, {11, 0, 0, 0}, {0, 0}},
	{30200, {16, 16, 15, 0}, {0, 0}},
	{30201, {0, 0, 0, 0}, {0, 0}},
	{30203, {0, 0, 0, 0}, {0, 0}},
	{30500, {0, 0, 0, 0}, {0, 0}},
	{30800, {10, 0, 0, 0}, {0, 0}},
	{30900, {3, 0, 0, 0}, {0, 0}},
	{31001, {16, 0, 0, 0}, {0, 0}},
	{31003, {129, 3, 139, 3}, {0, 0}},
	{31004, {0, 0, 0, 0}, {0, 0}},
	{31005, {0, 0, 0, 0}, {0, 0}},
	{31006, {3, 0, 0, 0}, {0, 0}},
	{31007, {0, 0, 0, 0}, {0, 0}},
	{31008, {13, 0, 0, 0}, {0, 0}},
	{31009, {0, 0, 0, 0}, {0, 0}},
	{31010, {0, 0, 0, 0}, {0, 0}},
	{31011, {7, 0, 0, 0}, {0, 0}},
	{31012, {0, 0, 0, 0}, {0, 0}},
	{31013, {129, 3, 139, 3}, {0, 0}},
	{31014, {10, 0, 0, 0}, {0, 0}},
	{31015, {4, 0, 0, 0}, {0, 0}},
	{31016, {8, 0, 0, 0}, {0, 0}},
	{31017, {4, 0, 0, 0}, {0, 0}},
	{31018, {4, 0, 0, 0}, {0, 0}},
	{31019, {4, 0, 0, 0}, {0, 0}},
	{31020, {8, 0, 0, 0}, {0, 0}},
	{31021, {8, 0, 0, 0}, {0, 0}},
	{31022, {6, 0, 0, 0}, {0, 0}},
};

struct aw8691_container *aw8691_rtp;
struct aw8691 *g_aw8691;

unsigned int play = 0;

/******************************************************
 *
 * functions
 *
 ******************************************************/
static void aw8691_interrupt_clear(struct aw8691 *aw8691);

 /******************************************************
 *
 * aw8691 i2c write/read
 *
 ******************************************************/
static int aw8691_i2c_write(struct aw8691 *aw8691,
	unsigned char reg_addr, unsigned char reg_data)
{
	int ret = -1;
	unsigned char cnt = 0;

	while(cnt < AW_I2C_RETRIES) {
#ifdef AWINIC_I2C_REGMAP
		ret = regmap_write(aw8691->regmap, reg_addr, reg_data);
		if (ret < 0) {
			pr_err("%s: regmap_write cnt=%d error=%d\n",
			       __func__, cnt, ret);
		} else {
			break;
		}
#else
		ret = i2c_smbus_write_byte_data(aw8691->i2c, reg_addr, reg_data);
		if (ret < 0) {
			pr_err("%s: i2c_write cnt=%d error=%d\n",
			       __func__, cnt, ret);
		} else {
			break;
		}
#endif
		cnt ++;
		msleep(AW_I2C_RETRY_DELAY);
	}

	return ret;
}

static int aw8691_i2c_read(struct aw8691 *aw8691,
	unsigned char reg_addr, unsigned char *reg_data)
{
	int ret = -1;
	unsigned char cnt = 0;

	while(cnt < AW_I2C_RETRIES) {
#ifdef AWINIC_I2C_REGMAP
		ret = regmap_read(aw8691->regmap, reg_addr, reg_data);
		if (ret < 0) {
			pr_err("%s: regmap_read cnt=%d error=%d\n",
			       __func__, cnt, ret);
		} else {
			break;
		}
#else
		ret = i2c_smbus_read_byte_data(aw8691->i2c, reg_addr);
		if (ret < 0) {
			pr_err("%s: i2c_read cnt=%d error=%d\n",
			       __func__, cnt, ret);
		} else {
			*reg_data = ret;
			break;
		}
#endif
		cnt ++;
		msleep(AW_I2C_RETRY_DELAY);
	}

	return ret;
}

static int aw8691_i2c_write_bits(struct aw8691 *aw8691,
	unsigned char reg_addr, unsigned int mask, unsigned char reg_data)
{
	unsigned char reg_val = 0;

	aw8691_i2c_read(aw8691, reg_addr, &reg_val);
	reg_val &= mask;
	reg_val |= reg_data;
	aw8691_i2c_write(aw8691, reg_addr, reg_val);

	return 0;
}

static int aw8691_i2c_writes(struct aw8691 *aw8691,
	unsigned char reg_addr, unsigned char *buf, unsigned int len)
{
	int ret = -1;
	unsigned char *data;

	data = kmalloc(len + 1, GFP_KERNEL);
	if (data == NULL) {
		pr_err("%s: can not allocate memory\n", __func__);
		return  -ENOMEM;
	}

	data[0] = reg_addr;
	memcpy(&data[1], buf, len);

	ret = i2c_master_send(aw8691->i2c, data, len + 1);
	if (ret < 0) {
		pr_err("%s: i2c master send error\n", __func__);
	}

	kfree(data);

	return ret;
}

/*****************************************************
 *
 * ram update
 *
 *****************************************************/
static void aw8691_rtp_loaded(const struct firmware *cont, void *context)
{
	struct aw8691 *aw8691 = context;
	pr_debug("%s enter\n", __func__);

	if (!cont) {
		pr_err("%s: failed to read %s\n", __func__, aw8691_rtp_name);
		release_firmware(cont);
		return;
	}

	pr_debug("%s: loaded %s - size: %zu\n", __func__, aw8691_rtp_name,
		 cont ? cont->size : 0);

	/* aw8691 rtp update */
	aw8691_rtp = kzalloc(cont->size + sizeof(int), GFP_KERNEL);
	if (!aw8691_rtp) {
		release_firmware(cont);
		pr_err("%s: Error allocating memory\n", __func__);
		return;
	}
	aw8691_rtp->len = cont->size;
	pr_debug("%s: rtp size = %d\n", __func__, aw8691_rtp->len);
	memcpy(aw8691_rtp->data, cont->data, cont->size);
	release_firmware(cont);

	aw8691->rtp_init = 1;
	pr_debug("%s: rtp update complete\n", __func__);
}

static int aw8691_rtp_update(struct aw8691 *aw8691)
{
	pr_debug("%s enter\n", __func__);

	return request_firmware_nowait(THIS_MODULE, FW_ACTION_HOTPLUG,
				       aw8691_rtp_name, aw8691->dev,
				       GFP_KERNEL, aw8691, aw8691_rtp_loaded);
}


static void aw8691_container_update(struct aw8691 *aw8691,
	struct aw8691_container *aw8691_cont)
{
	int i = 0;
	unsigned int shift = 0;

	pr_debug("%s enter\n", __func__);

	aw8691->ram.baseaddr_shift = 2;
	aw8691->ram.ram_shift = 4;

	aw8691_i2c_write_bits(aw8691, AW8691_REG_SYSCTRL,
			      AW8691_BIT_SYSCTRL_RAMINIT_MASK,
			      AW8691_BIT_SYSCTRL_RAMINIT_EN);

	shift = aw8691->ram.baseaddr_shift;
	aw8691->ram.base_addr = (unsigned int)((aw8691_cont->data[0 + shift] << 8) |
					       (aw8691_cont->data[1 + shift]));
	pr_debug("%s: base_addr=0x%4x\n", __func__, aw8691->ram.base_addr);

	aw8691_i2c_write(aw8691, AW8691_REG_BASE_ADDRH,
			 aw8691_cont->data[0 + shift]);
	aw8691_i2c_write(aw8691, AW8691_REG_BASE_ADDRL,
			 aw8691_cont->data[1 + shift]);

	aw8691_i2c_write(aw8691, AW8691_REG_FIFO_AEH, 0x06);
	aw8691_i2c_write(aw8691, AW8691_REG_FIFO_AEL,
			 (unsigned char)(aw8691->ram.base_addr >> 2));
	aw8691_i2c_write(aw8691, AW8691_REG_FIFO_AFH, 0x0A);
	aw8691_i2c_write(aw8691, AW8691_REG_FIFO_AFL,
			 (unsigned char)(aw8691->ram.base_addr -
			 		(aw8691->ram.base_addr >> 2)));

	shift = aw8691->ram.baseaddr_shift;
	aw8691_i2c_write(aw8691, AW8691_REG_RAMADDRH,
			 aw8691_cont->data[0 + shift]);
	aw8691_i2c_write(aw8691, AW8691_REG_RAMADDRL,
			 aw8691_cont->data[1 + shift]);
	shift = aw8691->ram.ram_shift;
	for (i = shift; i < aw8691_cont->len; i++) {
		aw8691_i2c_write(aw8691, AW8691_REG_RAMDATA, aw8691_cont->data[i]);
	}

	aw8691_i2c_write_bits(aw8691, AW8691_REG_SYSCTRL,
			      AW8691_BIT_SYSCTRL_RAMINIT_MASK,
			      AW8691_BIT_SYSCTRL_RAMINIT_OFF);

	pr_debug("%s exit\n", __func__);
}

static void aw8691_ram_loaded(const struct firmware *cont, void *context)
{
	struct aw8691 *aw8691 = context;
	struct aw8691_container *aw8691_fw;
	int i = 0;
	unsigned short check_sum = 0;

	pr_debug("%s enter\n", __func__);

	if (!cont) {
		pr_err("%s: failed to read %s\n", __func__, aw8691_ram_name);
		release_firmware(cont);
		return;
	}

	pr_debug("%s: loaded %s - size: %zu\n", __func__, aw8691_ram_name,
		 cont ? cont->size : 0);

	for (i = 2; i < cont->size; i++) {
		check_sum += cont->data[i];
	}
	if (check_sum != (unsigned short)((cont->data[0] << 8) | (cont->data[1]))) {
		pr_err("%s: check sum err: check_sum=0x%04x\n",
		       __func__, check_sum);
		return;
	} else {
		pr_debug("%s: check sum pass : 0x%04x\n", __func__, check_sum);
		aw8691->ram.check_sum = check_sum;
	}

	aw8691_fw = kzalloc(cont->size + sizeof(int), GFP_KERNEL);
	if (!aw8691_fw) {
		release_firmware(cont);
		pr_err("%s: Error allocating memory\n", __func__);
		return;
	}
	aw8691_fw->len = cont->size;
	memcpy(aw8691_fw->data, cont->data, cont->size);
	release_firmware(cont);

	aw8691_container_update(aw8691, aw8691_fw);

	aw8691->ram.len = aw8691_fw->len;

	kfree(aw8691_fw);

	aw8691->ram_init = 1;
	pr_debug("%s: fw update complete\n", __func__);

	aw8691_rtp_update(aw8691);
}

static int aw8691_ram_update(struct aw8691 *aw8691)
{
	aw8691->ram_init = 0;
	aw8691->rtp_init = 0;
	return request_firmware_nowait(THIS_MODULE, FW_ACTION_HOTPLUG,
				       aw8691_ram_name, aw8691->dev,
				       GFP_KERNEL, aw8691, aw8691_ram_loaded);
}

#ifdef AWINIC_RAM_UPDATE_DELAY
static enum hrtimer_restart aw8691_ram_timer_func(struct hrtimer *timer)
{
	struct aw8691 *aw8691 = container_of(timer, struct aw8691, ram_timer);

	pr_info("%s enter\n", __func__);

	schedule_work(&aw8691->ram_work);

	return HRTIMER_NORESTART;
}

static void aw8691_ram_work_routine(struct work_struct *work)
{
	struct aw8691 *aw8691 = container_of(work, struct aw8691, ram_work);

	pr_info("%s enter\n", __func__);

	aw8691_ram_update(aw8691);
}
#endif

static int aw8691_ram_init(struct aw8691 *aw8691)
{
#ifdef AWINIC_RAM_UPDATE_DELAY
	int ram_timer_val = 5000;

	hrtimer_init(&aw8691->ram_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	aw8691->ram_timer.function = aw8691_ram_timer_func;
	INIT_WORK(&aw8691->ram_work, aw8691_ram_work_routine);
	hrtimer_start(&aw8691->ram_timer,
		      ktime_set(ram_timer_val / 1000,
				(ram_timer_val % 1000) * 1000000),
		      HRTIMER_MODE_REL);
#else
	aw8691_ram_update(aw8691);
#endif
	return 0;
}



/*****************************************************
 *
 * haptic control
 *
 *****************************************************/
static int aw8691_haptic_play_mode(struct aw8691 *aw8691, unsigned char play_mode)
{
	switch (play_mode) {
	case AW8691_HAPTIC_RAM_MODE:
		aw8691->play_mode = AW8691_HAPTIC_RAM_MODE;
		aw8691_i2c_write_bits(aw8691, AW8691_REG_SYSCTRL,
				      AW8691_BIT_SYSCTRL_PLAY_MODE_MASK,
				      AW8691_BIT_SYSCTRL_PLAY_MODE_RAM);
		break;
	case AW8691_HAPTIC_RTP_MODE:
		aw8691->play_mode = AW8691_HAPTIC_RTP_MODE;
		aw8691_i2c_write_bits(aw8691, AW8691_REG_SYSCTRL,
				      AW8691_BIT_SYSCTRL_PLAY_MODE_MASK,
				      AW8691_BIT_SYSCTRL_PLAY_MODE_RTP);
		break;
	case AW8691_HAPTIC_TRIG_MODE:
		aw8691->play_mode = AW8691_HAPTIC_TRIG_MODE;
		aw8691_i2c_write_bits(aw8691, AW8691_REG_SYSCTRL,
				      AW8691_BIT_SYSCTRL_PLAY_MODE_MASK,
				      AW8691_BIT_SYSCTRL_PLAY_MODE_RAM);
		break;
	default:
		dev_err(aw8691->dev, "%s: play mode %d err",
			__func__, play_mode);
		break;
	}
	return 0;
}

static int aw8691_haptic_stop(struct aw8691 *aw8691)
{
	aw8691_i2c_write_bits(aw8691, AW8691_REG_SYSCTRL,
			      AW8691_BIT_SYSCTRL_WORK_MODE_MASK,
			      AW8691_BIT_SYSCTRL_STANDBY);
	return 0;
}

static int aw8691_haptic_start(struct aw8691 *aw8691)
{
	aw8691_i2c_write_bits(aw8691, AW8691_REG_SYSCTRL,
			      AW8691_BIT_SYSCTRL_WORK_MODE_MASK,
			      AW8691_BIT_SYSCTRL_ACTIVE);
	aw8691_i2c_write(aw8691, AW8691_REG_PROC_CTRL,
			 AW8691_BIT_PROC_CTRL_GO);
	return 0;
}

static int aw8691_haptic_set_repeat_seq(struct aw8691 *aw8691,
	unsigned char flag)
{
	if (flag) {
		aw8691_i2c_write_bits(aw8691, AW8691_REG_DATACTRL,
				      AW8691_BIT_DATACTRL_WAV_DBG_MASK,
				      AW8691_BIT_DATACTRL_WAV_DBG);
	} else {
		aw8691_i2c_write_bits(aw8691, AW8691_REG_DATACTRL,
				      AW8691_BIT_DATACTRL_WAV_DBG_MASK,
				      AW8691_BIT_DATACTRL_WAV_NORMAL);
	}

	return 0;
}

static int aw8691_haptic_set_repeat_que_seq(struct aw8691 *aw8691,
	unsigned char seq)
{
	unsigned char i;
	for (i = 0; i < AW8691_SEQUENCER_SIZE; i++) {
		aw8691_i2c_write(aw8691, AW8691_REG_QUE_SEQ1 + i, seq);
	}
	return 0;
}

static int aw8691_haptic_set_que_seq(struct aw8691 *aw8691, unsigned int seq)
{
	unsigned char i = 0;
	struct aw8691_que_seq que_seq;

	for (i = 0; i < AW8691_SEQUENCER_SIZE; i++) {
		que_seq.index[i] = (seq >> ((AW8691_SEQUENCER_SIZE - i - 1) * 8)) & 0xFF;
	}

	for (i = 0; i < AW8691_SEQUENCER_SIZE; i++) {
		aw8691_i2c_write(aw8691, AW8691_REG_QUE_SEQ1 + i,
				 que_seq.index[i]);
	}
	return 0;
}

static int aw8691_haptic_set_que_loop(struct aw8691 *aw8691, unsigned int loop)
{
	unsigned char i = 0;
	unsigned char tmp[2] = {0, 0 };

	tmp[0] |= (loop>>(24-4))&0xF0;
	tmp[0] |= (loop>>(16-0))&0x0F;
	tmp[1] |= (loop>>( 8-4))&0xF0;
	tmp[1] |= (loop>>( 0-0))&0x0F;

	for(i = 0; i < AW8691_SEQUENCER_LOOP_SIZE; i++) {
		aw8691_i2c_write(aw8691, AW8691_REG_SEQ_LOOP1 + i, tmp[i]);
	}

	return 0;
}


static int aw8691_set_que_seq(struct aw8691 *aw8691, unsigned long arg)
{
	int ret = 0;
	unsigned char i = 0;
	struct aw8691_que_seq que_seq;

	if (copy_from_user(&que_seq, (void __user *)arg,
			   sizeof(struct aw8691_que_seq)))
		return -EFAULT;

	for (i = 0; i < AW8691_SEQUENCER_SIZE; i++) {
		aw8691_i2c_write(aw8691, AW8691_REG_QUE_SEQ1 + i,
				 que_seq.index[i]);
	}

	return ret;
}

static int aw8691_set_seq_loop(struct aw8691 *aw8691, unsigned long arg)
{
	struct aw8691_seq_loop seq_loop;
	int ret = 0;
	unsigned char i = 0;
	unsigned char loop[2] = {0, 0};

	if (copy_from_user(&seq_loop, (void __user *)arg,
			   sizeof(struct aw8691_seq_loop)))
		return -EFAULT;

	for (i = 0; i < AW8691_SEQUENCER_SIZE; i++) {
		if (seq_loop.loop[i] & 0xF0) {
			dev_err(aw8691->dev, "%s: seq_loop err loop[%d]=0x%02x\n",
				__func__, i, seq_loop.loop[i]);
		return -1;
		}
	}
	loop[0] |= (seq_loop.loop[0] << 4);
	loop[0] |= (seq_loop.loop[1] << 0);
	loop[1] |= (seq_loop.loop[2] << 4);
	loop[1] |= (seq_loop.loop[3] << 0);

	for(i = 0; i < (AW8691_SEQUENCER_SIZE >> 1); i++) {
		aw8691_i2c_write(aw8691, AW8691_REG_SEQ_LOOP1 + i, loop[i]);
	}

	return ret;
}

static int aw8691_haptic_set_bst_vol(struct aw8691 *aw8691, unsigned char bst_vol)
{
	aw8691_i2c_write_bits(aw8691, AW8691_REG_BSTCFG,
			      AW8691_BIT_BSTCFG_BSTVOL_MASK,
			      bst_vol);
	return 0;
}

static int aw8691_haptic_set_bst_peak_cur(struct aw8691 *aw8691, unsigned char peak_cur)
{
	aw8691_i2c_write_bits(aw8691, AW8691_REG_BSTCFG,
			      AW8691_BIT_BSTCFG_PEAKCUR_MASK,
			      peak_cur);
	return 0;
}

static int aw8691_haptic_set_gain(struct aw8691 *aw8691, unsigned char gain)
{
	if (gain & 0x80) {
		gain = 0x7F;
	}

	aw8691_i2c_write(aw8691, AW8691_REG_DATDBG, gain);

	return 0;
}

#ifdef AW8691_HAPTIC_VBAT_MONITOR
static int aw8691_haptic_set_bst_mode(struct aw8691 *aw8691,
	unsigned char mode)
{
	if (mode == AW8691_HAPTIC_BYPASS_MODE) {
		aw8691_i2c_write_bits(aw8691, AW8691_REG_SYSCTRL,
				      AW8691_BIT_SYSCTRL_BST_MODE_MASK,
				      AW8691_BIT_SYSCTRL_BST_MODE_BYPASS);
	} else if (mode == AW8691_HAPTIC_BOOST_MODE){
		aw8691_i2c_write_bits(aw8691, AW8691_REG_SYSCTRL,
				      AW8691_BIT_SYSCTRL_BST_MODE_MASK,
				      AW8691_BIT_SYSCTRL_BST_MODE_BOOST);
	}

	return 0;
}
#endif

static int aw8691_haptic_play_que_seq(struct aw8691 *aw8691, unsigned char flag)
{
	aw8691_haptic_stop(aw8691);
	if (flag) {
		aw8691_haptic_play_mode(aw8691, AW8691_HAPTIC_RAM_MODE);
		aw8691_haptic_set_repeat_seq(aw8691, 0);
		aw8691_haptic_start(aw8691);
	}
	return 0;
}

static int aw8691_haptic_play_repeat_seq(struct aw8691 *aw8691,
	unsigned char flag)
{
	aw8691_haptic_stop(aw8691);
	if (flag) {
		aw8691_haptic_play_mode(aw8691, AW8691_HAPTIC_RAM_MODE);
		aw8691_haptic_set_repeat_seq(aw8691, 1);
		aw8691_haptic_start(aw8691);
	}
	return 0;
}

static void aw8691_haptic_set_rtp_aei(struct aw8691 *aw8691, bool flag)
{
	if (flag) {
		aw8691_i2c_write_bits(aw8691, AW8691_REG_SYSINTM,
				      AW8691_BIT_SYSINTM_FF_AE_MASK,
				      AW8691_BIT_SYSINTM_FF_AE_EN);
	} else {
		aw8691_i2c_write_bits(aw8691, AW8691_REG_SYSINTM,
				      AW8691_BIT_SYSINTM_FF_AE_MASK,
				      AW8691_BIT_SYSINTM_FF_AE_OFF);
	}
}

#ifdef AW8691_HAPTIC_VBAT_MONITOR
static int aw8691_get_sys_battery_info(char *dev)
{
#ifdef AWINIC_GET_BATTERY_READ_NODE
	int fd;
	int eCheck;
	int nReadSize;
	char buf[64],*pvalue;
	mm_segment_t oldfs;

	oldfs = get_fs();
	set_fs(KERNEL_DS);
	fd = sys_open(dev, O_RDONLY, 0);
	if (fd < 0) {
		pr_err("%s: open fail dev:%s fd:%d\n", __func__, dev, fd);
		set_fs(oldfs);
		return fd;
	}

	nReadSize = sys_read(fd, buf, sizeof(buf) - 1);
	pr_debug("%s: nReadSize:%d\n", __func__, nReadSize);

	eCheck = simple_strtoul(buf,&pvalue,10);
	pr_debug("%s: eCheck = %d\n",eCheck);

	set_fs(oldfs);
	sys_close(fd);

	if (eCheck > 0)
		return eCheck;
	else
		return 0;
#else
	struct power_supply *batt_psy;
	union power_supply_propval prop = {0, };
	int rc = -1;

	batt_psy = power_supply_get_by_name("battery");
	rc = power_supply_get_property(batt_psy, POWER_SUPPLY_PROP_VOLTAGE_NOW,
				       &prop);
	power_supply_put(batt_psy);
	if (rc < 0) {
		pr_err("Error in getting charging status, rc=%d\n", rc);
		return 0;
	}
	return prop.intval;
#endif
}
#endif

static unsigned char aw8691_haptic_rtp_get_fifo_afi(struct aw8691 *aw8691)
{
	unsigned char ret = 0;
	unsigned char reg_val = 0;

	aw8691_i2c_read(aw8691, AW8691_REG_SYSINT, &reg_val);
	reg_val &= AW8691_BIT_SYSINT_FF_AFI;
	ret = reg_val >> 3;

	return ret;
}

static int aw8691_haptic_rtp_init(struct aw8691 *aw8691)
{
	unsigned int buf_len = 0;

	pr_debug("%s enter\n", __func__);

	aw8691->rtp_cnt = 0;

	while ((!aw8691_haptic_rtp_get_fifo_afi(aw8691)) &&
	      (aw8691->play_mode == AW8691_HAPTIC_RTP_MODE)) {
		pr_debug("%s rtp cnt = %d\n", __func__, aw8691->rtp_cnt);
		if (aw8691_rtp->len-aw8691->rtp_cnt < (aw8691->ram.base_addr >> 3)) {
			buf_len = aw8691_rtp->len-aw8691->rtp_cnt;
		} else {
			buf_len = aw8691->ram.base_addr >> 3;
		}
		aw8691_i2c_writes(aw8691, AW8691_REG_RTP_DATA,
				  &aw8691_rtp->data[aw8691->rtp_cnt], buf_len);
		aw8691->rtp_cnt += buf_len;
		if (aw8691->rtp_cnt == aw8691_rtp->len) {
			pr_debug("%s: rtp update complete\n", __func__);
			aw8691->rtp_cnt = 0;
#ifdef AW8691_HAPTIC_VBAT_MONITOR
			aw8691_haptic_set_bst_mode(aw8691, AW8691_HAPTIC_BOOST_MODE);
#endif
			return 0;
		}
	}

	if (aw8691->play_mode == AW8691_HAPTIC_RTP_MODE) {
		aw8691_haptic_set_rtp_aei(aw8691, true);
	}

	pr_debug("%s exit\n", __func__);

	return 0;
}

static void aw8691_rtp_work_routine(struct work_struct *work)
{
	struct aw8691 *aw8691 = container_of(work, struct aw8691, rtp_work);

	pr_debug("%s enter\n", __func__);

	aw8691_haptic_play_mode(aw8691, AW8691_HAPTIC_RTP_MODE);
	aw8691_i2c_write_bits(aw8691, AW8691_REG_SYSCTRL,
			      AW8691_BIT_SYSCTRL_WORK_MODE_MASK,
			      AW8691_BIT_SYSCTRL_ACTIVE);
	msleep(2);
	aw8691_haptic_start(aw8691);
	aw8691_haptic_rtp_init(aw8691);
}


/*****************************************************
 *
 * haptic fops
 *
 *****************************************************/
static int aw8691_file_open(struct inode *inode, struct file *file)
{
	if (!try_module_get(THIS_MODULE))
		return -ENODEV;

	file->private_data = (void*)g_aw8691;

	return 0;
}

static int aw8691_file_release(struct inode *inode, struct file *file)
{
	file->private_data = (void*)NULL;

	module_put(THIS_MODULE);

	return 0;
}

static long aw8691_file_unlocked_ioctl(struct file *file, unsigned int cmd,
	unsigned long arg)
{
	struct aw8691 *aw8691 = (struct aw8691 *)file->private_data;

	int ret = 0;
	unsigned int temp = 0;

	dev_info(aw8691->dev, "%s: cmd=0x%x, arg=0x%lx\n",
		 __func__, cmd, arg);

	mutex_lock(&aw8691->lock);

	if (_IOC_TYPE(cmd) != AW8691_HAPTIC_IOCTL_MAGIC) {
		dev_err(aw8691->dev, "%s: cmd magic err\n",
			__func__);
		return -EINVAL;
	}

	switch (cmd) {
	case AW8691_HAPTIC_SET_QUE_SEQ:
		ret = aw8691_set_que_seq(aw8691, arg);
		break;
	case AW8691_HAPTIC_SET_SEQ_LOOP:
		ret = aw8691_set_seq_loop(aw8691, arg);
		break;
	case AW8691_HAPTIC_PLAY_QUE_SEQ:
		if (copy_from_user(&temp, (void __user *)arg, sizeof(int)))
			return -EFAULT;
		aw8691_haptic_play_que_seq(aw8691, (unsigned char)(temp));
		break;
	case AW8691_HAPTIC_SET_BST_VOL:
		if (copy_from_user(&temp, (void __user *)arg, sizeof(int)))
			return -EFAULT;
		aw8691_haptic_set_bst_vol(aw8691, (unsigned char)(temp << 3));
		break;
	case AW8691_HAPTIC_SET_BST_PEAK_CUR:
		if (copy_from_user(&temp, (void __user *)arg, sizeof(int)))
			return -EFAULT;
		aw8691_haptic_set_bst_peak_cur(aw8691, (unsigned char)(temp));
		break;
	case AW8691_HAPTIC_SET_GAIN:
		if (copy_from_user(&temp, (void __user *)arg, sizeof(int)))
			return -EFAULT;
		aw8691_haptic_set_gain(aw8691, (unsigned char)(temp));
		break;
	case AW8691_HAPTIC_PLAY_REPEAT_SEQ:
		if (copy_from_user(&temp, (void __user *)arg, sizeof(int)))
			return -EFAULT;
		aw8691_haptic_play_repeat_seq(aw8691, (unsigned char)(temp));
		break;
	default:
		dev_err(aw8691->dev, "%s, unknown cmd\n", __func__);
		break;
	}

	mutex_unlock(&aw8691->lock);

	return ret;
}

static ssize_t aw8691_file_read(struct file* filp, char* buff, size_t len,
	loff_t* offset)
{
	struct aw8691 *aw8691 = (struct aw8691 *)filp->private_data;
	int ret = 0;
	int i = 0;
	unsigned char reg_val = 0;
	unsigned char *pbuff = NULL;

	mutex_lock(&aw8691->lock);

	dev_info(aw8691->dev, "%s: len=%zu\n", __func__, len);

	switch (aw8691->fileops.cmd) {
	case AW8691_HAPTIC_CMD_READ_REG:
		pbuff = (unsigned char *)kzalloc(len, GFP_KERNEL);
		if (pbuff != NULL) {
			for (i = 0; i < len; i++) {
				aw8691_i2c_read(aw8691,
						aw8691->fileops.reg + i,
						&reg_val);
				pbuff[i] = reg_val;
			}
			ret = copy_to_user(buff, pbuff, len);
			if (ret) {
				dev_err(aw8691->dev, "%s: copy to user fail\n",
					__func__);
			}
			kfree(pbuff);
		} else {
			dev_err(aw8691->dev, "%s: alloc memory fail\n",
				__func__);
		}
		break;
	case AW8691_HAPTIC_CMD_READ_FIRMWARE:
		pbuff = (unsigned char *)kzalloc(len, GFP_KERNEL);
		if (pbuff != NULL) {
			aw8691_haptic_stop(aw8691);
			/* RAMINIT Enable */
			aw8691_i2c_write_bits(aw8691, AW8691_REG_SYSCTRL,
					      AW8691_BIT_SYSCTRL_RAMINIT_MASK,
					      AW8691_BIT_SYSCTRL_RAMINIT_EN);

			aw8691_i2c_write(aw8691, AW8691_REG_RAMADDRH, aw8691->fileops.ram_addrh);
			aw8691_i2c_write(aw8691, AW8691_REG_RAMADDRL, aw8691->fileops.ram_addrl);
			for (i = 0; i < len; i++) {
				aw8691_i2c_read(aw8691, AW8691_REG_RAMDATA, &reg_val);
				pbuff[i] = reg_val;
			}

			/* RAMINIT Disable */
			aw8691_i2c_write_bits(aw8691, AW8691_REG_SYSCTRL,
					      AW8691_BIT_SYSCTRL_RAMINIT_MASK,
					      AW8691_BIT_SYSCTRL_RAMINIT_OFF);

			ret = copy_to_user(buff, pbuff, len);
			if (ret) {
				dev_err(aw8691->dev, "%s: copy to user fail\n",
					__func__);
			}
			kfree(pbuff);
		} else {
			dev_err(aw8691->dev, "%s: alloc memory fail\n",
				__func__);
		}
		break;
	default:
		dev_err(aw8691->dev, "%s, unknown cmd %d \n", __func__,
			aw8691->fileops.cmd);
		break;
	}

	mutex_unlock(&aw8691->lock);

	for (i = 0; i < len; i++) {
		dev_info(aw8691->dev, "%s: buff[%d]=0x%02x\n",
			 __func__, i, buff[i]);
	}

	return len;
}

static ssize_t aw8691_file_write(struct file* filp, const char* buff,
	size_t len, loff_t* off)
{
	struct aw8691 *aw8691 = (struct aw8691 *)filp->private_data;
	int i = 0;
	int ret = 0;
	unsigned char *pbuff = NULL;

	for (i = 0; i < len; i++) {
		dev_info(aw8691->dev, "%s: buff[%d]=0x%02x\n",
			 __func__, i, buff[i]);
	}

	mutex_lock(&aw8691->lock);

	aw8691->fileops.cmd = buff[0];

	switch (aw8691->fileops.cmd) {
	case AW8691_HAPTIC_CMD_READ_REG:
		if (len == 2) {
			aw8691->fileops.reg = buff[1];
		} else {
			dev_err(aw8691->dev, "%s: read cmd len %zu err\n",
				__func__, len);
		}
	    break;
	case AW8691_HAPTIC_CMD_WRITE_REG:
		if (len > 2) {
			pbuff = (unsigned char *)kzalloc(len, GFP_KERNEL);
			if (pbuff != NULL) {
				ret = copy_from_user(pbuff, buff, len);
				if (ret) {
					dev_err(aw8691->dev,
						"%s: copy from user fail\n",
						__func__);
				} else {
					for (i = 0; i < len - 2; i++) {
						dev_info(aw8691->dev,
							 "%s: write reg0x%02x=0x%02x\n",
							 __func__, pbuff[1] + i, pbuff[i + 2]);
						aw8691_i2c_write(aw8691, pbuff[1] + i, pbuff[2 + i]);
					}
				}
				kfree(pbuff);
			} else {
				dev_err(aw8691->dev, "%s: alloc memory fail\n", __func__);
			}
		} else {
			dev_err(aw8691->dev, "%s: write cmd len %zu err\n", __func__, len);
		}
		break;
	case AW8691_HAPTIC_CMD_UPDATE_FIRMWARE:
		pbuff = (unsigned char *)kzalloc(len-1, GFP_KERNEL);
		if (pbuff != NULL) {
			ret = copy_from_user(pbuff, &buff[1], len-1);
			if (ret) {
				dev_err(aw8691->dev,
					"%s: copy from user fail\n", __func__);
			} else {
				aw8691_haptic_stop(aw8691);
				/* RAMINIT Enable */
				aw8691_i2c_write_bits(aw8691, AW8691_REG_SYSCTRL,
						      AW8691_BIT_SYSCTRL_RAMINIT_MASK,
						      AW8691_BIT_SYSCTRL_RAMINIT_EN);

				aw8691_i2c_write(aw8691, AW8691_REG_RAMADDRH, aw8691->fileops.ram_addrh);
				aw8691_i2c_write(aw8691, AW8691_REG_RAMADDRL, aw8691->fileops.ram_addrl);
				for (i = 0; i < len - 1; i++) {
					aw8691_i2c_write(aw8691, AW8691_REG_RAMDATA, pbuff[i]);
				}

				/* RAMINIT Disable */
				aw8691_i2c_write_bits(aw8691, AW8691_REG_SYSCTRL,
						      AW8691_BIT_SYSCTRL_RAMINIT_MASK,
						      AW8691_BIT_SYSCTRL_RAMINIT_OFF);
			}
			kfree(pbuff);
		} else {
			dev_err(aw8691->dev, "%s: alloc memory fail\n",
				__func__);
		}
	    break;
	case AW8691_HAPTIC_CMD_READ_FIRMWARE:
		if (len == 3) {
			aw8691->fileops.ram_addrh = buff[1];
			aw8691->fileops.ram_addrl = buff[2];
		} else {
			dev_err(aw8691->dev, "%s: read firmware len %zu err\n",
				__func__, len);
		}
		break;
	default:
		dev_err(aw8691->dev, "%s, unknown cmd %d \n", __func__,
			aw8691->fileops.cmd);
		break;
	}

	mutex_unlock(&aw8691->lock);

	return len;
}

static struct file_operations fops = {
	.owner = THIS_MODULE,
	.read = aw8691_file_read,
	.write = aw8691_file_write,
	.unlocked_ioctl = aw8691_file_unlocked_ioctl,
	.open = aw8691_file_open,
	.release = aw8691_file_release,
};

static struct miscdevice aw8691_haptic_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = AW8691_HAPTIC_NAME,
	.fops = &fops,
};

static int aw8691_haptic_init(struct aw8691 *aw8691)
{
	int ret = 0;

	pr_info("%s enter\n", __func__);

	ret = misc_register(&aw8691_haptic_misc);
	if (ret) {
		dev_err(aw8691->dev,  "%s: misc fail: %d\n", __func__, ret);
		return ret;
	}

	aw8691_haptic_stop(aw8691);
	aw8691_i2c_write(aw8691, AW8691_REG_ANACTRL, 0x01);
	aw8691_i2c_write(aw8691, AW8691_REG_BSTCFG, 0x57);
	aw8691_i2c_write(aw8691, AW8691_REG_TRG3_WAV_P, 0x00);
	aw8691_i2c_write(aw8691, AW8691_REG_TRG1_WAV_N, 0x02);
	aw8691_i2c_write(aw8691, AW8691_REG_TRG2_WAV_N, 0x02);
	aw8691_i2c_write(aw8691, AW8691_REG_TRG3_WAV_N, 0x00);

	return ret;
}



/*****************************************************
 *
 * vibrator
 *
 *****************************************************/
static void aw8691_wave_combain(struct aw8691 *aw8691,
	struct vibrator_combin combin)
{
	int i;

	for (i = 0; i < AW8691_SEQUENCER_SIZE; i++) {
		aw8691_i2c_write(aw8691, AW8691_REG_QUE_SEQ1 + i,
				 combin.index[i]);
	}
	for (i = 0; i < AW8691_SEQUENCER_LOOP_SIZE; i++) {
		aw8691_i2c_write(aw8691, AW8691_REG_SEQ_LOOP1 + i,
				 combin.loop[i]);
	}
	aw8691_haptic_start(aw8691);
}

static int aw8691_search_combin(unsigned int mode)
{
	int mid;
	int left = 0;
	int right = sizeof(aw8691_vibrator_combin) / sizeof(struct vibrator_combin);

	while (left <= right) {
		mid = left + (right - left) / 2;
		if (mode > aw8691_vibrator_combin[mid].id_num)
			left = mid + 1;
		else if (mode < aw8691_vibrator_combin[mid].id_num)
			right = mid - 1;
		else
			return mid;
	}

	pr_err("%s: error!\n", __func__);
	return -1;
}

static ssize_t aw8691_duration_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct aw8691 *aw8691 = dev_get_drvdata(dev);
	ktime_t time_rem;
	s64 time_ms = 0;

	if (hrtimer_active(&aw8691->timer)) {
		time_rem = hrtimer_get_remaining(&aw8691->timer);
		time_ms = ktime_to_ms(time_rem);
	}

	return snprintf(buf, PAGE_SIZE, "%lld\n", time_ms);
}

static ssize_t aw8691_duration_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct aw8691 *aw8691 = dev_get_drvdata(dev);
	unsigned int val = 0;
	int rc = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;

	pr_debug("%s: value=%d\n", __func__, val);

	/* setting 0 on duration is NOP for now */
	if (val <= 0)
		return count;

	aw8691->duration = val;

	return count;
}

static ssize_t aw8691_activate_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct aw8691 *aw8691 = dev_get_drvdata(dev);

	/* For now nothing to show */
	return snprintf(buf, PAGE_SIZE, "%d\n", aw8691->state);
}

static ssize_t aw8691_activate_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct aw8691 *aw8691 = dev_get_drvdata(dev);
	unsigned int val = 0;
	int rc = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;

	if (val != 0 && val != 1)
		return count;

	pr_debug("%s: value=%d\n", __func__, val);

	mutex_lock(&aw8691->lock);
	hrtimer_cancel(&aw8691->timer);

	aw8691->state = val;

	if (val != 0) {
		hrtimer_start(&aw8691->timer,
			      ktime_set(aw8691->duration / 1000,
					(aw8691->duration % 1000) * 1000000),
			      HRTIMER_MODE_REL);
	}

	mutex_unlock(&aw8691->lock);
	schedule_work(&aw8691->vibrator_work);

	return count;
}

static ssize_t aw8691_index_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct aw8691 *aw8691 = dev_get_drvdata(dev);
	unsigned char reg_val = 0;
	aw8691_i2c_read(aw8691, AW8691_REG_QUE_SEQ1, &reg_val);
	aw8691->index = reg_val;

	return snprintf(buf, PAGE_SIZE, "%d\n", aw8691->index);
}

static ssize_t aw8691_index_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct aw8691 *aw8691 = dev_get_drvdata(dev);
	unsigned int val = 0;
	int rc = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;

	pr_debug("%s: value=%d\n", __func__, val);

	mutex_lock(&aw8691->lock);
	aw8691->index = val;
	aw8691_haptic_set_repeat_que_seq(aw8691, aw8691->index);
	mutex_unlock(&aw8691->lock);
	return count;
}

static ssize_t aw8691_vmax_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct aw8691 *aw8691 = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%d\n", aw8691->vmax);
}

static ssize_t aw8691_vmax_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct aw8691 *aw8691 = dev_get_drvdata(dev);
	unsigned int val = 0;
	int rc = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;

	pr_debug("%s: value=%d\n", __func__, val);

	mutex_lock(&aw8691->lock);
	aw8691->vmax = val;
	aw8691_haptic_set_bst_vol(aw8691, (unsigned char)(aw8691->vmax << 3));
	mutex_unlock(&aw8691->lock);
	return count;
}

static ssize_t aw8691_gain_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct aw8691 *aw8691 = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%d\n", aw8691->gain);
}

static ssize_t aw8691_gain_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct aw8691 *aw8691 = dev_get_drvdata(dev);
	unsigned int val = 0;
	int rc = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;

	pr_debug("%s: value=%d\n", __func__, val);

	mutex_lock(&aw8691->lock);
	aw8691->gain = val;
	aw8691_haptic_set_gain(aw8691, (unsigned char)aw8691->gain);
	mutex_unlock(&aw8691->lock);
	return count;
}

static ssize_t aw8691_seq_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct aw8691 *aw8691 = dev_get_drvdata(dev);
	size_t count = 0;
	unsigned char i = 0;
	unsigned char reg_val = 0;

	aw8691->seq = 0;
	for(i = 0; i < AW8691_SEQUENCER_SIZE; i++) {
		aw8691_i2c_read(aw8691, AW8691_REG_QUE_SEQ1 + i, &reg_val);
		count += snprintf(buf+count, PAGE_SIZE - count,
				  "seq%d: 0x%02x\n", i + 1, reg_val);
		aw8691->seq |= (reg_val << ((AW8691_SEQUENCER_SIZE - i - 1) * 8));
	}
	return count;
}

static ssize_t aw8691_seq_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct aw8691 *aw8691 = dev_get_drvdata(dev);
	unsigned int val = 0;
	int rc = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;

	pr_debug("%s: value=%x\n", __func__, val);

	mutex_lock(&aw8691->lock);
	aw8691->seq = val;
	aw8691_haptic_set_que_seq(aw8691, aw8691->seq);
	mutex_unlock(&aw8691->lock);
	return count;
}

static ssize_t aw8691_loop_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct aw8691 *aw8691 = dev_get_drvdata(dev);
	size_t count = 0;
	unsigned char i = 0;
	unsigned char reg_val = 0;
	unsigned char tmp[2];
	unsigned char loop_tmp = 0;

	aw8691->loop = 0;
	for (i = 0; i < AW8691_SEQUENCER_LOOP_SIZE; i++) {
		aw8691_i2c_read(aw8691, AW8691_REG_SEQ_LOOP1+i, &reg_val);
		tmp[0] = reg_val >> 4;
		tmp[1] = reg_val & 0x0F;
		loop_tmp = (tmp[0] << 8) | tmp[1];
		count += snprintf(buf + count, PAGE_SIZE - count,
				  "seq%d loop: 0x%02x\n", i * 2 + 1, tmp[0]);
		count += snprintf(buf + count, PAGE_SIZE - count,
				  "seq%d loop: 0x%02x\n", i * 2 + 2, tmp[1]);
		aw8691->loop |= (loop_tmp << ((AW8691_SEQUENCER_LOOP_SIZE - i - 1) * 8));
	}
	return count;
}

static ssize_t aw8691_loop_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct aw8691 *aw8691 = dev_get_drvdata(dev);
	unsigned int val = 0;
	int rc = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;

	pr_debug("%s: value=%x\n", __func__, val);

	mutex_lock(&aw8691->lock);
	aw8691->loop = val;
	aw8691_haptic_set_que_loop(aw8691, aw8691->loop);
	mutex_unlock(&aw8691->lock);
	return count;
}

static ssize_t aw8691_reg_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct aw8691 *aw8691 = dev_get_drvdata(dev);
	ssize_t len = 0;
	unsigned char i = 0;
	unsigned char reg_val = 0;
	for (i = 0; i < AW8691_REG_MAX; i++) {
		if (!(aw8691_reg_access[i] & REG_RD_ACCESS))
			continue;
		aw8691_i2c_read(aw8691, i, &reg_val);
		len += snprintf(buf + len, PAGE_SIZE - len,
				"reg:0x%02x=0x%02x \n", i, reg_val);
	}
	return len;
}

static ssize_t aw8691_reg_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct aw8691 *aw8691 = dev_get_drvdata(dev);
	unsigned int databuf[2] = {0, 0};

	if (2 == sscanf(buf, "%x %x", &databuf[0], &databuf[1])) {
		aw8691_i2c_write(aw8691, (unsigned char)databuf[0],
				 (unsigned char)databuf[1]);
	}

	return count;
}

static ssize_t aw8691_rtp_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	ssize_t len = 0;
	len += snprintf(buf + len, PAGE_SIZE - len, "rtp mode\n");
	return len;
}

static ssize_t aw8691_rtp_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct aw8691 *aw8691 = dev_get_drvdata(dev);
	unsigned int val = 0;
	int rc = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;

	pr_debug("%s: value=%d\n", __func__, val);

	enable_irq(aw8691->irq_gpio);
	aw8691_haptic_set_rtp_aei(aw8691, false);
	aw8691_interrupt_clear(aw8691);

	return count;
}

static ssize_t aw8691_ram_update_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	ssize_t len = 0;
	len += snprintf(buf + len, PAGE_SIZE - len, "ram/rtp mode\n");
	return len;
}

static ssize_t aw8691_ram_update_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct aw8691 *aw8691 = dev_get_drvdata(dev);
	unsigned int val = 0;
	int rc = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;

	if (val) {
		aw8691_ram_update(aw8691);
	}
	return count;
}

static ssize_t aw8691_enable_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct aw8691 *aw8691 = dev_get_drvdata(dev);
	int rc = 0;

	rc = kstrtouint(buf, 0, &play);
	if (rc < 0)
		return rc;

	pr_debug("%s: play is %d\n", __func__, play);
	if (play < 20001) {
		if (play != 0)
			schedule_work(&aw8691->proline_work);
		else {
			aw8691_haptic_play_mode(aw8691, AW8691_HAPTIC_RAM_MODE);
			aw8691_haptic_set_repeat_seq(aw8691, 0);
			aw8691_haptic_stop(aw8691);
		}
	}

	return count;
}

static ssize_t aw8691_play_effect_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct aw8691 *aw8691 = dev_get_drvdata(dev);
	unsigned int mode = 0;
	int rc = 0;
	int i;

	rc = kstrtouint(buf, 0, &mode);
	if (rc < 0)
		return rc;

	aw8691_haptic_play_mode(aw8691, AW8691_HAPTIC_RAM_MODE);
	aw8691_haptic_set_repeat_seq(aw8691, 0);

	if (mode == 0) {
		aw8691_haptic_stop(aw8691);
		return count;
	}

	i = aw8691_search_combin(mode);
	if (i != -1)
		aw8691_wave_combain(aw8691, aw8691_vibrator_combin[i]);
	else
		pr_err("%s: mode error %d\n", __func__, mode);

	return count;
}

static DEVICE_ATTR(duration, S_IWUSR | S_IRUGO, aw8691_duration_show, aw8691_duration_store);
static DEVICE_ATTR(activate, S_IWUSR | S_IRUGO, aw8691_activate_show, aw8691_activate_store);
static DEVICE_ATTR(index, S_IWUSR | S_IRUGO, aw8691_index_show, aw8691_index_store);
static DEVICE_ATTR(vmax, S_IWUSR | S_IRUGO, aw8691_vmax_show, aw8691_vmax_store);
static DEVICE_ATTR(gain, S_IWUSR | S_IRUGO, aw8691_gain_show, aw8691_gain_store);
static DEVICE_ATTR(seq, S_IWUSR | S_IRUGO, aw8691_seq_show, aw8691_seq_store);
static DEVICE_ATTR(loop, S_IWUSR | S_IRUGO, aw8691_loop_show, aw8691_loop_store);
static DEVICE_ATTR(register, S_IWUSR | S_IRUGO, aw8691_reg_show, aw8691_reg_store);
static DEVICE_ATTR(rtp, S_IWUSR | S_IRUGO, aw8691_rtp_show, aw8691_rtp_store);
static DEVICE_ATTR(ram_update, S_IWUSR | S_IRUGO, aw8691_ram_update_show, aw8691_ram_update_store);
static DEVICE_ATTR(enable, S_IWUSR | S_IRUGO, NULL, aw8691_enable_store);
static DEVICE_ATTR(play_effect, S_IWUSR | S_IRUGO, NULL, aw8691_play_effect_store);

static struct attribute *aw8691_vibrator_attributes[] = {
	&dev_attr_duration.attr,
	&dev_attr_activate.attr,
	&dev_attr_index.attr,
	&dev_attr_vmax.attr,
	&dev_attr_gain.attr,
	&dev_attr_seq.attr,
	&dev_attr_loop.attr,
	&dev_attr_register.attr,
	&dev_attr_rtp.attr,
	&dev_attr_ram_update.attr,
	&dev_attr_enable.attr,
	&dev_attr_play_effect.attr,
	NULL
};

static struct attribute_group aw8691_vibrator_attribute_group = {
	.attrs = aw8691_vibrator_attributes
};

static int aw8691_register_vibrator(struct aw8691 *aw8691) {
	struct class *cls;
	struct device *dev;
	int ret = 0;

	cls = class_create(THIS_MODULE, "timed_output");
	if (IS_ERR(cls)) {
		dev_err(aw8691->dev, "%s: fail to create timed output class\n",
			__func__);
		return PTR_ERR(cls);
	}

	dev = device_create(cls, aw8691->dev, 0, NULL, "vibrator");
	if (IS_ERR(dev)) {
		dev_err(aw8691->dev, "%s: fail to create vibrator device\n",
			__func__);
		return PTR_ERR(dev);
	}

	dev_set_drvdata(dev, aw8691);

	ret = sysfs_create_group(&dev->kobj, &aw8691_vibrator_attribute_group);
	if (ret < 0) {
		dev_err(aw8691->dev, "%s error creating sysfs attr files\n",
			__func__);
		sysfs_remove_group(&dev->kobj,
				   &aw8691_vibrator_attribute_group);
	}

	return ret;
}

static void aw8691_proline_work_routine(struct work_struct *work)
{
	struct aw8691 *aw8691 = container_of(work, struct aw8691, proline_work);

	aw8691_haptic_play_mode(aw8691, false);
	aw8691_haptic_set_repeat_seq(aw8691, 1);

	aw8691_wave_combain(aw8691, aw8691_vibrator_combin[0]);
	msleep(play);
	aw8691_haptic_stop(aw8691);

	aw8691_haptic_set_repeat_seq(aw8691, 0);
}

static enum hrtimer_restart aw8691_vibrator_timer_func(struct hrtimer *timer)
{
	struct aw8691 *aw8691 = container_of(timer, struct aw8691, timer);

	pr_debug("%s enter\n", __func__);
	aw8691->state = 0;
	schedule_work(&aw8691->vibrator_work);

	return HRTIMER_NORESTART;
}

static void aw8691_vibrator_work_routine(struct work_struct *work)
{
	struct aw8691 *aw8691 = container_of(work, struct aw8691, vibrator_work);

	pr_debug("%s enter\n", __func__);

	mutex_lock(&aw8691->lock);

	if (aw8691->state) {
		aw8691_haptic_play_mode(aw8691, AW8691_HAPTIC_RAM_MODE);
		aw8691_haptic_set_repeat_seq(aw8691, 1);
		aw8691_haptic_start(aw8691);
	} else {
		aw8691_haptic_stop(aw8691);
	}
	mutex_unlock(&aw8691->lock);
}

static int aw8691_vibrator_init(struct aw8691 *aw8691)
{
	int ret = 0;
	unsigned char i = 0;
	unsigned char reg_val = 0;

	pr_info("%s enter\n", __func__);

	ret = aw8691_i2c_read(aw8691, AW8691_REG_QUE_SEQ1, &reg_val);
	aw8691->index = reg_val & 0x7F;
	ret = aw8691_i2c_read(aw8691, AW8691_REG_DATDBG, &reg_val);
	aw8691->gain = reg_val & 0x7F;
	ret = aw8691_i2c_read(aw8691, AW8691_REG_BSTCFG, &reg_val);
	aw8691->vmax = (reg_val >> 3);
	for (i = 0; i < AW8691_SEQUENCER_SIZE; i++) {
		ret = aw8691_i2c_read(aw8691, AW8691_REG_QUE_SEQ1 + i, &reg_val);
		aw8691->seq |= (reg_val << ((AW8691_SEQUENCER_SIZE - i - 1) * 8));
	}

	ret = aw8691_register_vibrator(aw8691);
	if (ret < 0) {
		dev_err(aw8691->dev, "%s: fail to register vibrator\n",
			__func__);
		return ret;
	}

	hrtimer_init(&aw8691->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	aw8691->timer.function = aw8691_vibrator_timer_func;
	INIT_WORK(&aw8691->vibrator_work, aw8691_vibrator_work_routine);

	INIT_WORK(&aw8691->proline_work, aw8691_proline_work_routine);

	INIT_WORK(&aw8691->rtp_work, aw8691_rtp_work_routine);

	mutex_init(&aw8691->lock);

	return 0;
}

/*****************************************************
 *
 * regmap
 *
 *****************************************************/
static bool aw8691_writeable_register(struct device *dev, unsigned int reg)
{
	/* enable read access for all registers */
	return true;
}

static bool aw8691_readable_register(struct device *dev, unsigned int reg)
{
	/* enable read access for all registers */
	return true;
}

static bool aw8691_volatile_register(struct device *dev, unsigned int reg)
{
	/* enable read access for all registers */
	return true;
}

static const struct regmap_config aw8691_regmap = {
	.reg_bits = 8,
	.val_bits = 8,

	.max_register = AW8691_REG_MAX,
	.writeable_reg = aw8691_writeable_register,
	.readable_reg = aw8691_readable_register,
	.volatile_reg = aw8691_volatile_register,
	.cache_type = REGCACHE_RBTREE,
};



/******************************************************
 *
 * irq
 *
 ******************************************************/
static void aw8691_interrupt_clear(struct aw8691 *aw8691)
{
	unsigned char reg_val = 0;
	pr_debug("%s enter\n", __func__);
	aw8691_i2c_read(aw8691, AW8691_REG_SYSINT, &reg_val);
	pr_debug("%s: reg SYSINT=0x%x\n", __func__, reg_val);
}

static void aw8691_interrupt_setup(struct aw8691 *aw8691)
{
	unsigned char reg_val = 0;

	pr_info("%s enter\n", __func__);

	aw8691_i2c_read(aw8691, AW8691_REG_SYSINT, &reg_val);
	pr_info("%s: reg SYSINT=0x%x\n", __func__, reg_val);
}

static irqreturn_t aw8691_irq(int irq, void *data)
{
	struct aw8691 *aw8691 = data;
	unsigned char reg_val = 0;
	unsigned int buf_len = 0;

	pr_debug("%s enter\n", __func__);

	aw8691_i2c_read(aw8691, AW8691_REG_SYSINT, &reg_val);

	if (reg_val & AW8691_BIT_SYSINT_FF_AEI) {
		if (aw8691->rtp_init) {
			while ((!aw8691_haptic_rtp_get_fifo_afi(aw8691)) &&
			       (aw8691->play_mode == AW8691_HAPTIC_RTP_MODE)) {
				pr_debug("%s: aw8691 rtp mode fifo update, cnt=%d\n",
					 __func__, aw8691->rtp_cnt);
				if (aw8691_rtp->len-aw8691->rtp_cnt < (aw8691->ram.base_addr >> 3)) {
					buf_len = aw8691_rtp->len - aw8691->rtp_cnt;
				} else {
					buf_len = aw8691->ram.base_addr >> 3;
				}
				aw8691_i2c_writes(aw8691, AW8691_REG_RTP_DATA,
						  &aw8691_rtp->data[aw8691->rtp_cnt],
						  buf_len);
				aw8691->rtp_cnt += buf_len;
				if (aw8691->rtp_cnt == aw8691_rtp->len) {
					pr_debug("%s: rtp update complete\n", __func__);
					aw8691_haptic_set_rtp_aei(aw8691, false);
					aw8691->rtp_cnt = 0;
					aw8691->rtp_init = 0;
#ifdef AW8691_HAPTIC_VBAT_MONITOR
					aw8691_haptic_set_bst_mode(aw8691, AW8691_HAPTIC_BOOST_MODE);
#endif
					break;
				}
			}
		} else {
			pr_err("%s: aw8691 rtp init = %d, init error\n",
			       __func__, aw8691->rtp_init);
		}
	}

	if (reg_val & AW8691_BIT_SYSINT_FF_AFI) {
		pr_debug("%s: aw8691 rtp mode fifo full empty\n", __func__);
	}

	if (aw8691->play_mode != AW8691_HAPTIC_RTP_MODE) {
		aw8691_haptic_set_rtp_aei(aw8691, false);
	}

	aw8691_i2c_read(aw8691, AW8691_REG_SYSINT, &reg_val);
	aw8691_i2c_read(aw8691, AW8691_REG_SYSST, &reg_val);

	pr_debug("%s exit\n", __func__);

	return IRQ_HANDLED;
}

/*****************************************************
 *
 * device tree
 *
 *****************************************************/
static int aw8691_parse_dt(struct device *dev, struct aw8691 *aw8691,
	struct device_node *np)
{
	struct pinctrl *pinctrl;
	struct pinctrl_state *pinctrl_state;

	pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR_OR_NULL(pinctrl)) {
		dev_err(dev, "%s: failed to get pinctrl\n", __func__);
		return PTR_ERR(pinctrl);
	}

	pinctrl_state = pinctrl_lookup_state(pinctrl, "default");
	if (IS_ERR_OR_NULL(pinctrl_state)) {
		dev_err(dev, "%s: can't find pinctrl state haptic_default",
			__func__);
		return PTR_ERR(pinctrl_state);
	}

	pinctrl_select_state(pinctrl, pinctrl_state);

	aw8691->reset_gpio = of_get_named_gpio(np, "reset-gpio", 0);
	if (aw8691->reset_gpio < 0) {
		dev_err(dev, "%s: no reset gpio provided, will not HW reset device\n",
			__func__);
		return -1;
	} else {
		dev_info(dev, "%s: reset gpio provided ok\n", __func__);
	}

	aw8691->irq_gpio =  of_get_named_gpio(np, "irq-gpio", 0);
	if (aw8691->irq_gpio < 0) {
		dev_err(dev, "%s: no irq gpio provided.\n", __func__);
	} else {
		dev_info(dev, "%s: irq gpio provided ok.\n", __func__);
	}

	return 0;
}

static int aw8691_hw_reset(struct aw8691 *aw8691)
{
	pr_info("%s enter\n", __func__);

	if (aw8691 && gpio_is_valid(aw8691->reset_gpio)) {
		gpio_set_value_cansleep(aw8691->reset_gpio, 0);
		msleep(1);
		gpio_set_value_cansleep(aw8691->reset_gpio, 1);
		msleep(1);
	} else {
		dev_err(aw8691->dev, "%s: failed\n", __func__);
	}
	return 0;
}


/*****************************************************
 *
 * check chip id
 *
 *****************************************************/
static int aw8691_read_chipid(struct aw8691 *aw8691)
{
	int ret = -1;
	unsigned char cnt = 0;
	unsigned char reg = 0;

	while (cnt < AW_READ_CHIPID_RETRIES) {
		/* hardware reset */
		aw8691_hw_reset(aw8691);

		ret = aw8691_i2c_read(aw8691, AW8691_REG_ID, &reg);
		if (ret < 0) {
			dev_err(aw8691->dev,
				"%s: failed to read register AW8691_REG_ID: %d\n",
				__func__, ret);
		}
		switch (reg) {
		case 0x91:
			pr_info("%s aw8691 detected\n", __func__);
			aw8691->chipid = AW8691_ID;
			return 0;
		default:
			pr_info("%s unsupported device revision (0x%x)\n",
				__func__, reg );
			break;
		}
		cnt ++;

		msleep(AW_READ_CHIPID_RETRY_DELAY);
	}

	return -EINVAL;
}


/******************************************************
 *
 * sys group attribute: reg
 *
 ******************************************************/
static ssize_t aw8691_i2c_reg_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct aw8691 *aw8691 = dev_get_drvdata(dev);

	unsigned int databuf[2] = {0, 0};

	if (2 == sscanf(buf, "%x %x", &databuf[0], &databuf[1])) {
		aw8691_i2c_write(aw8691, (unsigned char)databuf[0],
				 (unsigned char)databuf[1]);
	}

	return count;
}

static ssize_t aw8691_i2c_reg_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct aw8691 *aw8691 = dev_get_drvdata(dev);
	ssize_t len = 0;
	unsigned char i = 0;
	unsigned char reg_val = 0;
	for(i = 0; i < AW8691_REG_MAX; i++) {
		if(!(aw8691_reg_access[i]&REG_RD_ACCESS))
			continue;
		aw8691_i2c_read(aw8691, i, &reg_val);
		len += snprintf(buf+len, PAGE_SIZE-len, "reg:0x%02x=0x%02x \n", i, reg_val);
	}
	return len;
}

static ssize_t aw8691_i2c_ram_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct aw8691 *aw8691 = dev_get_drvdata(dev);

	unsigned int databuf[1] = {0};

	if (1 == sscanf(buf, "%x", &databuf[0])) {
		if (1 == databuf[0]) {
			aw8691_ram_update(aw8691);
		}
	}

	return count;
}

static ssize_t aw8691_i2c_ram_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct aw8691 *aw8691 = dev_get_drvdata(dev);
	ssize_t len = 0;
	unsigned int i = 0;
	unsigned char reg_val = 0;

	aw8691_haptic_stop(aw8691);
	/* RAMINIT Enable */
	aw8691_i2c_write_bits(aw8691, AW8691_REG_SYSCTRL,
			      AW8691_BIT_SYSCTRL_RAMINIT_MASK,
			      AW8691_BIT_SYSCTRL_RAMINIT_EN);

	aw8691_i2c_write(aw8691, AW8691_REG_RAMADDRH,
			 (unsigned char)(aw8691->ram.base_addr >> 8));
	aw8691_i2c_write(aw8691, AW8691_REG_RAMADDRL,
			 (unsigned char)(aw8691->ram.base_addr & 0x00ff));
	len += snprintf(buf + len, PAGE_SIZE - len, "aw8691_haptic_ram:\n");
	for (i = 0; i < aw8691->ram.len; i++) {
		aw8691_i2c_read(aw8691, AW8691_REG_RAMDATA, &reg_val);
		len += snprintf(buf+len, PAGE_SIZE-len, "0x%02x,", reg_val);
	}
	len += snprintf(buf + len, PAGE_SIZE - len, "\n");
	aw8691_i2c_write_bits(aw8691, AW8691_REG_SYSCTRL,
			      AW8691_BIT_SYSCTRL_RAMINIT_MASK,
			      AW8691_BIT_SYSCTRL_RAMINIT_OFF);
	return len;
}

static DEVICE_ATTR(reg, S_IWUSR | S_IRUGO, aw8691_i2c_reg_show, aw8691_i2c_reg_store);
static DEVICE_ATTR(ram, S_IWUSR | S_IRUGO, aw8691_i2c_ram_show, aw8691_i2c_ram_store);

static struct attribute *aw8691_attributes[] = {
	&dev_attr_reg.attr,
	&dev_attr_ram.attr,
	NULL
};

static struct attribute_group aw8691_attribute_group = {
	.attrs = aw8691_attributes
};


/******************************************************
 *
 * i2c driver
 *
 ******************************************************/
static int aw8691_i2c_probe(struct i2c_client *i2c, const struct i2c_device_id *id)
{
	struct aw8691 *aw8691;
	struct device_node *np = i2c->dev.of_node;
	int irq_flags = 0;
	int ret = -1;

	pr_info("%s enter\n", __func__);

	if (!i2c_check_functionality(i2c->adapter, I2C_FUNC_I2C)) {
		dev_err(&i2c->dev, "check_functionality failed\n");
		return -EIO;
	}

	aw8691 = devm_kzalloc(&i2c->dev, sizeof(struct aw8691), GFP_KERNEL);
	if (aw8691 == NULL)
		return -ENOMEM;

	aw8691->dev = &i2c->dev;
	aw8691->i2c = i2c;

	aw8691->regmap = devm_regmap_init_i2c(i2c, &aw8691_regmap);
	if (IS_ERR(aw8691->regmap)) {
		ret = PTR_ERR(aw8691->regmap);
		dev_err(&i2c->dev, "%s: failed to allocate register map: %d\n",
			__func__, ret);
		goto err;
	}

	i2c_set_clientdata(i2c, aw8691);

	if (np) {
		ret = aw8691_parse_dt(&i2c->dev, aw8691, np);
		if (ret) {
			dev_err(&i2c->dev, "%s: failed to parse device tree node\n",
				__func__);
			goto err;
		}
	} else {
		aw8691->reset_gpio = -1;
		aw8691->irq_gpio = -1;
	}

	if (gpio_is_valid(aw8691->reset_gpio)) {
		ret = devm_gpio_request_one(&i2c->dev, aw8691->reset_gpio,
					    GPIOF_OUT_INIT_LOW, "aw8691_rst");
		if (ret){
			dev_err(&i2c->dev, "%s: rst request failed\n", __func__);
			goto err;
		}
	}

	if (gpio_is_valid(aw8691->irq_gpio)) {
		ret = devm_gpio_request_one(&i2c->dev, aw8691->irq_gpio,
					    GPIOF_DIR_IN, "aw8691_int");
		if (ret){
			dev_err(&i2c->dev, "%s: int request failed\n", __func__);
			goto err;
		}
	}

	/* aw8691 chip id */
	ret = aw8691_read_chipid(aw8691);
	if (ret < 0) {
		dev_err(&i2c->dev, "%s: aw8691_read_chipid failed ret=%d\n", __func__, ret);
		goto err_id;
	}

	/* aw8691 irq */
	if (gpio_is_valid(aw8691->irq_gpio) &&
	    !(aw8691->flags & AW8691_FLAG_SKIP_INTERRUPTS)) {
		/* register irq handler */
		aw8691_interrupt_setup(aw8691);
		irq_flags = IRQF_TRIGGER_FALLING | IRQF_ONESHOT;
		ret = devm_request_threaded_irq(&i2c->dev,
						gpio_to_irq(aw8691->irq_gpio),
						NULL, aw8691_irq, irq_flags,
						"aw8691", aw8691);
		if (ret != 0) {
			dev_err(&i2c->dev, "%s: failed to request IRQ %d: %d\n",
				__func__, gpio_to_irq(aw8691->irq_gpio), ret);
			goto err_irq;
		}
	} else {
		dev_info(&i2c->dev, "%s skipping IRQ registration\n", __func__);
		/* disable feature support if gpio was invalid */
		aw8691->flags |= AW8691_FLAG_SKIP_INTERRUPTS;
	}

	dev_set_drvdata(&i2c->dev, aw8691);

	ret = sysfs_create_group(&i2c->dev.kobj, &aw8691_attribute_group);
	if (ret < 0) {
		dev_info(&i2c->dev, "%s error creating sysfs attr files\n", __func__);
		goto err_sysfs;
	}

	g_aw8691 = aw8691;

	aw8691_vibrator_init(aw8691);

	aw8691_haptic_init(aw8691);

	aw8691_ram_init(aw8691);

	pr_info("%s probe completed successfully!\n", __func__);

	return 0;

err_sysfs:
	sysfs_remove_group(&i2c->dev.kobj, &aw8691_attribute_group);
err_irq:
err_id:
err:
	return ret;
}

static int aw8691_i2c_remove(struct i2c_client *i2c)
{
	struct aw8691 *aw8691 = i2c_get_clientdata(i2c);

	pr_info("%s enter\n", __func__);

	sysfs_remove_group(&i2c->dev.kobj, &aw8691_attribute_group);

	misc_deregister(&aw8691_haptic_misc);

	if (gpio_is_valid(aw8691->irq_gpio))
		devm_gpio_free(&i2c->dev, aw8691->irq_gpio);
	if (gpio_is_valid(aw8691->reset_gpio))
		devm_gpio_free(&i2c->dev, aw8691->reset_gpio);

	return 0;
}

static const struct i2c_device_id aw8691_i2c_id[] = {
	{ AW8691_I2C_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, aw8691_i2c_id);

static struct of_device_id aw8691_dt_match[] = {
	{ .compatible = "meizu,aw8691_haptic" },
	{ },
};

static struct i2c_driver aw8691_i2c_driver = {
	.driver = {
		.name = AW8691_I2C_NAME,
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(aw8691_dt_match),
	},
	.probe = aw8691_i2c_probe,
	.remove = aw8691_i2c_remove,
	.id_table = aw8691_i2c_id,
};


static int __init aw8691_i2c_init(void)
{
	int ret = 0;

	pr_info("aw8691 driver version %s\n", AW8691_VERSION);

	ret = i2c_add_driver(&aw8691_i2c_driver);
	if (ret) {
		pr_err("fail to add aw8691 device into i2c\n");
		return ret;
	}

	return 0;
}
module_init(aw8691_i2c_init);


static void __exit aw8691_i2c_exit(void)
{
	i2c_del_driver(&aw8691_i2c_driver);
}
module_exit(aw8691_i2c_exit);


MODULE_DESCRIPTION("AW8691 Haptic Driver");
MODULE_LICENSE("GPL v2");
