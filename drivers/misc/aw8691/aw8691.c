/*
 * aw8691.c   aw8691 haptic module
 *
 * Version: v1.4.0
 *
 * Copyright (c) 2017 AWINIC Technology CO., LTD
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

#define AW8691_VERSION "v1.4.0"


//#define AWINIC_I2C_REGMAP
//#define AWINIC_RAM_UPDATE_DELAY

#define AW_I2C_RETRIES 2
#define AW_I2C_RETRY_DELAY 2
#define AW_READ_CHIPID_RETRIES 5
#define AW_READ_CHIPID_RETRY_DELAY 2

#define AW8691_MAX_DSP_START_TRY_COUNT    10


//#define AW8691_REPEAT_RTP_PLAYING
#define AW8691_MAX_FIRMWARE_LOAD_CNT 20
#define AW8691_SEQ_NO_RTP_BASE 102
/******************************************************
 *
 * variable
 *
 ******************************************************/
#define AW8691_RTP_NAME_MAX        64
static char *aw8691_ram_name = "aw8691_haptic.bin";
static char aw8691_rtp_name[][AW8691_RTP_NAME_MAX] = {
    {"aw8691_rtp.bin"},
    {"aw8691_rtp_Argo_Navis.bin"},
    {"aw8691_rtp_Attentive.bin"},
    {"aw8691_rtp_Awake.bin"},
    {"aw8691_rtp_Bird_Loop.bin"},
    {"aw8691_rtp_Brilliant_Times.bin"},
    {"aw8691_rtp_Chimey_Phone.bin"},
    {"aw8691_rtp_Complex.bin"},
    {"aw8691_rtp_Crazy_Dream.bin"},
    {"aw8691_rtp_Curve_Ball_Blend.bin"},
    {"aw8691_rtp_Digital_Phone.bin"},
    {"aw8691_rtp_Electrovision.bin"},
    {"aw8691_rtp_Ether_Shake.bin"},
    {"aw8691_rtp_Fateful_Words.bin"},
    {"aw8691_rtp_Flutey_Phone.bin"},
    {"aw8691_rtp_Future_Funk.bin"},
    {"aw8691_rtp_Future_Hi_Tech.bin"},
    {"aw8691_rtp_Girtab.bin"},
    {"aw8691_rtp_Hello.bin"},
    {"aw8691_rtp_Hexagon.bin"},
    {"aw8691_rtp_Hydra.bin"},
    {"aw8691_rtp_Insert_Coin.bin"},
    {"aw8691_rtp_Jumping_Dots.bin"},
    {"aw8691_rtp_Keys.bin"},
    {"aw8691_rtp_Loopy.bin"},
    {"aw8691_rtp_Loopy_Lounge.bin"},
    {"aw8691_rtp_Modular.bin"},
    {"aw8691_rtp_Momentum.bin"},
    {"aw8691_rtp_Morning.bin"},
    {"aw8691_rtp_Moto.bin"},
    {"aw8691_rtp_Natural.bin"},
    {"aw8691_rtp_New_Player.bin"},
    {"aw8691_rtp_Onward.bin"},
    {"aw8691_rtp_Organ_Dub.bin"},
    {"aw8691_rtp_Overclocked.bin"},
    {"aw8691_rtp_Pegasus.bin"},
    {"aw8691_rtp_Pyxis.bin"},
    {"aw8691_rtp_Regrade.bin"},
    {"aw8691_rtp_Scarabaeus.bin"},
    {"aw8691_rtp_Sceptrum.bin"},
    {"aw8691_rtp_Simple.bin"},
    {"aw8691_rtp_Solarium.bin"},
    {"aw8691_rtp_Sparse.bin"},
    {"aw8691_rtp_Terrabytes.bin"},
    {"aw8691_rtp_TJINGLE.bin"},
};

struct aw8691_container *aw8691_rtp;
struct aw8691 *g_aw8691;

/******************************************************
 *
 * functions
 *
 ******************************************************/
static void aw8691_interrupt_clear(struct aw8691 *aw8691);
static void aw8691_vibrate(struct aw8691 *aw8691, int value);

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
        if(ret < 0) {
            pr_err("%s: regmap_write cnt=%d error=%d\n", __func__, cnt, ret);
        } else {
            break;
        }
#else
        ret = i2c_smbus_write_byte_data(aw8691->i2c, reg_addr, reg_data);
        if(ret < 0) {
            pr_err("%s: i2c_write cnt=%d error=%d\n", __func__, cnt, ret);
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
        if(ret < 0) {
            pr_err("%s: regmap_read cnt=%d error=%d\n", __func__, cnt, ret);
        } else {
            break;
        }
#else
        ret = i2c_smbus_read_byte_data(aw8691->i2c, reg_addr);
        if(ret < 0) {
            pr_err("%s: i2c_read cnt=%d error=%d\n", __func__, cnt, ret);
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

    data = kmalloc(len+1, GFP_KERNEL);
    if (data == NULL) {
        pr_err("%s: can not allocate memory\n", __func__);
        return  -ENOMEM;
    }

    data[0] = reg_addr;
    memcpy(&data[1], buf, len);

    ret = i2c_master_send(aw8691->i2c, data, len+1);
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
        pr_err("%s: failed to read %s\n", __func__, aw8691_rtp_name[aw8691->rtp_file_num]);
        release_firmware(cont);
        return;
    }

    pr_debug("%s: loaded %s - size: %zu\n", __func__, aw8691_rtp_name[aw8691->rtp_file_num],
                    cont ? cont->size : 0);

    /* aw8691 rtp update */
    aw8691_rtp = kzalloc(cont->size+sizeof(int), GFP_KERNEL);
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

    /* Vibrate for 1 second as TI drv2624's auto-calibration does */
    aw8691_vibrate(aw8691, 1000);
}

static int aw8691_rtp_update(struct aw8691 *aw8691)
{
    pr_debug("%s enter\n", __func__);

    return request_firmware_nowait(THIS_MODULE, FW_ACTION_HOTPLUG,
                aw8691_rtp_name[aw8691->rtp_file_num], aw8691->dev, GFP_KERNEL,
                aw8691, aw8691_rtp_loaded);
}


 static void aw8691_container_update(struct aw8691 *aw8691, 
        struct aw8691_container *aw8691_cont)
{
    int i = 0;
    unsigned int shift = 0;

    pr_debug("%s enter\n", __func__);

    aw8691->ram.baseaddr_shift = 2;
    aw8691->ram.ram_shift = 4;

    /* RAMINIT Enable */
    aw8691_i2c_write_bits(aw8691, AW8691_REG_SYSCTRL, 
            AW8691_BIT_SYSCTRL_RAMINIT_MASK, AW8691_BIT_SYSCTRL_RAMINIT_EN);

    /* base addr */
    shift = aw8691->ram.baseaddr_shift;
    aw8691->ram.base_addr = (unsigned int)((aw8691_cont->data[0+shift]<<8) | 
            (aw8691_cont->data[1+shift]));
    pr_debug("%s: base_addr=0x%4x\n", __func__, aw8691->ram.base_addr);
    
    aw8691_i2c_write(aw8691, AW8691_REG_BASE_ADDRH, aw8691_cont->data[0+shift]);
    aw8691_i2c_write(aw8691, AW8691_REG_BASE_ADDRL, aw8691_cont->data[1+shift]);
    
    aw8691_i2c_write(aw8691, AW8691_REG_FIFO_AEH, 
                    (unsigned char)((aw8691->ram.base_addr>>2)>>8));
    aw8691_i2c_write(aw8691, AW8691_REG_FIFO_AEL, 
                    (unsigned char)((aw8691->ram.base_addr>>2)&0x00FF));
    aw8691_i2c_write(aw8691, AW8691_REG_FIFO_AFH, 
                    (unsigned char)((aw8691->ram.base_addr-(aw8691->ram.base_addr>>2))>>8));
    aw8691_i2c_write(aw8691, AW8691_REG_FIFO_AFL, 
                    (unsigned char)((aw8691->ram.base_addr-(aw8691->ram.base_addr>>2))&0x00FF));

    /* ram */
    shift = aw8691->ram.baseaddr_shift;
    aw8691_i2c_write(aw8691, AW8691_REG_RAMADDRH, aw8691_cont->data[0+shift]);
    aw8691_i2c_write(aw8691, AW8691_REG_RAMADDRL, aw8691_cont->data[1+shift]);
    shift = aw8691->ram.ram_shift;
    for(i=shift; i<aw8691_cont->len; i++) {
        aw8691_i2c_write(aw8691, AW8691_REG_RAMDATA, aw8691_cont->data[i]);
    }

#if 0
    /* ram check */
    shift = aw8691->ram.baseaddr_shift;
    aw8691_i2c_write(aw8691, AW8691_REG_RAMADDRH, aw8691_cont->data[0+shift]);
    aw8691_i2c_write(aw8691, AW8691_REG_RAMADDRL, aw8691_cont->data[1+shift]);
    shift = aw8691->ram.ram_shift;
    for(i=shift; i<aw8691_cont->len; i++) {
        aw8691_i2c_read(aw8691, AW8691_REG_RAMDATA, &reg_val);
        if(reg_val != aw8691_cont->data[i]) {
            pr_err("%s: ram check error addr=0x%04x, file_data=0x%02x, ram_data=0x%02x\n",
                        __func__, i, aw8691_cont->data[i], reg_val);
            return;
        }
    }
#endif

    /* RAMINIT Disable */
    aw8691_i2c_write_bits(aw8691, AW8691_REG_SYSCTRL, 
            AW8691_BIT_SYSCTRL_RAMINIT_MASK, AW8691_BIT_SYSCTRL_RAMINIT_OFF);

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
/*
    for(i=0; i<cont->size; i++) {
        pr_info("%s: addr:0x%04x, data:0x%02x\n", __func__, i, *(cont->data+i));
    }
*/

    /* check sum */
    for(i=2; i<cont->size; i++) {
        check_sum += cont->data[i];
    }
    if(check_sum != (unsigned short)((cont->data[0]<<8)|(cont->data[1]))) {
        pr_err("%s: check sum err: check_sum=0x%04x\n", __func__, check_sum);
        return;
    } else {
        pr_debug("%s: check sum pass : 0x%04x\n", __func__, check_sum);
        aw8691->ram.check_sum = check_sum;
    }

    /* aw8691 ram update */
    aw8691_fw = kzalloc(cont->size+sizeof(int), GFP_KERNEL);
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
                aw8691_ram_name, aw8691->dev, GFP_KERNEL,
                aw8691, aw8691_ram_loaded);
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
            ktime_set(ram_timer_val/1000, (ram_timer_val%1000)*1000000), 
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
    switch(play_mode) {
        case AW8691_HAPTIC_STANDBY_MODE:
            aw8691->play_mode = AW8691_HAPTIC_STANDBY_MODE;
            break;
        case AW8691_HAPTIC_RAM_MODE:
            aw8691->play_mode = AW8691_HAPTIC_RAM_MODE;
            aw8691_i2c_write_bits(aw8691, AW8691_REG_SYSCTRL, 
                    AW8691_BIT_SYSCTRL_PLAY_MODE_MASK, AW8691_BIT_SYSCTRL_PLAY_MODE_RAM);
            break;
        case AW8691_HAPTIC_RTP_MODE:
            aw8691->play_mode = AW8691_HAPTIC_RTP_MODE;
            aw8691_i2c_write_bits(aw8691, AW8691_REG_SYSCTRL, 
                    AW8691_BIT_SYSCTRL_PLAY_MODE_MASK, AW8691_BIT_SYSCTRL_PLAY_MODE_RTP);
            break;
        case AW8691_HAPTIC_TRIG_MODE:
            aw8691->play_mode = AW8691_HAPTIC_TRIG_MODE;
            aw8691_i2c_write_bits(aw8691, AW8691_REG_SYSCTRL, 
                    AW8691_BIT_SYSCTRL_PLAY_MODE_MASK, AW8691_BIT_SYSCTRL_PLAY_MODE_RAM);
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
    aw8691_haptic_play_mode(aw8691, AW8691_HAPTIC_STANDBY_MODE);
    aw8691_i2c_write_bits(aw8691, AW8691_REG_SYSCTRL, 
            AW8691_BIT_SYSCTRL_WORK_MODE_MASK, AW8691_BIT_SYSCTRL_STANDBY);
    msleep(1);
    /* RAMINIT Disable */
    aw8691_i2c_write_bits(aw8691, AW8691_REG_SYSCTRL,
        AW8691_BIT_SYSCTRL_RAMINIT_MASK, AW8691_BIT_SYSCTRL_RAMINIT_OFF);
    /* RAMINIT Disable End */
    return 0;
}

static int aw8691_haptic_start(struct aw8691 *aw8691)
{
    aw8691_i2c_write_bits(aw8691, AW8691_REG_SYSCTRL, 
            AW8691_BIT_SYSCTRL_WORK_MODE_MASK, AW8691_BIT_SYSCTRL_ACTIVE);
    /* RAMINIT Enable */
    aw8691_i2c_write_bits(aw8691, AW8691_REG_SYSCTRL,
            AW8691_BIT_SYSCTRL_RAMINIT_MASK, AW8691_BIT_SYSCTRL_RAMINIT_EN);
    /* RAMINIT Enable End */
    aw8691_interrupt_clear(aw8691);
    aw8691_i2c_write(aw8691, AW8691_REG_PROC_CTRL,
            AW8691_BIT_PROC_CTRL_GO);
    return 0;
}

static int aw8691_haptic_set_repeat_seq(struct aw8691 *aw8691, unsigned char flag)
{
    if(flag) {
        aw8691_i2c_write_bits(aw8691, AW8691_REG_DATACTRL,
                AW8691_BIT_DATACTRL_WAV_DBG_MASK, AW8691_BIT_DATACTRL_WAV_DBG);
    } else {
        aw8691_i2c_write_bits(aw8691, AW8691_REG_DATACTRL,
                AW8691_BIT_DATACTRL_WAV_DBG_MASK, AW8691_BIT_DATACTRL_WAV_NORMAL);
    }

    return 0;
}

static int aw8691_haptic_set_repeat_que_seq(struct aw8691 *aw8691, unsigned char seq)
{
    unsigned char i;
    for(i=0; i<AW8691_SEQUENCER_SIZE; i++) {
        aw8691_i2c_write(aw8691, AW8691_REG_QUE_SEQ1+i,
                seq);
    }
    return 0;
}

static int aw8691_haptic_set_que_seq(struct aw8691 *aw8691, unsigned int seq)
{
    unsigned char i = 0;
    struct aw8691_que_seq que_seq;

    for(i=0; i<AW8691_SEQUENCER_SIZE; i++) {
        que_seq.index[i] = (seq>>((AW8691_SEQUENCER_SIZE-i-1)*8))&0xFF;
    }

    for(i=0; i<AW8691_SEQUENCER_SIZE; i++) {
        aw8691_i2c_write(aw8691, AW8691_REG_QUE_SEQ1+i,
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

    for(i=0; i<AW8691_SEQUENCER_LOOP_SIZE; i++) {
        aw8691_i2c_write(aw8691, AW8691_REG_SEQ_LOOP1+i, tmp[i]);
    }

    return 0;
}


static int aw8691_set_que_seq(struct aw8691 *aw8691, unsigned long arg)
{
    int ret = 0;
    unsigned char i = 0;
    struct aw8691_que_seq que_seq;

    if (copy_from_user(&que_seq,
        (void __user *)arg, sizeof(struct aw8691_que_seq)))
        return -EFAULT;

    for(i=0; i<AW8691_SEQUENCER_SIZE; i++) {
        aw8691_i2c_write(aw8691, AW8691_REG_QUE_SEQ1+i, que_seq.index[i]);
    }

    return ret;
}

static int aw8691_set_seq_loop(struct aw8691 *aw8691, unsigned long arg)
{
    struct aw8691_seq_loop seq_loop;
    int ret = 0;
    unsigned char i = 0;
    unsigned char loop[2] = {0, 0 };

    if (copy_from_user(&seq_loop, 
        (void __user *)arg, sizeof(struct aw8691_seq_loop)))
        return -EFAULT;

    for(i=0; i<AW8691_SEQUENCER_SIZE; i++) {
        if(seq_loop.loop[i] & 0xF0) {
            dev_err(aw8691->dev, "%s: seq_loop err loop[%d]=0x%02x\n",
                    __func__, i, seq_loop.loop[i]);
            return -1;
        }
    }
    loop[0] |= (seq_loop.loop[0]<<4);
    loop[0] |= (seq_loop.loop[1]<<0);
    loop[1] |= (seq_loop.loop[2]<<4);
    loop[1] |= (seq_loop.loop[3]<<0);

    for(i=0; i<(AW8691_SEQUENCER_SIZE>>1); i++) {
        aw8691_i2c_write(aw8691, AW8691_REG_SEQ_LOOP1+i, loop[i]);
    }

    return ret;
}

static int aw8691_haptic_set_bst_vol(struct aw8691 *aw8691, unsigned char bst_vol)
{
    aw8691_i2c_write_bits(aw8691, AW8691_REG_BSTCFG,
            AW8691_BIT_BSTCFG_BSTVOL_MASK, bst_vol);
    return 0;
}

static int aw8691_haptic_set_bst_peak_cur(struct aw8691 *aw8691, unsigned char peak_cur)
{
    aw8691_i2c_write_bits(aw8691, AW8691_REG_BSTCFG,
            AW8691_BIT_BSTCFG_PEAKCUR_MASK, peak_cur);
    return 0;
}

static int aw8691_haptic_set_gain(struct aw8691 *aw8691, unsigned char gain)
{
    if(gain & 0x80) {
        gain = 0x7F;
    }

    aw8691_i2c_write(aw8691, AW8691_REG_DATDBG, gain);

    return 0;
}

#ifdef AW8691_HAPTIC_VBAT_MONITOR
static int aw8691_haptic_set_bst_mode(struct aw8691 *aw8691, unsigned char mode)
{
    if(mode == AW8691_HAPTIC_BYPASS_MODE) {
        aw8691_i2c_write_bits(aw8691, AW8691_REG_SYSCTRL,
                AW8691_BIT_SYSCTRL_BST_MODE_MASK, AW8691_BIT_SYSCTRL_BST_MODE_BYPASS);
    } else if (mode == AW8691_HAPTIC_BOOST_MODE){
        aw8691_i2c_write_bits(aw8691, AW8691_REG_SYSCTRL,
                AW8691_BIT_SYSCTRL_BST_MODE_MASK, AW8691_BIT_SYSCTRL_BST_MODE_BOOST);
    }

    return 0;
}
#endif

static int aw8691_haptic_play_que_seq(struct aw8691 *aw8691, unsigned char flag)
{
    aw8691_haptic_stop(aw8691);
    if(flag) {
        aw8691_haptic_play_mode(aw8691, AW8691_HAPTIC_RAM_MODE);
        aw8691_haptic_set_repeat_seq(aw8691, 0);
        aw8691_haptic_start(aw8691);
    }
    return 0;
}

static int aw8691_haptic_play_repeat_seq(struct aw8691 *aw8691, unsigned char flag)
{
    aw8691_haptic_stop(aw8691);
    if(flag) {
        aw8691_haptic_play_mode(aw8691, AW8691_HAPTIC_RAM_MODE);
        aw8691_haptic_set_repeat_seq(aw8691, 1);
        aw8691_haptic_start(aw8691);
    }

    return 0;
}

static void aw8691_haptic_set_rtp_aei(struct aw8691 *aw8691, bool flag)
{
    if(flag) {
        aw8691_i2c_write_bits(aw8691, AW8691_REG_SYSINTM,
                AW8691_BIT_SYSINTM_FF_AE_MASK, AW8691_BIT_SYSINTM_FF_AE_EN);
    } else {
        aw8691_i2c_write_bits(aw8691, AW8691_REG_SYSINTM,
                AW8691_BIT_SYSINTM_FF_AE_MASK, AW8691_BIT_SYSINTM_FF_AE_OFF);        
    }
}
/*
static void aw8691_haptic_set_rtp_afi(struct aw8691 *aw8691, bool flag)
{
    if(flag) {
        aw8691_i2c_write_bits(aw8691, AW8691_REG_SYSINTM,
                AW8691_BIT_SYSINTM_FF_AF_MASK, AW8691_BIT_SYSINTM_FF_AF_EN);
    } else {
        aw8691_i2c_write_bits(aw8691, AW8691_REG_SYSINTM,
                AW8691_BIT_SYSINTM_FF_AF_MASK, AW8691_BIT_SYSINTM_FF_AF_OFF);        
    }
}
*/
/*
static unsigned char aw8691_haptic_rtp_get_fifo_aei(struct aw8691 *aw8691)
{
    unsigned char ret;
    unsigned char reg_val = 0;

    aw8691_i2c_read(aw8691, AW8691_REG_SYSINT, &reg_val);
    reg_val &= AW8691_BIT_SYSINT_FF_AEI;
    ret = reg_val>>4;

    return ret;
}
*/
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
    ret = reg_val>>3;

    return ret;
}

static int aw8691_haptic_rtp_init(struct aw8691 *aw8691)
{
    unsigned int buf_len = 0;

    pr_debug("%s enter\n", __func__);

    aw8691->rtp_cnt = 0;

    while((!aw8691_haptic_rtp_get_fifo_afi(aw8691)) && 
            (aw8691->play_mode == AW8691_HAPTIC_RTP_MODE)) {
        pr_debug("%s rtp cnt = %d\n", __func__, aw8691->rtp_cnt);
        if((aw8691_rtp->len-aw8691->rtp_cnt) < (aw8691->ram.base_addr>>3)) {
            buf_len = aw8691_rtp->len-aw8691->rtp_cnt;
        } else {
            buf_len = (aw8691->ram.base_addr>>3);
        }
        aw8691_i2c_writes(aw8691, AW8691_REG_RTP_DATA,
                &aw8691_rtp->data[aw8691->rtp_cnt], buf_len);
        aw8691->rtp_cnt += buf_len;
        if(aw8691->rtp_cnt == aw8691_rtp->len) {
            pr_debug("%s: rtp update complete\n", __func__);
            aw8691->rtp_cnt = 0;
#ifdef AW8691_HAPTIC_VBAT_MONITOR
            aw8691_haptic_set_bst_mode(aw8691, AW8691_HAPTIC_BOOST_MODE);
#endif
            return 0;
        }
    }

    if(aw8691->play_mode == AW8691_HAPTIC_RTP_MODE) {
        aw8691_haptic_set_rtp_aei(aw8691, true);
    }

    pr_debug("%s exit\n", __func__);

    return 0;
}

static void aw8691_rtp_work_routine(struct work_struct *work)
{
#ifdef AW8691_HAPTIC_VBAT_MONITOR
    int i = 0;
    int tmp = 0;
    unsigned int vbat = 0;
#endif
    const struct firmware *rtp_file;
    int ret = -1;
    struct aw8691 *aw8691 = container_of(work, struct aw8691, rtp_work);

    pr_debug("%s enter\n", __func__);

    /* fw loaded */
    ret = request_firmware(&rtp_file,
            aw8691_rtp_name[aw8691->rtp_file_num],
            aw8691->dev);
    if(ret < 0)
    {
        pr_err("%s: failed to read %s\n", __func__,
                aw8691_rtp_name[aw8691->rtp_file_num]);
        return ;
    }
    aw8691->rtp_init = 0;
    kfree(aw8691_rtp);
    aw8691_rtp = kzalloc(rtp_file->size+sizeof(int), GFP_KERNEL);
    if (!aw8691_rtp) {
        release_firmware(rtp_file);
        pr_err("%s: error allocating memory\n", __func__);
        return;
    }
    aw8691_rtp->len = rtp_file->size;
    pr_debug("%s: rtp file [%s] size = %d\n", __func__,
            aw8691_rtp_name[aw8691->rtp_file_num], aw8691_rtp->len);
    memcpy(aw8691_rtp->data, rtp_file->data, rtp_file->size);
    release_firmware(rtp_file);

    aw8691->rtp_init = 1;

    /* rtp mode config */
    aw8691_haptic_play_mode(aw8691, AW8691_HAPTIC_RTP_MODE);
    aw8691_i2c_write_bits(aw8691, AW8691_REG_PWMDBG,
           AW8691_BIT_PWMDBG_PWMCLK_MODE_MASK, AW8691_BIT_PWMDBG_PWMCLK_MODE_12KB);
    aw8691_i2c_write_bits(aw8691, AW8691_REG_SYSCTRL,
            AW8691_BIT_SYSCTRL_WORK_MODE_MASK, AW8691_BIT_SYSCTRL_ACTIVE);
    msleep(2);
#ifdef AW8691_HAPTIC_VBAT_MONITOR
    vbat = aw8691_get_sys_battery_info(SYS_BAT_DEV);
    pr_debug("%s: get sys battery = %d\n", __func__, vbat);
    if((vbat > AW8691_SYS_VBAT_MIN) && (vbat < AW8691_SYS_VBAT_MAX)) {
        for(i=0; i<aw8691_rtp->len; i++) {
            tmp = (int)aw8691_rtp->data[i];
            tmp = tmp*AW8691_SYS_VBAT_REFERENCE/vbat;
            aw8691_rtp->data[i] = (unsigned char)tmp;
        }
    }
#endif
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

static long aw8691_file_unlocked_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    struct aw8691 *aw8691 = (struct aw8691 *)file->private_data;

    int ret = 0;
    unsigned int temp = 0;
 
    dev_info(aw8691->dev, "%s: cmd=0x%x, arg=0x%lx\n",
              __func__, cmd, arg);

    mutex_lock(&aw8691->lock);
   
    if(_IOC_TYPE(cmd) != AW8691_HAPTIC_IOCTL_MAGIC) {
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
            aw8691_haptic_play_que_seq(aw8691, (unsigned char)temp);
            break;
        case AW8691_HAPTIC_SET_BST_VOL:
            if (copy_from_user(&temp, (void __user *)arg, sizeof(int)))
                return -EFAULT;
            aw8691_haptic_set_bst_vol(aw8691, (unsigned char)(temp<<3));
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

static ssize_t aw8691_file_read(struct file* filp, char* buff, size_t len, loff_t* offset)
{
    struct aw8691 *aw8691 = (struct aw8691 *)filp->private_data;
    int ret = 0;
    int i = 0;
    unsigned char reg_val = 0;
    unsigned char *pbuff = NULL;
    
    mutex_lock(&aw8691->lock);

    dev_info(aw8691->dev, "%s: len=%zu\n", __func__, len);

    switch(aw8691->fileops.cmd)
    {
        case AW8691_HAPTIC_CMD_READ_REG:
            pbuff = (unsigned char *)kzalloc(len, GFP_KERNEL);
            if(pbuff != NULL) {
                for(i=0; i<len; i++) {
                    aw8691_i2c_read(aw8691, aw8691->fileops.reg+i, &reg_val);
                    pbuff[i] = reg_val;
                }
                ret = copy_to_user(buff, pbuff, len);
                if(ret) {
                    dev_err(aw8691->dev, "%s: copy to user fail\n", __func__);
                }
                kfree(pbuff);
            } else {
                dev_err(aw8691->dev, "%s: alloc memory fail\n", __func__);
            }
            break;
        case AW8691_HAPTIC_CMD_READ_FIRMWARE:
            pbuff = (unsigned char *)kzalloc(len, GFP_KERNEL);
            if(pbuff != NULL) {
                aw8691_haptic_stop(aw8691);
                /* RAMINIT Enable */
                aw8691_i2c_write_bits(aw8691, AW8691_REG_SYSCTRL,
                        AW8691_BIT_SYSCTRL_RAMINIT_MASK, AW8691_BIT_SYSCTRL_RAMINIT_EN);
 
                aw8691_i2c_write(aw8691, AW8691_REG_RAMADDRH, aw8691->fileops.ram_addrh);
                aw8691_i2c_write(aw8691, AW8691_REG_RAMADDRL, aw8691->fileops.ram_addrl);
                for(i=0; i<len; i++) {
                    aw8691_i2c_read(aw8691, AW8691_REG_RAMDATA, &reg_val);
                    pbuff[i] = reg_val;
                }

                /* RAMINIT Disable */
                aw8691_i2c_write_bits(aw8691, AW8691_REG_SYSCTRL,
                        AW8691_BIT_SYSCTRL_RAMINIT_MASK, AW8691_BIT_SYSCTRL_RAMINIT_OFF);

                ret = copy_to_user(buff, pbuff, len);
                if(ret) {
                    dev_err(aw8691->dev, "%s: copy to user fail\n", __func__);
                }
                kfree(pbuff);
            } else {
                dev_err(aw8691->dev, "%s: alloc memory fail\n", __func__);                
            }
            break;
        default:
            dev_err(aw8691->dev, "%s, unknown cmd %d \n", __func__, aw8691->fileops.cmd);
            break;
    }

    mutex_unlock(&aw8691->lock);

    for(i=0; i<len; i++) {
        dev_info(aw8691->dev, "%s: buff[%d]=0x%02x\n", 
                __func__, i, buff[i]);
    }

    return len;
}

static ssize_t aw8691_file_write(struct file* filp, const char* buff, size_t len, loff_t* off)
{
    struct aw8691 *aw8691 = (struct aw8691 *)filp->private_data;
    int i = 0;
    int ret = 0;
    unsigned char *pbuff = NULL;

    for(i=0; i<len; i++) {
        dev_info(aw8691->dev, "%s: buff[%d]=0x%02x\n", 
                __func__, i, buff[i]);
    }

    mutex_lock(&aw8691->lock);
    
    aw8691->fileops.cmd = buff[0];
    
    switch(aw8691->fileops.cmd)
    {
        case AW8691_HAPTIC_CMD_READ_REG:
            if(len == 2) {
                aw8691->fileops.reg = buff[1];
            } else {
                dev_err(aw8691->dev, "%s: read cmd len %zu err\n", __func__, len);
            }
            break;
        case AW8691_HAPTIC_CMD_WRITE_REG:
            if(len > 2) {
                pbuff = (unsigned char *)kzalloc(len, GFP_KERNEL);
                if(pbuff != NULL) {
                    ret = copy_from_user(pbuff, buff, len);
                    if(ret) {
                        dev_err(aw8691->dev, "%s: copy from user fail\n", __func__);
                    } else {
                        for(i=0; i<len-2; i++) {
                            dev_info(aw8691->dev, "%s: write reg0x%02x=0x%02x\n", 
                                    __func__, pbuff[1]+i, pbuff[i+2]);
                            aw8691_i2c_write(aw8691, pbuff[1]+i, pbuff[2+i]);
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
            if(pbuff != NULL) {
                ret = copy_from_user(pbuff, &buff[1], len-1);
                if(ret) {
                    dev_err(aw8691->dev, "%s: copy from user fail\n", __func__);
                } else {
                    __pm_stay_awake(aw8691->ws);

                    aw8691_haptic_stop(aw8691);
                    /* RAMINIT Enable */
                    aw8691_i2c_write_bits(aw8691, AW8691_REG_SYSCTRL,
                            AW8691_BIT_SYSCTRL_RAMINIT_MASK, AW8691_BIT_SYSCTRL_RAMINIT_EN);

                    aw8691_i2c_write(aw8691, AW8691_REG_RAMADDRH, aw8691->fileops.ram_addrh);
                    aw8691_i2c_write(aw8691, AW8691_REG_RAMADDRL, aw8691->fileops.ram_addrl);
                    for(i=0; i<len-1; i++) {
                        aw8691_i2c_write(aw8691, AW8691_REG_RAMDATA, pbuff[i]);
                    }

                    /* RAMINIT Disable */
                    aw8691_i2c_write_bits(aw8691, AW8691_REG_SYSCTRL,
                            AW8691_BIT_SYSCTRL_RAMINIT_MASK, AW8691_BIT_SYSCTRL_RAMINIT_OFF);

                    __pm_relax(aw8691->ws);
                }
                kfree(pbuff);
            } else {
                dev_err(aw8691->dev, "%s: alloc memory fail\n", __func__);                
            }

            break;
        case AW8691_HAPTIC_CMD_READ_FIRMWARE:
            if(len == 3) {
                aw8691->fileops.ram_addrh = buff[1];
                aw8691->fileops.ram_addrl = buff[2];
            } else {
                dev_err(aw8691->dev, "%s: read firmware len %zu err\n", __func__, len);
            }
            break;
        default:
            dev_err(aw8691->dev, "%s, unknown cmd %d \n", __func__, aw8691->fileops.cmd);
        break;
    }

    mutex_unlock(&aw8691->lock);

    return len;
}

static struct file_operations fops =
{
    .owner = THIS_MODULE,
    .read = aw8691_file_read,
    .write = aw8691_file_write,
    .unlocked_ioctl = aw8691_file_unlocked_ioctl,
    .open = aw8691_file_open,
    .release = aw8691_file_release,
};

static struct miscdevice aw8691_haptic_misc =
{
    .minor = MISC_DYNAMIC_MINOR,
    .name = AW8691_HAPTIC_NAME,
    .fops = &fops,
};

static int aw8691_haptic_init(struct aw8691 *aw8691)
{
    int ret = 0;

    pr_info("%s enter\n", __func__);

    ret = misc_register(&aw8691_haptic_misc);
    if(ret) {
        dev_err(aw8691->dev,  "%s: misc fail: %d\n", __func__, ret);
        return ret;
    }

    aw8691_i2c_write_bits(aw8691, AW8691_REG_SYSCTRL, 
            AW8691_BIT_SYSCTRL_WORK_MODE_MASK, AW8691_BIT_SYSCTRL_STANDBY);
    aw8691_i2c_write(aw8691, AW8691_REG_ANACTRL, 0x01);
    //aw8691_i2c_write_bits(aw8691, AW8691_REG_PWMDBG,
    //        AW8691_BIT_PWMDBG_PWMCLK_MODE_MASK, AW8691_BIT_PWMDBG_PWMCLK_MODE_24KB);
    return ret;
}



/*****************************************************
 *
 * vibrator
 *
 *****************************************************/
static void aw8691_rtp_play(struct aw8691 *aw8691, int value)
{
	aw8691_haptic_stop(aw8691);
	aw8691_haptic_set_rtp_aei(aw8691, false);
	aw8691_interrupt_clear(aw8691);
#if 0
#ifdef AW8691_HAPTIC_VBAT_MONITOR
	aw8691_haptic_set_bst_mode(aw8691, AW8691_HAPTIC_BYPASS_MODE);
#endif
#else
	aw8691_haptic_set_bst_vol(aw8691, AW8691_BIT_BSTCFG_BSTVOL_8V);
#endif
	if(value < (sizeof(aw8691_rtp_name)/AW8691_RTP_NAME_MAX)) {
		aw8691->rtp_file_num = value;
		if(value) {
			schedule_work(&aw8691->rtp_work);
		}
	} else {
		pr_err("%s: rtp_file_num 0x%02x over max value \n", __func__, aw8691->rtp_file_num);
	}
}

static void aw8691_haptic_context(struct aw8691 *aw8691, enum aw8691_haptic_mode cmd)
{
	int t_top = 0;
	if (!gpio_is_valid(aw8691->haptic_context_gpio)) {
		pr_debug("%s haptic context gpio is invalid \n", __func__);
		return;
	}

	t_top = gpio_get_value(aw8691->haptic_context_gpio);
	if (t_top) {
		switch (cmd) {
		case HAPTIC_RTP:
			aw8691->gain = 0x20;
			break;
		case HAPTIC_SHORT:
			aw8691->gain = 0x20;
			break;
		case HAPTIC_LONG:
			aw8691->gain = 0x06;
			break;
		default:
			break;
		}
	}
}

static void aw8691_vibrate(struct aw8691 *aw8691, int value)
{
	int seq;
	mutex_lock(&aw8691->lock);

	aw8691_haptic_stop(aw8691);
	seq = (aw8691->seq >> ((AW8691_SEQUENCER_SIZE - 1) * 8)) & 0xFF;
	pr_debug("%s: value=%d, seq=%d\n", __FUNCTION__, value, seq);

	if (value > 0 || seq > 2) {

		if (seq >= AW8691_SEQ_NO_RTP_BASE) {
			aw8691->haptic_mode = HAPTIC_RTP;
			aw8691->gain = 0x20;
		} else if (value < 100 || seq > 2) {
			aw8691->haptic_mode = HAPTIC_SHORT;
			aw8691->gain = 0x20;
		} else {
			aw8691->haptic_mode = HAPTIC_LONG;
			aw8691->gain = 0x0e;
		}

		if(!aw8691->factory_mode)
			aw8691_haptic_context(aw8691,aw8691->haptic_mode);

		if (aw8691->debugfs_debug)
			aw8691_haptic_set_gain(aw8691, aw8691->gain_debug);
		else
			aw8691_haptic_set_gain(aw8691, aw8691->gain);

		switch (aw8691->haptic_mode) {
		case HAPTIC_RTP:
			aw8691_rtp_play(aw8691, seq - AW8691_SEQ_NO_RTP_BASE);
			break;
		case HAPTIC_SHORT:
			aw8691_i2c_write_bits(aw8691, AW8691_REG_PWMDBG,
				AW8691_BIT_PWMDBG_PWMCLK_MODE_MASK,
				AW8691_BIT_PWMDBG_PWMCLK_MODE_12KB);
			aw8691_haptic_set_bst_vol(aw8691, AW8691_BIT_BSTCFG_BSTVOL_8P75V);
			//aw8691_haptic_set_peak_cur(aw8691, AW8691_BIT_BSTCFG_PEAKCUR_3P5A);

			if (aw8691->seq == 0)
				aw8691->seq = 0x01000000;

			aw8691_haptic_set_que_seq(aw8691, aw8691->seq);
			//aw8691_haptic_set_repeat_seq(aw8691, 0);
			//aw8691->index = 0x01;
			aw8691_haptic_play_que_seq(aw8691, 0x01);
		/*
			value = (value>HAPTIC_MAX_TIMEOUT)? HAPTIC_MAX_TIMEOUT:value;
			hrtimer_start(&aw8691->timer,
			ns_to_ktime((u64)value * NSEC_PER_MSEC), HRTIMER_MODE_REL);
		*/
			break;
		case HAPTIC_LONG:
			aw8691_i2c_write_bits(aw8691, AW8691_REG_PWMDBG,
				AW8691_BIT_PWMDBG_PWMCLK_MODE_MASK,
				AW8691_BIT_PWMDBG_PWMCLK_MODE_12KB);
			aw8691_haptic_set_bst_vol(aw8691, AW8691_BIT_BSTCFG_BSTVOL_6P25V);

			aw8691->duration = value;
			/* wav index config */
			aw8691->index = 0x02;
			aw8691_haptic_set_repeat_que_seq(aw8691, aw8691->index);

			__pm_wakeup_event(aw8691->ws, value + 100);
			/* run ms timer */
			hrtimer_cancel(&aw8691->timer);
			aw8691->state = 0x01;
			if (aw8691->state)
			{
			hrtimer_start(&aw8691->timer,
				ktime_set(aw8691->duration / 1000, (value % 1000) * 1000000),
				HRTIMER_MODE_REL);
			}
			schedule_work(&aw8691->vibrator_work);
			break;
		default:
			break;
		}

		/* Restore to default short waveform */
		if (seq > 2)
			aw8691->seq = 0;
	}

    mutex_unlock(&aw8691->lock);
}

#ifdef TIMED_OUTPUT
static int aw8691_vibrator_get_time(struct timed_output_dev *dev)
{
    struct aw8691 *aw8691 = container_of(dev, struct aw8691, to_dev);

    if (hrtimer_active(&aw8691->timer)) {
        ktime_t r = hrtimer_get_remaining(&aw8691->timer);
        return ktime_to_ms(r);
    }

    return 0;
}

static void aw8691_vibrator_enable( struct timed_output_dev *dev, int value)
{
    struct aw8691 *aw8691 = container_of(dev, struct aw8691, to_dev);

    pr_debug("%s enter, value=%d\n", __func__, value);
    aw8691_vibrate(aw8691, value);
    pr_debug("%s exit\n", __func__);
}

#else
static enum led_brightness aw8691_haptic_brightness_get(struct led_classdev *cdev)
{
    struct aw8691 *aw8691 =
        container_of(cdev, struct aw8691, cdev);

    return aw8691->amplitude;
}

static void aw8691_haptic_brightness_set(struct led_classdev *cdev,
                enum led_brightness level)
{
    struct aw8691 *aw8691 =
        container_of(cdev, struct aw8691, cdev);

    aw8691->amplitude = level;

    mutex_lock(&aw8691->lock);

    aw8691_haptic_stop(aw8691);
    if (aw8691->amplitude > 0) {
        aw8691_haptic_play_que_seq(aw8691, aw8691->amplitude);
    }

    mutex_unlock(&aw8691->lock);

}
#endif

static ssize_t aw8691_extra_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
	return 0;
}

static ssize_t aw8691_extra_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
	return count;
}

static ssize_t aw8691_state_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
#ifdef TIMED_OUTPUT
    struct timed_output_dev *to_dev = dev_get_drvdata(dev);
    struct aw8691 *aw8691 = container_of(to_dev, struct aw8691, to_dev);
#else
    struct led_classdev *cdev = dev_get_drvdata(dev);
    struct aw8691 *aw8691 = container_of(cdev, struct aw8691, cdev);
#endif

    return snprintf(buf, PAGE_SIZE, "%d\n", aw8691->state);
}

static ssize_t aw8691_state_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
	return count;
}

static ssize_t aw8691_duration_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
#ifdef TIMED_OUTPUT
    struct timed_output_dev *to_dev = dev_get_drvdata(dev);
    struct aw8691 *aw8691 = container_of(to_dev, struct aw8691, to_dev);
#else
    struct led_classdev *cdev = dev_get_drvdata(dev);
    struct aw8691 *aw8691 = container_of(cdev, struct aw8691, cdev);
#endif
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
#ifdef TIMED_OUTPUT
    struct timed_output_dev *to_dev = dev_get_drvdata(dev);
    struct aw8691 *aw8691 = container_of(to_dev, struct aw8691, to_dev);
#else
    struct led_classdev *cdev = dev_get_drvdata(dev);
    struct aw8691 *aw8691 = container_of(cdev, struct aw8691, cdev);
#endif
    unsigned int val = 0;
    int rc = 0;

    rc = kstrtouint(buf, 0, &val);
    if (rc < 0)
        return rc;

    pr_debug("%s: value=%d\n", __FUNCTION__, val);

    /* setting 0 on duration is NOP for now */
    if (val <= 0)
        return count;

    aw8691->duration = val;

    return count;
}

static ssize_t aw8691_activate_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
#ifdef TIMED_OUTPUT
    struct timed_output_dev *to_dev = dev_get_drvdata(dev);
    struct aw8691 *aw8691 = container_of(to_dev, struct aw8691, to_dev);
#else
    struct led_classdev *cdev = dev_get_drvdata(dev);
    struct aw8691 *aw8691 = container_of(cdev, struct aw8691, cdev);
#endif

    /* For now nothing to show */
    return snprintf(buf, PAGE_SIZE, "%d\n", aw8691->state);
}

static ssize_t aw8691_activate_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
#ifdef TIMED_OUTPUT
    struct timed_output_dev *to_dev = dev_get_drvdata(dev);
    struct aw8691 *aw8691 = container_of(to_dev, struct aw8691, to_dev);
#else
    struct led_classdev *cdev = dev_get_drvdata(dev);
    struct aw8691 *aw8691 = container_of(cdev, struct aw8691, cdev);
#endif
    unsigned int val = 0;
    int rc = 0;

    rc = kstrtouint(buf, 0, &val);
    if (rc < 0)
        return rc;

    if (val != 0 && val != 1)
        return count;

    pr_debug("%s: value=%d\n", __FUNCTION__, val);

    aw8691->state = val;

    if (aw8691->state)
        aw8691_vibrate(aw8691, aw8691->duration);
    else
        aw8691_vibrate(aw8691, 0);

    return count;
}

static ssize_t aw8691_index_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
#ifdef TIMED_OUTPUT
    struct timed_output_dev *to_dev = dev_get_drvdata(dev);
    struct aw8691 *aw8691 = container_of(to_dev, struct aw8691, to_dev);
#else
    struct led_classdev *cdev = dev_get_drvdata(dev);
    struct aw8691 *aw8691 = container_of(cdev, struct aw8691, cdev);
#endif
    unsigned char reg_val = 0;
    aw8691_i2c_read(aw8691, AW8691_REG_QUE_SEQ1, &reg_val);
    aw8691->index = reg_val;

    return snprintf(buf, PAGE_SIZE, "%d\n", aw8691->index);
}

static ssize_t aw8691_index_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
#ifdef TIMED_OUTPUT
    struct timed_output_dev *to_dev = dev_get_drvdata(dev);
    struct aw8691 *aw8691 = container_of(to_dev, struct aw8691, to_dev);
#else
    struct led_classdev *cdev = dev_get_drvdata(dev);
    struct aw8691 *aw8691 = container_of(cdev, struct aw8691, cdev);
#endif
    unsigned int val = 0;
    int rc = 0;

    rc = kstrtouint(buf, 0, &val);
    if (rc < 0)
        return rc;

    pr_debug("%s: value=%d\n", __FUNCTION__, val);

    mutex_lock(&aw8691->lock);
    aw8691->index = val;
    aw8691_haptic_set_repeat_que_seq(aw8691, aw8691->index);
    mutex_unlock(&aw8691->lock);
    return count;
}

static ssize_t aw8691_vmax_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
#ifdef TIMED_OUTPUT
    struct timed_output_dev *to_dev = dev_get_drvdata(dev);
    struct aw8691 *aw8691 = container_of(to_dev, struct aw8691, to_dev);
#else
    struct led_classdev *cdev = dev_get_drvdata(dev);
    struct aw8691 *aw8691 = container_of(cdev, struct aw8691, cdev);
#endif

    return snprintf(buf, PAGE_SIZE, "%d\n", aw8691->vmax);
}

static ssize_t aw8691_vmax_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
#ifdef TIMED_OUTPUT
    struct timed_output_dev *to_dev = dev_get_drvdata(dev);
    struct aw8691 *aw8691 = container_of(to_dev, struct aw8691, to_dev);
#else
    struct led_classdev *cdev = dev_get_drvdata(dev);
    struct aw8691 *aw8691 = container_of(cdev, struct aw8691, cdev);
#endif
    unsigned int val = 0;
    int rc = 0;

    rc = kstrtouint(buf, 0, &val);
    if (rc < 0)
        return rc;

    pr_debug("%s: value=%d\n", __FUNCTION__, val);

    mutex_lock(&aw8691->lock);
    aw8691->vmax = val;
    aw8691_haptic_set_bst_vol(aw8691, (unsigned char)(aw8691->vmax<<3));
    mutex_unlock(&aw8691->lock);
    return count;
}

static ssize_t aw8691_gain_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
#ifdef TIMED_OUTPUT
    struct timed_output_dev *to_dev = dev_get_drvdata(dev);
    struct aw8691 *aw8691 = container_of(to_dev, struct aw8691, to_dev);
#else
    struct led_classdev *cdev = dev_get_drvdata(dev);
    struct aw8691 *aw8691 = container_of(cdev, struct aw8691, cdev);
#endif

    return snprintf(buf, PAGE_SIZE, "%d\n", aw8691->gain_debug);
}

static ssize_t aw8691_gain_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
#ifdef TIMED_OUTPUT
    struct timed_output_dev *to_dev = dev_get_drvdata(dev);
    struct aw8691 *aw8691 = container_of(to_dev, struct aw8691, to_dev);
#else
    struct led_classdev *cdev = dev_get_drvdata(dev);
    struct aw8691 *aw8691 = container_of(cdev, struct aw8691, cdev);
#endif
    unsigned int val = 0;
    int rc = 0;

    rc = kstrtouint(buf, 0, &val);
    if (rc < 0)
        return rc;

    pr_debug("%s: value=%d\n", __FUNCTION__, val);

    mutex_lock(&aw8691->lock);
	if (val > 0)
		aw8691->debugfs_debug = true;
	else
		aw8691->debugfs_debug = false;
    aw8691->gain_debug = val;
    aw8691_haptic_set_gain(aw8691, (unsigned char)aw8691->gain_debug);
    mutex_unlock(&aw8691->lock);
    return count;
}

static ssize_t aw8691_seq_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
#ifdef TIMED_OUTPUT
    struct timed_output_dev *to_dev = dev_get_drvdata(dev);
    struct aw8691 *aw8691 = container_of(to_dev, struct aw8691, to_dev);
#else
    struct led_classdev *cdev = dev_get_drvdata(dev);
    struct aw8691 *aw8691 = container_of(cdev, struct aw8691, cdev);
#endif
    size_t count = 0;
    unsigned char i = 0;
    unsigned char reg_val = 0;

    aw8691->seq = 0;
    for(i=0; i<AW8691_SEQUENCER_SIZE; i++) {
        aw8691_i2c_read(aw8691, AW8691_REG_QUE_SEQ1+i, &reg_val);
        count += snprintf(buf+count, PAGE_SIZE-count,
                "seq%d: 0x%02x\n", i+1, reg_val);
        aw8691->seq |= (reg_val<<((AW8691_SEQUENCER_SIZE-i-1)*8));
    }
    return count;
}

static ssize_t aw8691_seq_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
#ifdef TIMED_OUTPUT
    struct timed_output_dev *to_dev = dev_get_drvdata(dev);
    struct aw8691 *aw8691 = container_of(to_dev, struct aw8691, to_dev);
#else
    struct led_classdev *cdev = dev_get_drvdata(dev);
    struct aw8691 *aw8691 = container_of(cdev, struct aw8691, cdev);
#endif
    unsigned int val = 0;
    int rc = 0;

    rc = kstrtouint(buf, 0, &val);
    if (rc < 0)
        return rc;

    pr_debug("%s: value=%x\n", __FUNCTION__, val);

    mutex_lock(&aw8691->lock);
    aw8691->seq = val;
    aw8691_haptic_set_que_seq(aw8691, aw8691->seq);
    mutex_unlock(&aw8691->lock);
    return count;
}

static ssize_t aw8691_loop_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
#ifdef TIMED_OUTPUT
    struct timed_output_dev *to_dev = dev_get_drvdata(dev);
    struct aw8691 *aw8691 = container_of(to_dev, struct aw8691, to_dev);
#else
    struct led_classdev *cdev = dev_get_drvdata(dev);
    struct aw8691 *aw8691 = container_of(cdev, struct aw8691, cdev);
#endif
    size_t count = 0;
    unsigned char i = 0;
    unsigned char reg_val = 0;
    unsigned char tmp[2];
    unsigned char loop_tmp = 0;

    aw8691->loop = 0;
    for(i=0; i<AW8691_SEQUENCER_LOOP_SIZE; i++) {
        aw8691_i2c_read(aw8691, AW8691_REG_SEQ_LOOP1+i, &reg_val);
        tmp[0] = reg_val>>4;
        tmp[1] = reg_val&0x0F;
        loop_tmp = (tmp[0]<<8)|tmp[1];
        count += snprintf(buf+count, PAGE_SIZE-count,
                "seq%d loop: 0x%02x\n", i*2+1, tmp[0]);
        count += snprintf(buf+count, PAGE_SIZE-count,
                "seq%d loop: 0x%02x\n", i*2+2, tmp[1]);
        aw8691->loop |= (loop_tmp<<((AW8691_SEQUENCER_LOOP_SIZE-i-1)*8));
    }
    return count;
}

static ssize_t aw8691_loop_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
#ifdef TIMED_OUTPUT
    struct timed_output_dev *to_dev = dev_get_drvdata(dev);
    struct aw8691 *aw8691 = container_of(to_dev, struct aw8691, to_dev);
#else
    struct led_classdev *cdev = dev_get_drvdata(dev);
    struct aw8691 *aw8691 = container_of(cdev, struct aw8691, cdev);
#endif
    unsigned int val = 0;
    int rc = 0;

    rc = kstrtouint(buf, 0, &val);
    if (rc < 0)
        return rc;

    pr_debug("%s: value=%x\n", __FUNCTION__, val);

    mutex_lock(&aw8691->lock);
    aw8691->loop = val;
    aw8691_haptic_set_que_loop(aw8691, aw8691->loop);
    mutex_unlock(&aw8691->lock);
    return count;
}

static ssize_t aw8691_reg_show(struct device *dev, struct device_attribute *attr,
        char *buf)
{
#ifdef TIMED_OUTPUT
    struct timed_output_dev *to_dev = dev_get_drvdata(dev);
    struct aw8691 *aw8691 = container_of(to_dev, struct aw8691, to_dev);
#else
    struct led_classdev *cdev = dev_get_drvdata(dev);
    struct aw8691 *aw8691 = container_of(cdev, struct aw8691, cdev);
#endif
    ssize_t len = 0;
    unsigned char i = 0;
    unsigned char reg_val = 0;
    for(i = 0; i < AW8691_REG_MAX; i ++) {
        if(!(aw8691_reg_access[i]&REG_RD_ACCESS))
           continue;
        aw8691_i2c_read(aw8691, i, &reg_val);
        len += snprintf(buf+len, PAGE_SIZE-len, "reg:0x%02x=0x%02x \n", i, reg_val);
    }
    return len;
}

static ssize_t aw8691_reg_store(struct device *dev, struct device_attribute *attr,
        const char *buf, size_t count)
{
#ifdef TIMED_OUTPUT
    struct timed_output_dev *to_dev = dev_get_drvdata(dev);
    struct aw8691 *aw8691 = container_of(to_dev, struct aw8691, to_dev);
#else
    struct led_classdev *cdev = dev_get_drvdata(dev);
    struct aw8691 *aw8691 = container_of(cdev, struct aw8691, cdev);
#endif
    unsigned int databuf[2] = {0, 0};

    if(2 == sscanf(buf, "%x %x", &databuf[0], &databuf[1])) {
        aw8691_i2c_write(aw8691, (unsigned char)databuf[0], (unsigned char)databuf[1]);
    }

    return count;
}

static ssize_t aw8691_rtp_show(struct device *dev, struct device_attribute *attr,
        char *buf)
{
#ifdef TIMED_OUTPUT
    //struct timed_output_dev *to_dev = dev_get_drvdata(dev);
    //struct aw8691 *aw8691 = container_of(to_dev, struct aw8691, to_dev);
#else
    //struct led_classdev *cdev = dev_get_drvdata(dev);
    //struct aw8691 *aw8691 = container_of(cdev, struct aw8691, cdev);
#endif
    ssize_t len = 0;
    len += snprintf(buf+len, PAGE_SIZE-len, "rtp mode\n");
    return len;
}

static ssize_t aw8691_rtp_store(struct device *dev, struct device_attribute *attr,
        const char *buf, size_t count)
{
#ifdef TIMED_OUTPUT
    struct timed_output_dev *to_dev = dev_get_drvdata(dev);
    struct aw8691 *aw8691 = container_of(to_dev, struct aw8691, to_dev);
#else
    struct led_classdev *cdev = dev_get_drvdata(dev);
    struct aw8691 *aw8691 = container_of(cdev, struct aw8691, cdev);
#endif
    unsigned int val = 0;
    int rc = 0;

    rc = kstrtouint(buf, 0, &val);
    if (rc < 0)
        return rc;

    aw8691_rtp_play(aw8691, val);

    return count;
}

static ssize_t aw8691_ram_update_show(struct device *dev, struct device_attribute *attr,
        char *buf)
{
#ifdef TIMED_OUTPUT
    //struct timed_output_dev *to_dev = dev_get_drvdata(dev);
    //struct aw8691 *aw8691 = container_of(to_dev, struct aw8691, to_dev);
#else
    //struct led_classdev *cdev = dev_get_drvdata(dev);
    //struct aw8691 *aw8691 = container_of(cdev, struct aw8691, cdev);
#endif
    ssize_t len = 0;
    len += snprintf(buf+len, PAGE_SIZE-len, "ram/rtp mode\n");
    return len;
}

static ssize_t aw8691_ram_update_store(struct device *dev, struct device_attribute *attr,
        const char *buf, size_t count)
{
#ifdef TIMED_OUTPUT
    struct timed_output_dev *to_dev = dev_get_drvdata(dev);
    struct aw8691 *aw8691 = container_of(to_dev, struct aw8691, to_dev);
#else
    struct led_classdev *cdev = dev_get_drvdata(dev);
    struct aw8691 *aw8691 = container_of(cdev, struct aw8691, cdev);
#endif
    unsigned int val = 0;
    int rc = 0;

    rc = kstrtouint(buf, 0, &val);
    if (rc < 0)
        return rc;

    if(val) {
        aw8691_ram_update(aw8691);
    }
    return count;
}

#ifdef CONFIG_INPUT_AWINIC_HAPTIC
static ssize_t aw8691_pwk_p_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
#ifdef TIMED_OUTPUT
    struct timed_output_dev *to_dev = dev_get_drvdata(dev);
    struct aw8691 *aw8691 = container_of(to_dev, struct aw8691, to_dev);
#else
    struct led_classdev *cdev = dev_get_drvdata(dev);
    struct aw8691 *aw8691 = container_of(cdev, struct aw8691, cdev);
#endif
    unsigned char reg_val = 0;
    aw8691_i2c_read(aw8691, AW8691_REG_TRG1_WAV_P, &reg_val);

    return snprintf(buf, PAGE_SIZE, "%d\n", reg_val);
}

static ssize_t aw8691_pwk_p_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
#ifdef TIMED_OUTPUT
    struct timed_output_dev *to_dev = dev_get_drvdata(dev);
    struct aw8691 *aw8691 = container_of(to_dev, struct aw8691, to_dev);
#else
    struct led_classdev *cdev = dev_get_drvdata(dev);
    struct aw8691 *aw8691 = container_of(cdev, struct aw8691, cdev);
#endif
    unsigned int databuf = 0;
    if(1 == sscanf(buf, "%x", &databuf)) {
        aw8691_i2c_write(aw8691, AW8691_REG_TRG1_WAV_P, (unsigned char)databuf);
    }

    return count;
}

static ssize_t aw8691_pwk_n_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
#ifdef TIMED_OUTPUT
    struct timed_output_dev *to_dev = dev_get_drvdata(dev);
    struct aw8691 *aw8691 = container_of(to_dev, struct aw8691, to_dev);
#else
    struct led_classdev *cdev = dev_get_drvdata(dev);
    struct aw8691 *aw8691 = container_of(cdev, struct aw8691, cdev);
#endif
    unsigned char reg_val = 0;
    aw8691_i2c_read(aw8691, AW8691_REG_TRG1_WAV_N, &reg_val);

    return snprintf(buf, PAGE_SIZE, "%d\n", reg_val);
}

static ssize_t aw8691_pwk_n_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
#ifdef TIMED_OUTPUT
    struct timed_output_dev *to_dev = dev_get_drvdata(dev);
    struct aw8691 *aw8691 = container_of(to_dev, struct aw8691, to_dev);
#else
    struct led_classdev *cdev = dev_get_drvdata(dev);
    struct aw8691 *aw8691 = container_of(cdev, struct aw8691, cdev);
#endif
    unsigned int databuf = 0;
    if(1 == sscanf(buf, "%x", &databuf)) {
        aw8691_i2c_write(aw8691, AW8691_REG_TRG1_WAV_N, (unsigned char)databuf);
    }

    return count;
}

static ssize_t aw8691_voldown_p_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
#ifdef TIMED_OUTPUT
    struct timed_output_dev *to_dev = dev_get_drvdata(dev);
    struct aw8691 *aw8691 = container_of(to_dev, struct aw8691, to_dev);
#else
    struct led_classdev *cdev = dev_get_drvdata(dev);
    struct aw8691 *aw8691 = container_of(cdev, struct aw8691, cdev);
#endif
    unsigned char reg_val = 0;
    aw8691_i2c_read(aw8691, AW8691_REG_TRG2_WAV_P, &reg_val);

    return snprintf(buf, PAGE_SIZE, "%d\n", reg_val);
}

static ssize_t aw8691_voldown_p_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
#ifdef TIMED_OUTPUT
    struct timed_output_dev *to_dev = dev_get_drvdata(dev);
    struct aw8691 *aw8691 = container_of(to_dev, struct aw8691, to_dev);
#else
    struct led_classdev *cdev = dev_get_drvdata(dev);
    struct aw8691 *aw8691 = container_of(cdev, struct aw8691, cdev);
#endif
    unsigned int databuf = 0;
    if(1 == sscanf(buf, "%x", &databuf)) {
        aw8691_i2c_write(aw8691, AW8691_REG_TRG2_WAV_P, (unsigned char)databuf);
    }

    return count;
}

static ssize_t aw8691_voldown_n_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
#ifdef TIMED_OUTPUT
    struct timed_output_dev *to_dev = dev_get_drvdata(dev);
    struct aw8691 *aw8691 = container_of(to_dev, struct aw8691, to_dev);
#else
    struct led_classdev *cdev = dev_get_drvdata(dev);
    struct aw8691 *aw8691 = container_of(cdev, struct aw8691, cdev);
#endif
    unsigned char reg_val = 0;
    aw8691_i2c_read(aw8691, AW8691_REG_TRG2_WAV_N, &reg_val);

    return snprintf(buf, PAGE_SIZE, "%d\n", reg_val);
}

static ssize_t aw8691_voldown_n_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
#ifdef TIMED_OUTPUT
    struct timed_output_dev *to_dev = dev_get_drvdata(dev);
    struct aw8691 *aw8691 = container_of(to_dev, struct aw8691, to_dev);
#else
    struct led_classdev *cdev = dev_get_drvdata(dev);
    struct aw8691 *aw8691 = container_of(cdev, struct aw8691, cdev);
#endif
    unsigned int databuf = 0;
    if(1 == sscanf(buf, "%x", &databuf)) {
        aw8691_i2c_write(aw8691, AW8691_REG_TRG2_WAV_N, (unsigned char)databuf);
    }

    return count;
}
static ssize_t aw8691_volup_p_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
#ifdef TIMED_OUTPUT
    struct timed_output_dev *to_dev = dev_get_drvdata(dev);
    struct aw8691 *aw8691 = container_of(to_dev, struct aw8691, to_dev);
#else
    struct led_classdev *cdev = dev_get_drvdata(dev);
    struct aw8691 *aw8691 = container_of(cdev, struct aw8691, cdev);
#endif
    unsigned char reg_val = 0;
    aw8691_i2c_read(aw8691, AW8691_REG_TRG3_WAV_P, &reg_val);

    return snprintf(buf, PAGE_SIZE, "%d\n", reg_val);
}

static ssize_t aw8691_volup_p_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
#ifdef TIMED_OUTPUT
    struct timed_output_dev *to_dev = dev_get_drvdata(dev);
    struct aw8691 *aw8691 = container_of(to_dev, struct aw8691, to_dev);
#else
    struct led_classdev *cdev = dev_get_drvdata(dev);
    struct aw8691 *aw8691 = container_of(cdev, struct aw8691, cdev);
#endif
    unsigned int databuf = 0;
    if(1 == sscanf(buf, "%x", &databuf)) {
        aw8691_i2c_write(aw8691, AW8691_REG_TRG3_WAV_P, (unsigned char)databuf);
    }

    return count;
}

static ssize_t aw8691_volup_n_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
#ifdef TIMED_OUTPUT
    struct timed_output_dev *to_dev = dev_get_drvdata(dev);
    struct aw8691 *aw8691 = container_of(to_dev, struct aw8691, to_dev);
#else
    struct led_classdev *cdev = dev_get_drvdata(dev);
    struct aw8691 *aw8691 = container_of(cdev, struct aw8691, cdev);
#endif
    unsigned char reg_val = 0;
    aw8691_i2c_read(aw8691, AW8691_REG_TRG3_WAV_N, &reg_val);

    return snprintf(buf, PAGE_SIZE, "%d\n", reg_val);
}

static ssize_t aw8691_volup_n_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
#ifdef TIMED_OUTPUT
    struct timed_output_dev *to_dev = dev_get_drvdata(dev);
    struct aw8691 *aw8691 = container_of(to_dev, struct aw8691, to_dev);
#else
    struct led_classdev *cdev = dev_get_drvdata(dev);
    struct aw8691 *aw8691 = container_of(cdev, struct aw8691, cdev);
#endif
    unsigned int databuf = 0;
    if(1 == sscanf(buf, "%x", &databuf)) {
        aw8691_i2c_write(aw8691, AW8691_REG_TRG3_WAV_N, (unsigned char)databuf);
    }

    return count;
}
static ssize_t aw8691_reset_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
#ifdef TIMED_OUTPUT
    struct timed_output_dev *to_dev = dev_get_drvdata(dev);
    struct aw8691 *aw8691 = container_of(to_dev, struct aw8691, to_dev);
#else
    struct led_classdev *cdev = dev_get_drvdata(dev);
    struct aw8691 *aw8691 = container_of(cdev, struct aw8691, cdev);
#endif
    unsigned char reg_val = 0;
    aw8691_i2c_read(aw8691, AW8691_REG_ID, &reg_val);

    return snprintf(buf, PAGE_SIZE, "%d\n", reg_val);
}

static ssize_t aw8691_reset_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
#ifdef TIMED_OUTPUT
    struct timed_output_dev *to_dev = dev_get_drvdata(dev);
    struct aw8691 *aw8691 = container_of(to_dev, struct aw8691, to_dev);
#else
    struct led_classdev *cdev = dev_get_drvdata(dev);
    struct aw8691 *aw8691 = container_of(cdev, struct aw8691, cdev);
#endif
    unsigned int val = 0;
    int rc = 0;

    rc = kstrtouint(buf, 0, &val);
    if (rc < 0)
        return rc;

    pr_debug("%s: value=%d\n", __FUNCTION__, val);

    if(1 == val) {
        aw8691_i2c_write(aw8691, AW8691_REG_ID, 0xAA);
    }

    return count;
}
#endif

static DEVICE_ATTR(extra, S_IWUSR | S_IRUGO, aw8691_extra_show, aw8691_extra_store);
static DEVICE_ATTR(state, S_IWUSR | S_IRUGO, aw8691_state_show, aw8691_state_store);
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
#ifdef CONFIG_INPUT_AWINIC_HAPTIC
static DEVICE_ATTR(pwk_p, S_IWUSR | S_IRUGO, aw8691_pwk_p_show, aw8691_pwk_p_store);/* Power key for trig1(0x0d) */
static DEVICE_ATTR(voldown_p, S_IWUSR | S_IRUGO, aw8691_voldown_p_show, aw8691_voldown_p_store);/* Vol down for trig2(0x0e) */
static DEVICE_ATTR(volup_p, S_IWUSR | S_IRUGO, aw8691_volup_p_show, aw8691_volup_p_store);/* Vol up key for trig3(0x0f) */
static DEVICE_ATTR(pwk_n, S_IWUSR | S_IRUGO, aw8691_pwk_n_show, aw8691_pwk_n_store);/* Power key for trig1_n(0x10) */
static DEVICE_ATTR(voldown_n, S_IWUSR | S_IRUGO, aw8691_voldown_n_show, aw8691_voldown_n_store);/* Vol down for trig2_n(0x11) */
static DEVICE_ATTR(volup_n, S_IWUSR | S_IRUGO, aw8691_volup_n_show, aw8691_volup_n_store);/* Vol up key for trig3_n(0x12) */
static DEVICE_ATTR(reset, S_IWUSR | S_IRUGO, aw8691_reset_show, aw8691_reset_store);/* Reset device */
#endif

static struct attribute *aw8691_vibrator_attributes[] = {
    &dev_attr_extra.attr,
    &dev_attr_state.attr,
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
#ifdef CONFIG_INPUT_AWINIC_HAPTIC
    &dev_attr_pwk_p.attr,
    &dev_attr_voldown_p.attr,
    &dev_attr_volup_p.attr,
    &dev_attr_reset.attr,
    &dev_attr_pwk_n.attr,
    &dev_attr_voldown_n.attr,
    &dev_attr_volup_n.attr,
#endif
    NULL
};

static struct attribute_group aw8691_vibrator_attribute_group = {
    .attrs = aw8691_vibrator_attributes
};

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
    
    if(aw8691->state) {
        //aw8691_haptic_set_repeat_que_seq(aw8691, aw8691->index);
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
    for(i=0; i<AW8691_SEQUENCER_SIZE; i++) {
        ret = aw8691_i2c_read(aw8691, AW8691_REG_QUE_SEQ1+i, &reg_val);
        aw8691->seq |= (reg_val<<((AW8691_SEQUENCER_SIZE-i-1)*8));
    }
#ifdef TIMED_OUTPUT
    aw8691->to_dev.name = "vibrator";
    aw8691->to_dev.get_time = aw8691_vibrator_get_time;
    aw8691->to_dev.enable = aw8691_vibrator_enable;

    ret = timed_output_dev_register(&(aw8691->to_dev));
    if ( ret < 0){
        dev_err(aw8691->dev, "%s: fail to create timed output dev\n",
                __func__);
        return ret;
    }
    ret = sysfs_create_group(&aw8691->to_dev.dev->kobj, &aw8691_vibrator_attribute_group);
    if (ret < 0) {
        dev_err(aw8691->dev, "%s error creating sysfs attr files\n", __func__);
        return ret;
    }
#else
    aw8691->cdev.name = "vibrator";
    aw8691->cdev.brightness_get = aw8691_haptic_brightness_get;
    aw8691->cdev.brightness_set = aw8691_haptic_brightness_set;
    
    ret = devm_led_classdev_register(&aw8691->i2c->dev, &aw8691->cdev);
    if (ret < 0){
        dev_err(aw8691->dev, "%s: fail to create led dev\n",
                __func__);
        return ret;
    }
    ret = sysfs_create_group(&aw8691->cdev.dev->kobj, &aw8691_vibrator_attribute_group);
    if (ret < 0) {
        dev_err(aw8691->dev, "%s error creating sysfs attr files\n", __func__);
        return ret;
     }
#endif
    hrtimer_init(&aw8691->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
    aw8691->timer.function = aw8691_vibrator_timer_func;
    INIT_WORK(&aw8691->vibrator_work, aw8691_vibrator_work_routine);

    INIT_WORK(&aw8691->rtp_work, aw8691_rtp_work_routine);

    aw8691->ws = wakeup_source_register("vibrator");
    if (!aw8691->ws)
        return -ENOMEM;

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
/*
    aw8691_i2c_write_bits(aw8691, AW8691_REG_SYSINTM,
            AW8691_BIT_SYSINTM_UVLO_MASK, AW8691_BIT_SYSINTM_UVLO_EN);
    aw8691_i2c_write_bits(aw8691, AW8691_REG_SYSINTM,
            AW8691_BIT_SYSINTM_OCD_MASK, AW8691_BIT_SYSINTM_OCD_EN);
    aw8691_i2c_write_bits(aw8691, AW8691_REG_SYSINTM,
            AW8691_BIT_SYSINTM_OT_MASK, AW8691_BIT_SYSINTM_OT_EN);
*/
}

static irqreturn_t aw8691_irq(int irq, void *data)
{
    struct aw8691 *aw8691 = data;
    unsigned char reg_val = 0;
    unsigned int buf_len = 0;

    pr_debug("%s enter\n", __func__);

    aw8691_i2c_read(aw8691, AW8691_REG_SYSINT, &reg_val);
    pr_debug("%s: reg SYSINT=0x%x\n", __func__, reg_val);

    if(reg_val & AW8691_BIT_SYSINT_UVLI) {
        pr_err("%s chip uvlo int error\n", __func__);
    }
    if(reg_val & AW8691_BIT_SYSINT_OCDI) {
        pr_err("%s chip over current int error\n", __func__);
    }
    if(reg_val & AW8691_BIT_SYSINT_OTI) {
        pr_err("%s chip over temperature int error\n", __func__);
    }
    if(reg_val & AW8691_BIT_SYSINT_DONEI) {
        pr_debug("%s chip playback done\n", __func__);
    }

    if(reg_val & AW8691_BIT_SYSINT_FF_AEI) {
        pr_debug("%s: aw8691 rtp fifo almost empty int\n", __func__);
        if(aw8691->rtp_init) {
            while((!aw8691_haptic_rtp_get_fifo_afi(aw8691)) && 
                    (aw8691->play_mode == AW8691_HAPTIC_RTP_MODE)) {
                pr_debug("%s: aw8691 rtp mode fifo update, cnt=%d\n",
                        __func__, aw8691->rtp_cnt);
                if((aw8691_rtp->len-aw8691->rtp_cnt) < (aw8691->ram.base_addr>>3)) {
                    buf_len = aw8691_rtp->len-aw8691->rtp_cnt;
                } else {
                    buf_len = (aw8691->ram.base_addr>>3);
                }
                aw8691_i2c_writes(aw8691, AW8691_REG_RTP_DATA,
                        &aw8691_rtp->data[aw8691->rtp_cnt], buf_len);
                aw8691->rtp_cnt += buf_len;
                if(aw8691->rtp_cnt == aw8691_rtp->len) {
                    pr_debug("%s: rtp update complete\n", __func__);
                    aw8691_haptic_set_rtp_aei(aw8691, false);
                    aw8691->rtp_cnt = 0;
                    aw8691->rtp_init = 0;
#ifdef AW8691_HAPTIC_VBAT_MONITOR
                    aw8691_haptic_set_bst_mode(aw8691, AW8691_HAPTIC_BOOST_MODE);
#endif
#ifdef AW8691_REPEAT_RTP_PLAYING
                    aw8691_rtp_play(aw8691, aw8691->rtp_file_num);
#endif
                    break;
                }
            }
        } else {
            pr_err("%s: aw8691 rtp init = %d, init error\n", __func__, aw8691->rtp_init);
        }
    }

    if(reg_val & AW8691_BIT_SYSINT_FF_AFI) {
        pr_debug("%s: aw8691 rtp mode fifo full empty\n", __func__);
    }

    if(aw8691->play_mode != AW8691_HAPTIC_RTP_MODE) {
        aw8691_haptic_set_rtp_aei(aw8691, false);
    }

    aw8691_i2c_read(aw8691, AW8691_REG_SYSINT, &reg_val);
    pr_debug("%s: reg SYSINT=0x%x\n", __func__, reg_val);
    aw8691_i2c_read(aw8691, AW8691_REG_SYSST, &reg_val);
    pr_debug("%s: reg SYSST=0x%x\n", __func__, reg_val);

    pr_debug("%s exit\n", __func__);

    return IRQ_HANDLED;
}

/*****************************************************
 *
 * device tree
 *
 *****************************************************/
static int aw8691_parse_dt(struct device *dev, struct aw8691 *aw8691,
        struct device_node *np) {
    aw8691->reset_gpio = of_get_named_gpio(np, "reset-gpio", 0);
    if (aw8691->reset_gpio < 0) {
        dev_err(dev, "%s: no reset gpio provided, will not HW reset device\n", __func__);
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

    aw8691->haptic_context_gpio = of_get_named_gpio(np, "haptic-context-gpio", 0);
    if (aw8691->haptic_context_gpio < 0) {
        dev_err(dev, "%s: no haptic context gpio provided.\n", __func__);
    } else {
        dev_info(dev, "%s: haptic context gpio provided ok.\n", __func__);
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
        dev_err(aw8691->dev, "%s:  failed\n", __func__);
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

    while(cnt < AW_READ_CHIPID_RETRIES) {
        /* hardware reset */
        aw8691_hw_reset(aw8691);

        ret = aw8691_i2c_read(aw8691, AW8691_REG_ID, &reg);
        if (ret < 0) {
            dev_err(aw8691->dev, "%s: failed to read register AW8691_REG_ID: %d\n", __func__, ret);
        }
        switch (reg) {
        case 0x91:
            pr_info("%s aw8691 detected\n", __func__);
            aw8691->chipid = AW8691_ID;
            //aw8691->flags |= AW8691_FLAG_SKIP_INTERRUPTS;
            return 0;
        default:
            pr_info("%s unsupported device revision (0x%x)\n", __func__, reg );
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
static ssize_t aw8691_i2c_reg_store(struct device *dev, struct device_attribute *attr,
                const char *buf, size_t count)
{
    struct aw8691 *aw8691 = dev_get_drvdata(dev);

    unsigned int databuf[2] = {0, 0};

    if(2 == sscanf(buf, "%x %x", &databuf[0], &databuf[1])) {
        aw8691_i2c_write(aw8691, (unsigned char)databuf[0], (unsigned char)databuf[1]);
    }

    return count;
}

static ssize_t aw8691_i2c_reg_show(struct device *dev, struct device_attribute *attr,
                char *buf)
{
    struct aw8691 *aw8691 = dev_get_drvdata(dev);
    ssize_t len = 0;
    unsigned char i = 0;
    unsigned char reg_val = 0;
    for(i = 0; i < AW8691_REG_MAX; i ++) {
        if(!(aw8691_reg_access[i]&REG_RD_ACCESS))
           continue;
        aw8691_i2c_read(aw8691, i, &reg_val);
        len += snprintf(buf+len, PAGE_SIZE-len, "reg:0x%02x=0x%02x \n", i, reg_val);
    }
    return len;
}
static ssize_t aw8691_i2c_ram_store(struct device *dev, struct device_attribute *attr,
                const char *buf, size_t count)
{
    struct aw8691 *aw8691 = dev_get_drvdata(dev);

    unsigned int databuf[1] = {0};

    if(1 == sscanf(buf, "%x", &databuf[0])) {
        if(1 == databuf[0]) {
            aw8691_ram_update(aw8691);
        }
    }

    return count;
}

static ssize_t aw8691_i2c_ram_show(struct device *dev, struct device_attribute *attr,
                char *buf)
{
    struct aw8691 *aw8691 = dev_get_drvdata(dev);
    ssize_t len = 0;
    unsigned int i = 0;
    unsigned char reg_val = 0;

    aw8691_haptic_stop(aw8691);
    /* RAMINIT Enable */
    aw8691_i2c_write_bits(aw8691, AW8691_REG_SYSCTRL,
            AW8691_BIT_SYSCTRL_RAMINIT_MASK, AW8691_BIT_SYSCTRL_RAMINIT_EN);

    aw8691_i2c_write(aw8691, AW8691_REG_RAMADDRH, (unsigned char)(aw8691->ram.base_addr>>8));
    aw8691_i2c_write(aw8691, AW8691_REG_RAMADDRL, (unsigned char)(aw8691->ram.base_addr&0x00ff));
    len += snprintf(buf+len, PAGE_SIZE-len, "aw8691_haptic_ram:\n");
    for(i=0; i<aw8691->ram.len; i++) {
        aw8691_i2c_read(aw8691, AW8691_REG_RAMDATA, &reg_val);
        len += snprintf(buf+len, PAGE_SIZE-len, "0x%02x,", reg_val);
    }
    len += snprintf(buf+len, PAGE_SIZE-len, "\n");
    /* RAMINIT Disable */
    aw8691_i2c_write_bits(aw8691, AW8691_REG_SYSCTRL,
            AW8691_BIT_SYSCTRL_RAMINIT_MASK, AW8691_BIT_SYSCTRL_RAMINIT_OFF);

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


static bool mmi_factory_check(void)
{
	struct device_node *np = of_find_node_by_path("/chosen");
	bool factory = false;

	if (np)
		factory = of_property_read_bool(np, "mmi,factory-cable");

	of_node_put(np);

	return factory;
}

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

    /* aw8691 regmap */
    aw8691->regmap = devm_regmap_init_i2c(i2c, &aw8691_regmap);
    if (IS_ERR(aw8691->regmap)) {
        ret = PTR_ERR(aw8691->regmap);
        dev_err(&i2c->dev, "%s: failed to allocate register map: %d\n", __func__, ret);
        goto err;
    }

    i2c_set_clientdata(i2c, aw8691);

    /* aw8691 rst & int */
    if (np) {
        ret = aw8691_parse_dt(&i2c->dev, aw8691, np);
        if (ret) {
            dev_err(&i2c->dev, "%s: failed to parse device tree node\n", __func__);
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

    aw8691->factory_mode = mmi_factory_check();

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

    wakeup_source_unregister(aw8691->ws);

    return 0;
}

static const struct i2c_device_id aw8691_i2c_id[] = {
    { AW8691_I2C_NAME, 0 },
    { }
};
MODULE_DEVICE_TABLE(i2c, aw8691_i2c_id);

static struct of_device_id aw8691_dt_match[] = {
    { .compatible = "awinic,aw8691_haptic" },
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
    if(ret){
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
