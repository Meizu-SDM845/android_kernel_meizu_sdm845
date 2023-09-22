#ifndef _AW8691_H_
#define _AW8691_H_

/*********************************************************
 *
 * kernel 3.18
 *
 ********************************************************/
#if LINUX_VERSION_CODE <= KERNEL_VERSION(4, 4, 1)
#define TIMED_OUTPUT
#endif

/*********************************************************
 *
 * aw8691.h
 *
 ********************************************************/
#include <linux/regmap.h>
#include <linux/timer.h>
#include <linux/workqueue.h>
#include <linux/hrtimer.h>
#include <linux/mutex.h>
#include <linux/cdev.h>
#include <linux/pm_wakeup.h>
#ifdef TIMED_OUTPUT
#include <../../../../drivers/staging/android/timed_output.h>
#else
#include <linux/leds.h>
#endif

/*********************************************************
 *
 * marco
 *
 ********************************************************/
#define MAX_I2C_BUFFER_SIZE			65536

#define AW8691_REG_MAX				0xff

#define AW8691_SEQUENCER_SIZE			4
#define AW8691_SEQUENCER_LOOP_SIZE		2

#define AW8691_RTP_I2C_SINGLE_MAX_NUM		512

#define HAPTIC_MAX_TIMEOUT			10000

//#define AW8691_HAPTIC_VBAT_MONITOR

#ifdef AW8691_HAPTIC_VBAT_MONITOR
//#define AWINIC_GET_BATTERY_READ_NODE
#define SYS_BAT_DEV "/sys/class/power_supply/battery/voltage_now"
#define AW8691_SYS_VBAT_REFERENCE	4200000
#define AW8691_SYS_VBAT_MIN		3000000
#define AW8691_SYS_VBAT_MAX		4500000
#endif

enum aw8691_flags {
	AW8691_FLAG_NONR = 0,
	AW8691_FLAG_SKIP_INTERRUPTS = 1,
};

enum aw8691_chipids {
	AW8691_ID = 0,
};

enum aw8691_haptic_read_write {
	AW8691_HAPTIC_CMD_READ_REG = 0,
	AW8691_HAPTIC_CMD_WRITE_REG = 1,
	AW8691_HAPTIC_CMD_UPDATE_FIRMWARE = 2,
	AW8691_HAPTIC_CMD_READ_FIRMWARE = 3,
};

enum aw8691_haptic_work_mode {
	AW8691_HAPTIC_STANDBY_MODE = 0,
	AW8691_HAPTIC_RAM_MODE = 1,
	AW8691_HAPTIC_RTP_MODE = 2,
	AW8691_HAPTIC_TRIG_MODE = 3,
};

enum aw8691_haptic_bst_mode {
	AW8691_HAPTIC_BYPASS_MODE = 0,
	AW8691_HAPTIC_BOOST_MODE = 1,
};

enum aw8691_haptic_mode{
	HAPTIC_NONE	= 0x00,
	HAPTIC_SHORT	= 0x01,
	HAPTIC_LONG	= 0x02,
	HAPTIC_RTP	= 0x03,
};

/*********************************************************
 *
 * struct
 *
 ********************************************************/
struct fileops {
	unsigned char cmd;
	unsigned char reg;
	unsigned char ram_addrh;
	unsigned char ram_addrl;
};

struct ram {
	unsigned int len;
	unsigned int check_sum;
	unsigned int base_addr;
	unsigned char version;
	unsigned char ram_shift;
	unsigned char baseaddr_shift;
};

struct aw8691 {
	struct regmap *regmap;
	struct i2c_client *i2c;
	struct device *dev;
	struct input_dev *input;

	struct wakeup_source *ws;
	struct mutex lock;
	struct hrtimer timer;
	struct work_struct vibrator_work;
	struct work_struct rtp_work;
#ifdef TIMED_OUTPUT
	struct timed_output_dev to_dev;
#else
	struct led_classdev cdev;
#endif

	int reset_gpio;
	int irq_gpio;
	int haptic_context_gpio;
	int state;
	int duration;
	int amplitude;
	int index;
	int vmax;
	int gain;
	int gain_debug;
	int seq;
	int loop;

	enum aw8691_haptic_mode  haptic_mode;
	bool factory_mode;
	bool debugfs_debug;
	unsigned char hwen_flag;
	unsigned char flags;
	unsigned char chipid;

	unsigned int rtp_cnt;
	unsigned int rtp_file_num;

	unsigned char rtp_init;
	unsigned char ram_init;

	unsigned char play_mode;
	struct fileops fileops;
	struct ram ram;

	struct hrtimer ram_timer;
	struct work_struct ram_work;
};

struct aw8691_container{
	int len;
	unsigned char data[];
};


/*********************************************************
 *
 * ioctl
 *
 ********************************************************/
struct aw8691_seq_loop {
	unsigned char loop[AW8691_SEQUENCER_SIZE];
};

struct aw8691_que_seq {
	unsigned char index[AW8691_SEQUENCER_SIZE];
};


#define AW8691_HAPTIC_IOCTL_MAGIC	'h'

#define AW8691_HAPTIC_SET_QUE_SEQ	_IOWR(AW8691_HAPTIC_IOCTL_MAGIC, 1, struct aw8691_que_seq*)
#define AW8691_HAPTIC_SET_SEQ_LOOP	_IOWR(AW8691_HAPTIC_IOCTL_MAGIC, 2, struct aw8691_seq_loop*)
#define AW8691_HAPTIC_PLAY_QUE_SEQ	_IOWR(AW8691_HAPTIC_IOCTL_MAGIC, 3, unsigned int)
#define AW8691_HAPTIC_SET_BST_VOL	_IOWR(AW8691_HAPTIC_IOCTL_MAGIC, 4, unsigned int)
#define AW8691_HAPTIC_SET_BST_PEAK_CUR	_IOWR(AW8691_HAPTIC_IOCTL_MAGIC, 5, unsigned int)
#define AW8691_HAPTIC_SET_GAIN		_IOWR(AW8691_HAPTIC_IOCTL_MAGIC, 6, unsigned int)
#define AW8691_HAPTIC_PLAY_REPEAT_SEQ	_IOWR(AW8691_HAPTIC_IOCTL_MAGIC, 7, unsigned int)


#endif
