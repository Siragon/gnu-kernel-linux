/* drivers/input/touchscreen/goodix_touch.c
 *
 * Copyright (C) 2010 - 2011 Goodix, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.Any problem,please contact andrew@goodix.com.
 *
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/time.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/earlysuspend.h>
#include <linux/hrtimer.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <mach/gpio.h>
#include <linux/irq.h>
#include <linux/syscalls.h>
#include <linux/reboot.h>
#include <linux/proc_fs.h>
#include <linux/i2c/goodix_touch.h>
#include <linux/kthread.h>

#include <linux/vmalloc.h>
#include <linux/fs.h>
#include <linux/string.h>
#include <linux/completion.h>
#include <asm/uaccess.h>
#include <linux/regulator/consumer.h>
#include <../gpio-names.h>
/*************************TouchScreen Work Part******************************/

#define GOODIX_I2C_NAME "Goodix-TS"
/*define default resolution of the touchscreen*/
#define TOUCH_MAX_HEIGHT 	4096
#define TOUCH_MAX_WIDTH	4096

#define INT_PORT  	    	1	/*S3C64XX_GPN(15)          //Int IO port  S3C64XX_GPL(10)*/
/*whether need send cfg?*/
#define DRIVER_SEND_CFG
/*set trigger mode*/
#define INT_TRIGGER		1

#define POLL_TIME		10	/*actual query spacing interval:POLL_TIME+6*/

#define GOODIX_MULTI_TOUCH
#ifdef GOODIX_MULTI_TOUCH
#define MAX_FINGER_NUM	10
#else
#define MAX_FINGER_NUM	1
#endif

#define TOUCH_REGLATOR_ENABLE		1
#define TOUCH_REGLATOR_DISABLE	0
#define CONFIG_ADB_UPDATE

struct goodix_ts_data {
	uint16_t addr;
	uint8_t bad_data;
	struct i2c_client *client;
	struct input_dev *input_dev;
	int use_reset;		/*use RESET flag*/
	int use_irq;			/*use EINT flag*/
	int read_mode;		/*read moudle mode,20110221 by andrew*/
	struct hrtimer timer;
	struct work_struct work;
	char phys[32];
	int retry;
	struct early_suspend early_suspend;
	int (*power) (struct goodix_ts_data *ts, int on);
	uint16_t abs_x_max;
	uint16_t abs_y_max;
	uint8_t max_touch_num;
	uint8_t int_trigger_type;
	uint8_t green_wake_mode;
};

static const char *goodix_ts_name = "goodix_touch";
static struct workqueue_struct *goodix_wq;
struct i2c_client *i2c_connect_client;
static struct proc_dir_entry *goodix_proc_entry;
static struct kobject *goodix_debug_kobj;

#ifdef CONFIG_HAS_EARLYSUSPEND
static void goodix_ts_early_suspend(struct early_suspend *h);
static void goodix_ts_late_resume(struct early_suspend *h);
#endif

/*****************************End of Part I *********************************/
#ifdef CONFIG_TOUCHSCREEN_GOODIX_IAP
#define UPDATE_NEW_PROTOCOL

unsigned int oldcrc32 = 0xFFFFFFFF;
unsigned int crc32_table[256];
unsigned int ulPolynomial = 0x04c11db7;
unsigned char rd_cfg_addr;
unsigned char rd_cfg_len;
unsigned short read_addr;
unsigned short read_len;
unsigned char g_enter_isp;

static int goodix_update_write(struct file *filp, const char __user * buff,
			       unsigned long len, void *data);
static int goodix_update_read(char *page, char **start, off_t off, int count,
			      int *eof, void *data);

#define PACK_SIZE 					64	/*update file package size*/
#define MAX_TIMEOUT				60000	/*update time out conut*/
#define MAX_I2C_RETRIES				20	/*i2c retry times*/

/*I2C buf address*/
#define ADDR_CMD					80
#define ADDR_STA					81
#ifdef UPDATE_NEW_PROTOCOL
#define ADDR_DAT				0
#else
#define ADDR_DAT				82
#endif

/*moudle state*/
#define NEW_UPDATE_START			0x01
#define UPDATE_START				0x02
#define SLAVE_READY					0x08
#define UNKNOWN_ERROR				0x00
#define FRAME_ERROR				0x10
#define CHECKSUM_ERROR				0x20
#define TRANSLATE_ERROR			0x40
#define FLASH_ERROR					0X80

/*error no*/
#define ERROR_NO_FILE				2	/*ENOENT*/
#define ERROR_FILE_READ			23	/*ENFILE*/
#define ERROR_FILE_TYPE				21	/*EISDIR*/
#define ERROR_GPIO_REQUEST			4	/*EINTR*/
#define ERROR_I2C_TRANSFER			5	/*EIO*/
#define ERROR_NO_RESPONSE			16	/*EBUSY*/
#define ERROR_TIMEOUT				110	/*ETIMEDOUT*/

/*update steps*/
#define STEP_SET_PATH		1
#define STEP_CHECK_FILE		2
#define STEP_WRITE_SYN		3
#define STEP_WAIT_SYN		4
#define STEP_WRITE_LENGTH	5
#define STEP_WAIT_READY	6
#define STEP_WRITE_DATA	7
#define STEP_READ_STATUS	8
#define FUN_CLR_VAL			9
#define FUN_CMD				10
#define FUN_WRITE_CONFIG	11

/*fun cmd*/
#define CMD_DISABLE_TP             0
#define CMD_ENABLE_TP              1
#define CMD_READ_VER               2
#define CMD_READ_RAW               3
#define CMD_READ_DIF               4
#define CMD_READ_CFG               5
#define CMD_READ_CHIP_TYPE         6
#define CMD_NORMAL_MODE            7
/*#define CMD_READ_CFG               10*/

#define CMD_SYS_REBOOT             101

/*read mode*/
#define MODE_RD_VER                1
#define MODE_RD_RAW                2
#define MODE_RD_DIF                3
#define MODE_RD_CFG                4
#define MODE_RD_CHIP_TYPE          5

#endif

#ifdef CONFIG_ADB_UPDATE
static char *gt81x_fw;
static char *gt81x_config_info;
static char *fw_path;
static int show_len;
static int total_len;
int gt81x_update_fw(void *arg);
int gt81x_update_write_config(struct i2c_client *client, char *cfg_path);
#endif

/*************************Firmware Update part*****************************/
/*****************************End of Part II*******************************/
#define CREATE_WR_NODE
#define GUITAR_TEST_TOOL
#ifdef GUITAR_TEST_TOOL
unsigned int data_ready = DATA_NON_ACTIVE;

/*#define DEBUG*/
#ifdef DEBUG
int sum;
int access_count;
int int_count;
int access_int = -1;
#endif

#ifdef CREATE_WR_NODE
extern s32 init_wr_node(struct i2c_client *);
extern void uninit_wr_node(void);
#endif
#endif
/*****************************End of Part III*********************************/

char chip_type;
static struct regulator *ts_en;

/*******************************************************
Description:
	Read data from the i2c slave device;
	This operation consisted of 2 i2c_msgs,the first msg used
	to write the operate address,the second msg used to read data.

Parameter:
	client:	i2c device.
	buf[0]:operate address.
	buf[1]~buf[len]:read data buffer.
	len:operate length.

return:
	numbers of i2c_msgs to transfer
*********************************************************/
static int i2c_read_bytes(struct i2c_client *client, uint8_t * buf, int len)
{
	struct i2c_msg msgs[2];
	int ret = -1;

	msgs[0].flags = !I2C_M_RD;
	msgs[0].addr = client->addr;
	msgs[0].len = 1;
	msgs[0].buf = &buf[0];

	msgs[1].flags = I2C_M_RD;
	msgs[1].addr = client->addr;
	msgs[1].len = len - 1;
	msgs[1].buf = &buf[1];

	ret = i2c_transfer(client->adapter, msgs, 2);

	return ret;
}

/*******************************************************
Description:
	write data to the i2c slave device.

Parameter:
	client:	i2c device.
	buf[0]:operate address.
	buf[1]~buf[len]:write data buffer.
	len:operate length.

return:
	numbers of i2c_msgs to transfer.
*********************************************************/
static int i2c_write_bytes(struct i2c_client *client, uint8_t * data, int len)
{
	struct i2c_msg msg;
	int ret = -1;

	msg.flags = !I2C_M_RD;
	msg.addr = client->addr;
	msg.len = len;
	msg.buf = data;

	ret = i2c_transfer(client->adapter, &msg, 1);

	return ret;
}

inline void int_wakeup_green(struct goodix_ts_data *ts, uint8_t disable)
{
	if (!ts->green_wake_mode)
		return;

	if (disable)
		disable_irq(ts->client->irq);
	gpio_direction_output(irq_to_gpio(ts->client->irq), 0);
	msleep(5);
	gpio_direction_input(irq_to_gpio(ts->client->irq));
	if (disable)
		enable_irq(ts->client->irq);
}

/*******************************************************
Description:
	Goodix touchscreen initialize function.

Parameter:
	ts:	i2c client private struct.

return:
	Executive outcomes.0---succeed.
*******************************************************/
static int goodix_init_panel(struct goodix_ts_data *ts)
{
	int ret = -1;
	uint8_t rd_cfg_buf[7] = { 0x66, };

#ifdef DRIVER_SEND_CFG
	/*sample config info(puts your group1 config info here,if need send config info)*/
	uint8_t cfg_info_group1[] = {
		0x65, 0x02, 0x10, 0x00, 0x10, 0x00, 0x0A, 0x6D, 0x01, 0x00,
		0x0F, 0x28, 0x02, 0xf3, 0x90, 0x00, 0x00, 0x15, 0x00, 0x00,
		0x08, 0x00, 0x00, 0x44, 0x37, 0x00, 0x00, 0x00, 0x01, 0x02,
		0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0xFF,
		0xFF, 0xFF, 0xFF, 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06,
		0x07, 0x08, 0x09, 0x0A, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00,
		0x00, 0x32, 0x3C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00
	};
	uint8_t cfg_info_group2[] = {
		/*TODO puts your group2 config info here,if need.*/
	};
	uint8_t cfg_info_group3[] = {
		/*TODO puts your group3 config info here,if need.*/
	};
	uint8_t cfg_info_group4[] = {
		/*TODO puts your group4 config info here,if need.*/
	};
	uint8_t *send_cfg_buf[4] = {
		cfg_info_group1, cfg_info_group2, cfg_info_group3, cfg_info_group4
	};
	uint8_t cfg_info_len[4] = {
		sizeof(cfg_info_group1) / sizeof(cfg_info_group1[0]),
		sizeof(cfg_info_group2) / sizeof(cfg_info_group2[0]),
		sizeof(cfg_info_group3) / sizeof(cfg_info_group3[0]),
		sizeof(cfg_info_group4) / sizeof(cfg_info_group4[0])
	};
	printk("len1=%d,len2=%d,len3=%d,len4=%d\n", cfg_info_len[0],
	       cfg_info_len[1], cfg_info_len[2], cfg_info_len[3]);
	if ((!cfg_info_len[1]) && (!cfg_info_len[2]) && (!cfg_info_len[3])) {
		rd_cfg_buf[1] = 0x00;
		goto send_cfg;
	}
	rd_cfg_buf[0] = 0x52;	/*read SCREEN_FLAG(82:Bit1~Bit0)*/
	rd_cfg_buf[1] = 0x00;
	ret = i2c_read_bytes(ts->client, rd_cfg_buf, 2);
	if (ret <= 0) {
		printk("Read screen_flag failed,default use group1 config!\n");
		rd_cfg_buf[1] = 0x00;
	}
      send_cfg:
	rd_cfg_buf[1] &= 0x03;
	printk("Screen Type:%d\n", rd_cfg_buf[1]);
	ret =
	    i2c_write_bytes(ts->client, send_cfg_buf[rd_cfg_buf[1]],
			    cfg_info_len[rd_cfg_buf[1]] + 1);
	if (ret < 0)
		return ret;
#endif
	rd_cfg_buf[0] = 0x66;
	ret = i2c_read_bytes(ts->client, rd_cfg_buf, 7);
	if (ret <= 0) {
		dev_info(&ts->client->dev,
			 "Read resolution & max_touch_num failed, use default value!\n");
		ts->abs_x_max = TOUCH_MAX_HEIGHT;
		ts->abs_y_max = TOUCH_MAX_WIDTH;
		ts->max_touch_num = MAX_FINGER_NUM;
		ts->int_trigger_type = INT_TRIGGER;
		return 0;
	}
	ts->abs_x_max = (rd_cfg_buf[1] << 8) + rd_cfg_buf[2];
	ts->abs_y_max = (rd_cfg_buf[3] << 8) + rd_cfg_buf[4];
	ts->max_touch_num = rd_cfg_buf[5];
	ts->int_trigger_type = rd_cfg_buf[6] & 0x03;
	if ((!ts->abs_x_max) || (!ts->abs_y_max) || (!ts->max_touch_num)) {
		dev_info(&ts->client->dev,
			 "Read invalid resolution & max_touch_num, use default value!\n");
		ts->abs_x_max = TOUCH_MAX_HEIGHT;
		ts->abs_y_max = TOUCH_MAX_WIDTH;
		ts->max_touch_num = MAX_FINGER_NUM;
	}
	/*wake up mode from green mode*/
	rd_cfg_buf[0] = 0x6e;
	rd_cfg_buf[1] = 0x00;
	i2c_read_bytes(ts->client, rd_cfg_buf, 2);
	if ((rd_cfg_buf[1] & 0x0f) == 0x0f) {
		dev_info(&ts->client->dev,
			 "Touchscreen works in INT wake up green mode!\n");
		ts->green_wake_mode = 1;
	} else {
		dev_info(&ts->client->dev,
			 "Touchscreen works in IIC wake up green mode!\n");
		ts->green_wake_mode = 0;
	}

	msleep(10);
	return 0;

}

/*******************************************************
Description:
	Read goodix touchscreen version function.

Parameter:
	ts:	i2c client private struct.

return:
	Executive outcomes.0---succeed.
*******************************************************/
static int goodix_read_version(struct goodix_ts_data *ts, char **version)
{
	int ret = -1, count = 0;
	char *version_data;
	char *p;

	*version = (char *)vmalloc(18);
	version_data = *version;
	if (!version_data)
		return -ENOMEM;
	p = version_data;
	memset(version_data, 0, sizeof(version_data));
	version_data[0] = 240;
	int_wakeup_green(ts, 1);	/*wakeup green mode*/
	ret = i2c_read_bytes(ts->client, version_data, 17);
	if (ret <= 0)
		return ret;
	version_data[17] = '\0';

	if (*p == '\0')
		return 0;
	do {
		if ((*p > 122) || (*p < 48 && *p != 32) || (*p > 57 && *p < 65)
		    || (*p > 90 && *p < 97 && *p != '_'))	/*check illeqal character*/
			count++;
	} while (*++p != '\0');
	if (count > 2)
		return 0;
	else
		return 1;
}

/*******************************************************
Description:
	Goodix touchscreen work function.

Parameter:
	ts:	i2c client private struct.

return:
	Executive outcomes.0---succeed.
*******************************************************/
static void goodix_ts_work_func(struct work_struct *work)
{
	int ret = -1;
	int tmp = 0;
	uint8_t point_data[(1 - READ_COOR_ADDR) + 1 + 2 + 5 * MAX_FINGER_NUM + 1] = { 0 };	/*read address(1byte)+key index(1byte)+point mask(2bytes)+5bytes*MAX_FINGER_NUM+coor checksum(1byte)*/
	uint8_t check_sum = 0;
	uint16_t finger_current = 0;
	uint16_t finger_bit = 0;
	unsigned int count = 0, point_count = 0;
	unsigned int position = 0;
	uint8_t track_id[MAX_FINGER_NUM] = { 0 };
	unsigned int input_x = 0;
	unsigned int input_y = 0;
	unsigned int input_w = 0;
	unsigned char index = 0;
	unsigned char touch_num = 0;

	struct goodix_ts_data *ts =
	    container_of(work, struct goodix_ts_data, work);

#ifdef DEBUG
	printk("int count :%d\n", ++int_count);
	printk("ready?:%d\n", data_ready);
#endif
	if (DATA_ACTIVE == data_ready) {
		data_ready = DATA_READY;

#ifdef DEBUG
		/*access_int = int_count;*/
		printk("ready!\n");
		msleep(5);
#endif

		goto NO_ACTION;
	}

	if (g_enter_isp)
		return;
      COORDINATE_POLL:
	if ((ts->int_trigger_type > 1)
	    && (gpio_get_value(irq_to_gpio(ts->client->irq)) !=
		(ts->int_trigger_type & 0x01))) {
		goto NO_ACTION;
	}

	if (tmp > 9) {

		dev_info(&(ts->client->dev),
			 "I2C transfer error,touchscreen stop working.\n");
		goto XFER_ERROR;
	}

	if (ts->bad_data)
		msleep(20);

	point_data[0] = READ_COOR_ADDR;	/*read coor address*/
	ret = i2c_read_bytes(ts->client, point_data,
			     ((1 - READ_COOR_ADDR) + 1 + 2 +
			      5 * ts->max_touch_num + 1));
	if (ret <= 0) {
		dev_err(&(ts->client->dev), "I2C transfer error. Number:%d\n ",
			ret);
		ts->bad_data = 1;
		tmp++;
		ts->retry++;

		if (ts->int_trigger_type > 1)
			goto COORDINATE_POLL;
		else
			goto XFER_ERROR;
	}
	ts->bad_data = 0;
	finger_current =
	    (point_data[3 - READ_COOR_ADDR] << 8) + point_data[2 -
							       READ_COOR_ADDR];

	if (finger_current) {
		point_count = 0, finger_bit = finger_current;
		/*cal how many point touch currntly*/
		for (count = 0; (finger_bit != 0) && (count < ts->max_touch_num); count++) {
			if (finger_bit & 0x01) {
				track_id[point_count] = count;
				point_count++;
			}
			finger_bit >>= 1;
		}
		touch_num = point_count;

		check_sum = point_data[2 - READ_COOR_ADDR] + point_data[3 - READ_COOR_ADDR];	/*cal coor checksum*/
		count = 4 - READ_COOR_ADDR;
		for (point_count *= 5; point_count > 0; point_count--)
			check_sum += point_data[count++];
		check_sum += point_data[count];
		/*checksum verify error*/
		if (check_sum != 0) {
#if 0
			dev_info(&ts->client->dev, "Check_sum:%d,  Data:%d\n",
				 check_sum, point_data[count]);
			printk(KERN_INFO "Finger Bit:%d\n", finger_current);
			for (; count > 0; count--)
				printk(KERN_INFO "count=%d:%d  ", count,
				       point_data[count]);
			printk(KERN_INFO "\n");
#endif
			printk("coor checksum error!\n");
			if (ts->int_trigger_type > 1)
				goto COORDINATE_POLL;
			else
				goto XFER_ERROR;
		}
	}

	if (touch_num) {
		for (index = 0; index < touch_num; index++) {
			position = 4 - READ_COOR_ADDR + 5 * index;
			input_x =
			    (unsigned int)(point_data[position] << 8) +
			    (unsigned int)(point_data[position + 1]);
			input_y =
			    (unsigned int)(point_data[position + 2] << 8) +
			    (unsigned int)(point_data[position + 3]);
			input_w = (unsigned int)(point_data[position + 4]);
			/*input_x = input_x *SCREEN_MAX_HEIGHT/(TOUCH_MAX_HEIGHT);*/
			/*input_y = input_y *SCREEN_MAX_WIDTH/(TOUCH_MAX_WIDTH);*/

			if ((input_x > ts->abs_x_max)
			    || (input_y > ts->abs_y_max))
				continue;
			input_report_abs(ts->input_dev, ABS_MT_POSITION_X,
					 input_x);
			input_report_abs(ts->input_dev, ABS_MT_POSITION_Y,
					 input_y);
			input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR,
					 input_w);
			input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID,
					 track_id[index]);
			input_mt_sync(ts->input_dev);
		}
		if (touch_num == 1) {
			input_report_abs(ts->input_dev, ABS_X, input_x);
			input_report_abs(ts->input_dev, ABS_Y, input_y);
		}
	}
#ifdef HAVE_TOUCH_KEY
	/*printk(KERN_INFO"HAVE KEY DOWN!0x%x\n",point_data[1]);*/
	for (count = 0; count < MAX_KEY_NUM; count++) {
		input_report_key(ts->input_dev, touch_key_array[count],
				 !!(point_data[1] & (0x01 << count)));
	}
#endif

	input_report_key(ts->input_dev, BTN_TOUCH, touch_num > 0);

	input_sync(ts->input_dev);

	if (ts->int_trigger_type > 1) {
		msleep(POLL_TIME);
		goto COORDINATE_POLL;
	}
	goto END_WORK_FUNC;

      NO_ACTION:

#ifdef HAVE_TOUCH_KEY
	/*printk(KERN_INFO"HAVE KEY DOWN!0x%x\n",point_data[1]);*/
	for (count = 0; count < MAX_KEY_NUM; count++) {
		input_report_key(ts->input_dev, touch_key_array[count],
				 !!(point_data[1] & (0x01 << count)));
	}
	input_sync(ts->input_dev);
#endif
      END_WORK_FUNC:
      XFER_ERROR:
	return;
}

/*******************************************************
Description:
	Timer interrupt service routine.

Parameter:
	timer:	timer struct pointer.

return:
	Timer work mode. HRTIMER_NORESTART---not restart mode
*******************************************************/
static enum hrtimer_restart goodix_ts_timer_func(struct hrtimer *timer)
{
	struct goodix_ts_data *ts =
	    container_of(timer, struct goodix_ts_data, timer);
	queue_work(goodix_wq, &ts->work);
	hrtimer_start(&ts->timer, ktime_set(0, (POLL_TIME + 6) * 1000000),
		      HRTIMER_MODE_REL);
	return HRTIMER_NORESTART;
}

/*******************************************************
Description:
	External interrupt service routine.

Parameter:
	irq:	interrupt number.
	dev_id: private data pointer.

return:
	irq execute status.
*******************************************************/
static irqreturn_t goodix_ts_irq_handler(int irq, void *dev_id)
{
	struct goodix_ts_data *ts = dev_id;

	queue_work(goodix_wq, &ts->work);

	return IRQ_HANDLED;
}

/*******************************************************
Description:
	Goodix touchscreen power manage function.

Parameter:
	on:	power status.0---suspend;1---resume.

return:
	Executive outcomes.-1---i2c transfer error;0---succeed.
*******************************************************/
static int goodix_ts_power(struct goodix_ts_data *ts, int on)
{
	int ret = -1;
	unsigned char i2c_control_buf[2] = { 80, 1 };	/*suspend cmd*/
	if (on != 0 && on != 1) {
		printk(KERN_DEBUG "%s: Cant't support this command.",
		       goodix_ts_name);
		return -EINVAL;
	}

	if (ts != NULL && !ts->use_irq)
		return -2;

	/*suspend*/
	if (on == 0) {
		int_wakeup_green(ts, 1);

		ret = i2c_write_bytes(ts->client, i2c_control_buf, 2);
		if (ret > 0)
			ret = 0;
	/*resume*/
	} else if (on == 1) {
		gpio_direction_output(irq_to_gpio(ts->client->irq), 0);
		msleep(20);
		if (ts->use_irq) {
			enable_irq(ts->client->irq);
			gpio_direction_input(irq_to_gpio(ts->client->irq));	/*Set IO port as interrupt port*/
		} else
			gpio_direction_input(irq_to_gpio(ts->client->irq));
		ret = 0;
	}
	return ret;
}

/*******************************************************
Description:
	Goodix debug sysfs cat version function.

Parameter:
	standard sysfs show param.

return:
	Executive outcomes. 0---failed.
*******************************************************/
static ssize_t goodix_debug_version_show(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	int ret = 0;
	char *version_info = NULL;
	struct goodix_ts_data *ts;

	ts = i2c_get_clientdata(i2c_connect_client);
	if (ts == NULL)
		return 0;

	ret = goodix_read_version(ts, &version_info);
	if (ret <= 0) {
		printk(KERN_INFO "Read version data failed!\n");
		vfree(version_info);
		return 0;
	}

	sprintf(buf, "Goodix TouchScreen Version:%s\n", (version_info + 1));
	vfree(version_info);
	ret = strlen(buf);
	return ret;
}

/*******************************************************
Description:
	Goodix debug sysfs cat resolution function.

Parameter:
	standard sysfs show param.

return:
	Executive outcomes. 0---failed.
*******************************************************/
static ssize_t goodix_debug_resolution_show(struct device *dev,
					    struct device_attribute *attr,
					    char *buf)
{
	struct goodix_ts_data *ts;
	ts = i2c_get_clientdata(i2c_connect_client);

	sprintf(buf, "ABS_X_MAX = %d,ABS_Y_MAX = %d\n", ts->abs_x_max,
		ts->abs_y_max);

	return strlen(buf);
}

/*******************************************************
Description:
	Goodix debug sysfs cat version function.

Parameter:
	standard sysfs show param.

return:
	Executive outcomes. 0---failed.
*******************************************************/
static ssize_t goodix_debug_diffdata_show(struct device *dev,
					  struct device_attribute *attr,
					  char *buf)
{
	unsigned char *diff_data = (char *)vmalloc(4241);
	int ret = -1;
	char diff_data_cmd[2] = { 80, 202 };
	int i;
	int short_tmp;
	struct goodix_ts_data *ts = NULL;

	ts = i2c_get_clientdata(i2c_connect_client);
	disable_irq(ts->client->irq);
	int_wakeup_green(ts, 0);
	ret = i2c_write_bytes(ts->client, diff_data_cmd, 2);
	if (ret != 1) {
		dev_info(&ts->client->dev, "Write diff data cmd failed!\n");
		enable_irq(ts->client->irq);
		return 0;
	}

	while (gpio_get_value(irq_to_gpio(ts->client->irq)))
		msleep(1);
	diff_data[0] = 0x00;
	ret = i2c_read_bytes(ts->client, diff_data, sizeof(diff_data));
	if (ret <= 0) {
		dev_info(&ts->client->dev, "Read diff data failed!\n");
		vfree(diff_data);
		enable_irq(ts->client->irq);
		return 0;
	}
	for (i = 1; i < sizeof(diff_data); i += 2) {
		short_tmp = diff_data[i] + (diff_data[i + 1] << 8);
		if (short_tmp & 0x8000)
			short_tmp -= 65535;
		if (short_tmp == 512)
			continue;
		sprintf(buf + strlen(buf), " %d", short_tmp);
	}

	diff_data_cmd[1] = 0;
	ret = i2c_write_bytes(ts->client, diff_data_cmd, 2);
	if (ret != 1) {
		dev_info(&ts->client->dev, "Write diff data cmd failed!\n");
		enable_irq(ts->client->irq);
		vfree(diff_data);
		return 0;
	}
	vfree(diff_data);
	enable_irq(ts->client->irq);

	return strlen(buf);
}

/*******************************************************
Description:
	Goodix debug sysfs echo calibration function.

Parameter:
	standard sysfs store param.

return:
	Executive outcomes..
*******************************************************/
static ssize_t goodix_debug_calibration_store(struct device *dev,
					      struct device_attribute *attr,
					      const char *buf, size_t count)
{
	int ret = -1;
	char cal_cmd_buf[] = { 110, 1 };
	struct goodix_ts_data *ts;

	ts = i2c_get_clientdata(i2c_connect_client);
	dev_info(&ts->client->dev, "Begin calibration......\n");
	if ((*buf == 10) || (*buf == 49)) {
		int_wakeup_green(ts, 1);
		ret = i2c_write_bytes(ts->client, cal_cmd_buf, 2);
		if (ret != 1) {
			dev_info(&ts->client->dev, "Calibration failed!\n");
			return count;
		} else {
			dev_info(&ts->client->dev, "Calibration succeed!\n");
		}
	}
	return count;
}

#ifdef CONFIG_ADB_UPDATE
/*******************************************************
Description:
	Goodix debug sysfs echo update function.

Parameter:
	standard sysfs store param.

return:
	Executive outcomes..
*******************************************************/
static ssize_t goodix_debug_update_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t count)
{
	struct goodix_ts_data *ts;
	struct task_struct *thread = NULL;

	fw_path = (char *)vmalloc(count + 10);

	if (g_enter_isp)
		return count;
	ts = i2c_get_clientdata(i2c_connect_client);
	dev_info(&ts->client->dev, "Begin update......\n");
	printk("nums path:%d\n", count);
	memcpy(fw_path, buf, count);
	fw_path[count - 1] = '\0';

	total_len = 0;
	show_len = 0;
	printk("PATH:%s\n", fw_path);
	disable_irq(ts->client->irq);
	gpio_direction_output(irq_to_gpio(ts->client->irq), 0);	/*INT output low to wakeup Green Mode*/
	msleep(5);
	gpio_direction_input(irq_to_gpio(ts->client->irq));	/*resume INT*/
	enable_irq(ts->client->irq);
	thread = kthread_run(gt81x_update_fw, NULL, "guitar_update");
	return count;
}

static ssize_t goodix_debug_update_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct goodix_ts_data *ts;
	ts = i2c_get_clientdata(i2c_connect_client);
	sprintf(buf, "%d/%d\n", show_len, total_len);

	return strlen(buf);
}

/*******************************************************
Description:
	Goodix debug sysfs echo config function.

Parameter:
	standard sysfs store param.

return:
	Executive outcomes..
*******************************************************/
static ssize_t goodix_debug_config_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t count)
{
	struct goodix_ts_data *ts;
	char *path = (char *)vmalloc(count + 10);

	ts = i2c_get_clientdata(i2c_connect_client);
	dev_info(&ts->client->dev, "Begin config......\n");
	printk("nums path:%d\n", count);
	memcpy(path, buf, count);
	path[count - 1] = '\0';

	printk("PATH:%s\n", path);
	disable_irq(ts->client->irq);
	gpio_direction_output(irq_to_gpio(ts->client->irq), 0);	/*INT output low to wakeup Green Mode*/
	msleep(5);
	gpio_direction_input(irq_to_gpio(ts->client->irq));	/*resume INT*/
	enable_irq(ts->client->irq);
	gt81x_update_write_config(i2c_connect_client, path);
	vfree(path);
	return count;
}
#endif

static DEVICE_ATTR(version, S_IRUGO, goodix_debug_version_show, NULL);
static DEVICE_ATTR(resolution, S_IRUGO, goodix_debug_resolution_show, NULL);
static DEVICE_ATTR(diffdata, S_IRUGO, goodix_debug_diffdata_show, NULL);
static DEVICE_ATTR(calibration, S_IWUSR, NULL, goodix_debug_calibration_store);
#ifdef CONFIG_ADB_UPDATE
static DEVICE_ATTR(update, S_IWUSR | S_IRUGO, goodix_debug_update_show,
		   goodix_debug_update_store);
static DEVICE_ATTR(config, S_IWUSR, NULL, goodix_debug_config_store);
#endif

/*******************************************************
Description:
	Goodix debug sysfs init function.

Parameter:
	none.

return:
	Executive outcomes. 0---succeed.
*******************************************************/
static int goodix_debug_sysfs_init(void)
{
	int ret;
	struct goodix_ts_data *ts;
	ts = i2c_get_clientdata(i2c_connect_client);

	goodix_debug_kobj = kobject_create_and_add("goodix_debug", NULL);
	if (goodix_debug_kobj == NULL) {
		printk(KERN_ERR "%s: subsystem_register failed\n", __func__);
		ret = -ENOMEM;
		return ret;
	}
	ret = sysfs_create_file(goodix_debug_kobj, &dev_attr_version.attr);
	if (ret) {
		printk(KERN_ERR "%s: sysfs_create_version_file failed\n",
		       __func__);
		return ret;
	}
	ret = sysfs_create_file(goodix_debug_kobj, &dev_attr_calibration.attr);
	if (ret) {
		printk(KERN_ERR "%s: sysfs_create_calibration_file failed\n",
		       __func__);
		return ret;
	}
#ifdef CONFIG_ADB_UPDATE
	ret = sysfs_create_file(goodix_debug_kobj, &dev_attr_update.attr);
	if (ret) {
		printk(KERN_ERR "%s: sysfs_create_update_file failed\n",
		       __func__);
		return ret;
	}
	ret = sysfs_create_file(goodix_debug_kobj, &dev_attr_config.attr);
	if (ret) {
		printk(KERN_ERR "%s: sysfs_create_config_file failed\n",
		       __func__);
		return ret;
	}
#endif
	ret = sysfs_create_file(goodix_debug_kobj, &dev_attr_diffdata.attr);
	if (ret) {
		printk(KERN_ERR "%s: sysfs_create_diffdata_file failed\n",
		       __func__);
		return ret;
	}
	ret = sysfs_create_file(goodix_debug_kobj, &dev_attr_resolution.attr);
	if (ret) {
		printk(KERN_ERR "%s: sysfs_create_resolution_file failed\n",
		       __func__);
		return ret;
	}

	return 0;
}

static void goodix_debug_sysfs_deinit(void)
{
	sysfs_remove_file(goodix_debug_kobj, &dev_attr_version.attr);
	sysfs_remove_file(goodix_debug_kobj, &dev_attr_resolution.attr);
	sysfs_remove_file(goodix_debug_kobj, &dev_attr_diffdata.attr);
	sysfs_remove_file(goodix_debug_kobj, &dev_attr_calibration.attr);
	kobject_del(goodix_debug_kobj);
}

static int goodix_ts_regulator(int ast_touch_enable)
{

	int ret = 1;

	if (ast_touch_enable) {
		/*ts_en = regulator_get(NULL, "vdd_3v3_ts");  this is avalon's regulator name */
		ts_en = regulator_get(NULL, "vdd_5v0_ts");
		if (ts_en == NULL) {
			if (WARN_ON(IS_ERR(ts_en))) {
				pr_err
				    ("%s: couldn't get regulator vdd_3v3_ts: %ld\n",
				     __func__, PTR_ERR(ts_en));
				return -ENODEV;
			}
		}
		regulator_enable(ts_en);
	} else {
		regulator_disable(ts_en);
	}
	return ret;
}

/*******************************************************
Description:
	Goodix touchscreen probe function.

Parameter:
	client:	i2c device struct.
	id:device id.

return:
	Executive outcomes. 0---succeed.
*******************************************************/
static int goodix_ts_probe(struct i2c_client *client,
			   const struct i2c_device_id *id)
{
	int ret = 0;
	int retry = 0;
	struct goodix_ts_data *ts = NULL;
	char *version_info = NULL;
	char test_data = 1;
	const char irq_table[4] = { IRQ_TYPE_EDGE_RISING,
		IRQ_TYPE_EDGE_FALLING,
		IRQ_TYPE_LEVEL_LOW,
		IRQ_TYPE_LEVEL_HIGH
	};

	struct goodix_i2c_platform_data *plat_data = client->dev.platform_data;

	dev_dbg(&client->dev, "Install touch driver.\n");

	ret = goodix_ts_regulator(TOUCH_REGLATOR_ENABLE);
	if (ret < 0)
		return 0;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "Must have I2C_FUNC_I2C.\n");
		return -ENODEV;
	}
	ts = kzalloc(sizeof(*ts), GFP_KERNEL);
	if (ts == NULL) {
		pr_err("%s: failed to allocate driver data\n", __FUNCTION__);
		return -ENOMEM;
	}

	ret = gpio_request(plat_data->gpio_reset, "TS_RESET");
	if (ret < 0) {
		dev_err(&client->dev, "Failed to request GPIO:%d, ERRNO:%d\n",
			irq_to_gpio(plat_data->gpio_reset), ret);
		kfree(ts);
		return -EINVAL;
	}
	ts->use_reset = plat_data->gpio_reset;

	gpio_direction_output(ts->use_reset, 0);
	msleep(10);

	gpio_set_value(ts->use_reset, 1);
	msleep(100);

	goodix_wq = create_singlethread_workqueue("goodix_wq");	/*create a work queue and worker thread*/
	if (!goodix_wq) {
		printk(KERN_ALERT "creat workqueue failed\n");
		kfree(ts);
		gpio_free(ts->use_reset);
		return -ENOMEM;
	}

	ts->client = client;
	INIT_WORK(&ts->work, goodix_ts_work_func);
	i2c_set_clientdata(client, ts);

#ifdef INT_PORT
	ret = gpio_request(irq_to_gpio(client->irq), "TS_INT");
	if (ret < 0) {
		dev_err(&client->dev, "Failed to request GPIO:%d, ERRNO:%d\n",
			irq_to_gpio(client->irq), ret);
		client->irq = 0;
		goto err_gpio_request;
	}

	gpio_direction_output(irq_to_gpio(client->irq), 1);
	msleep(10);
	gpio_direction_input(irq_to_gpio(client->irq));
#endif

	i2c_connect_client = client;

	for (retry = 0; retry < 5; retry++) {
		/*ts->green_wake_mode = 1;                      //init enable wakeup*/
		/*int_wakeup_green(ts, 0);                      //maybe wakeup*/
		ret = i2c_write_bytes(client, &test_data, 1);
		if (ret > 0)
			break;
	}
	if (ret <= 0) {
		dev_err(&client->dev,
			"I2C communication ERROR!Goodix touchscreen driver become invalid\n");
		goto err_i2c_failed;
	}

	ret = goodix_init_panel(ts);
	if (ret != 0) {
		ts->bad_data = 1;
		goto err_init_godix_ts;
	}
#ifdef INT_PORT
	if (client->irq) {
		ret =
		    request_irq(client->irq, goodix_ts_irq_handler,
				irq_table[ts->int_trigger_type], client->name,
				ts);
		if (ret != 0) {
			dev_err(&client->dev,
				"Cannot allocate ts INT!ERRNO:%d\n", ret);
			gpio_direction_input(irq_to_gpio(client->irq));
			gpio_free(irq_to_gpio(client->irq));
			goto works_in_polling_mode;
		} else {
			disable_irq(client->irq);
			ts->use_irq = 1;
			dev_dbg(&client->dev,
				"Reques EIRQ %d succesd on GPIO:%d\n",
				client->irq, irq_to_gpio(client->irq));
		}
	}
#endif

      works_in_polling_mode:

	ts->input_dev = input_allocate_device();
	if (ts->input_dev == NULL) {
		ret = -ENOMEM;
		dev_dbg(&client->dev, "Failed to allocate input device\n");
		goto err_input_dev_alloc_failed;
	}

	ts->input_dev->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
	ts->input_dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);
	ts->input_dev->absbit[0] = BIT(ABS_X) | BIT(ABS_Y) | BIT(ABS_PRESSURE);	/*absolute coor (x,y)*/
#ifdef HAVE_TOUCH_KEY
	for (retry = 0; retry < MAX_KEY_NUM; retry++) {
		input_set_capability(ts->input_dev, EV_KEY,
				     touch_key_array[retry]);
	}
#endif

	input_set_abs_params(ts->input_dev, ABS_X, 0, ts->abs_x_max, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_Y, 0, ts->abs_y_max, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_PRESSURE, 0, 255, 0, 0);

#ifdef GOODIX_MULTI_TOUCH
	input_set_abs_params(ts->input_dev, ABS_MT_WIDTH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X, 0, ts->abs_x_max,
			     0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y, 0, ts->abs_y_max,
			     0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_TRACKING_ID, 0,
			     ts->max_touch_num, 0, 0);
#endif

	sprintf(ts->phys, "input/ts");
	ts->input_dev->name = goodix_ts_name;
	ts->input_dev->phys = ts->phys;
	ts->input_dev->id.bustype = BUS_I2C;
	ts->input_dev->id.vendor = 0xDEAD;
	ts->input_dev->id.product = 0xBEEF;
	ts->input_dev->id.version = 10427;	/*screen firmware version*/

	ret = input_register_device(ts->input_dev);
	if (ret) {
		dev_err(&client->dev,
			"Probe: Unable to register %s input device\n",
			ts->input_dev->name);
		goto err_input_register_device_failed;
	}
	ts->bad_data = 0;

	if (!ts->use_irq) {
		hrtimer_init(&ts->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		ts->timer.function = goodix_ts_timer_func;
		hrtimer_start(&ts->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
	}

	if (ts->use_irq) {
		enable_irq(client->irq);
		ts->power = goodix_ts_power;
	}
	ret = goodix_read_version(ts, &version_info);
	if (ret <= 0) {
		printk(KERN_INFO "Read version data failed!\n");
	} else {
		printk(KERN_INFO "Goodix TouchScreen Version:%s\n",
		       (version_info + 1));
	}

	vfree(version_info);

#ifdef CONFIG_HAS_EARLYSUSPEND
	ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	ts->early_suspend.suspend = goodix_ts_early_suspend;
	ts->early_suspend.resume = goodix_ts_late_resume;
	register_early_suspend(&ts->early_suspend);
#endif
#ifdef CONFIG_TOUCHSCREEN_GOODIX_IAP
	goodix_proc_entry = create_proc_entry("goodix-update", 0666, NULL);
	if (goodix_proc_entry == NULL) {
		dev_info(&client->dev, "Couldn't create proc entry!\n");
		ret = -ENOMEM;
		goto err_create_proc_entry;
	} else {
		goodix_proc_entry->write_proc = goodix_update_write;
		goodix_proc_entry->read_proc = goodix_update_read;
		goodix_proc_entry->owner = THIS_MODULE;
	}
#endif
	goodix_debug_sysfs_init();

#ifdef CREATE_WR_NODE
	init_wr_node(client);
#endif
	dev_info(&client->dev,
		 "Start %s in %s mode. Driver Modify Date:2011-08-11\n",
		 ts->input_dev->name, ts->use_irq ? "interrupt" : "polling");
	return 0;

      err_create_proc_entry:
      err_input_register_device_failed:
	input_free_device(ts->input_dev);
      err_input_dev_alloc_failed:
      err_init_godix_ts:
	if (ts->use_irq) {
		ts->use_irq = 0;
		free_irq(client->irq, ts);
#ifdef INT_PORT
		gpio_direction_input(irq_to_gpio(client->irq));
		gpio_free(irq_to_gpio(client->irq));
#endif
	} else
		hrtimer_cancel(&ts->timer);
      err_i2c_failed:
      err_gpio_request:
	i2c_set_clientdata(client, NULL);
	cancel_work_sync(&ts->work);
	if (goodix_wq)
		destroy_workqueue(goodix_wq);
	if (ts->use_reset)
		gpio_free(ts->use_reset);
	kfree(ts);

	return ret;
}

/*******************************************************
Description:
	Goodix touchscreen driver release function.

Parameter:
	client:	i2c device struct.

return:
	Executive outcomes. 0---succeed.
*******************************************************/
static int goodix_ts_remove(struct i2c_client *client)
{
	struct goodix_ts_data *ts = i2c_get_clientdata(client);
#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&ts->early_suspend);
#endif
#ifdef CONFIG_TOUCHSCREEN_GOODIX_IAP
	remove_proc_entry("goodix-update", NULL);
#endif
	goodix_debug_sysfs_deinit();
#ifdef CREATE_WR_NODE
	uninit_wr_node();
#endif
	if (ts && ts->use_irq) {
#ifdef INT_PORT
		gpio_direction_input(irq_to_gpio(client->irq));
		gpio_free(irq_to_gpio(client->irq));
#endif
		free_irq(client->irq, ts);
	} else if (ts)
		hrtimer_cancel(&ts->timer);

	dev_notice(&client->dev, "The driver is removing...\n");
	i2c_set_clientdata(client, NULL);
	if (goodix_wq)
		destroy_workqueue(goodix_wq);	/*release our work queue*/
	if (ts->use_reset)
		gpio_free(ts->use_reset);
	input_unregister_device(ts->input_dev);
	kfree(ts);
	goodix_ts_regulator(TOUCH_REGLATOR_DISABLE);
	return 0;
}

static int goodix_ts_suspend(struct i2c_client *client, pm_message_t mesg)
{
	int ret;
	struct goodix_ts_data *ts = i2c_get_clientdata(client);

	if (ts->use_irq)
		disable_irq(client->irq);
	else
		hrtimer_cancel(&ts->timer);

	if (ts->power) {
		ret = ts->power(ts, 0);
		if (ret < 0)
			printk(KERN_ERR "goodix_ts_resume power off failed\n");
	}

	return 0;
}

static int goodix_ts_resume(struct i2c_client *client)
{
	int ret;
	struct goodix_ts_data *ts = i2c_get_clientdata(client);

	if (ts->power) {
		ret = ts->power(ts, 1);
		if (ret < 0)
			printk(KERN_ERR "goodix_ts_resume power on failed\n");
	}
	if (!ts->use_irq) {
		hrtimer_start(&ts->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
	}
	return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void goodix_ts_early_suspend(struct early_suspend *h)
{
	struct goodix_ts_data *ts;
	ts = container_of(h, struct goodix_ts_data, early_suspend);
	goodix_ts_suspend(ts->client, PMSG_SUSPEND);
}

static void goodix_ts_late_resume(struct early_suspend *h)
{
	struct goodix_ts_data *ts;
	ts = container_of(h, struct goodix_ts_data, early_suspend);
	goodix_ts_resume(ts->client);
}
#endif

/******************************Begin of firmware update surpport*******************************/
#ifdef CONFIG_TOUCHSCREEN_GOODIX_IAP
/*
@brief CRC cal proc,include : Reflect,init_crc32_table,GenerateCRC32
@param global var oldcrc32
@return states
*/
static unsigned int Reflect(unsigned long int ref, char ch)
{
	unsigned int value = 0;
	int i;
	for (i = 1; i < (ch + 1); i++) {
		if (ref & 1)
			value |= 1 << (ch - i);
		ref >>= 1;
	}
	return value;
}

/*---------------------------------------------------------------------------------------*/
/*  CRC Check Program INIT								                                           		  */
/*---------------------------------------------------------------------------------------*/
static void init_crc32_table(void)
{
	unsigned int temp;
	unsigned int t1, t2;
	unsigned int flag;
	int i, j;
	for (i = 0; i <= 0xFF; i++) {
		temp = Reflect(i, 8);
		crc32_table[i] = temp << 24;
		for (j = 0; j < 8; j++) {

			flag = crc32_table[i] & 0x80000000;
			t1 = (crc32_table[i] << 1);
			if (flag == 0)
				t2 = 0;
			else
				t2 = ulPolynomial;
			crc32_table[i] = t1 ^ t2;

		}
		crc32_table[i] = Reflect(crc32_table[i], 32);
	}
}

/*---------------------------------------------------------------------------------------------------------*/
/*  CRC main Program									                                           		   */
/*---------------------------------------------------------------------------------------------------------*/
static void GenerateCRC32(unsigned char *buf, unsigned int len)
{
	unsigned int i;
	unsigned int t;

	for (i = 0; i != len; ++i) {
		t = (oldcrc32 ^ buf[i]) & 0xFF;
		oldcrc32 = ((oldcrc32 >> 8) & 0xFFFFFF) ^ crc32_table[t];
	}
}

static struct file *update_file_open(char *path, mm_segment_t * old_fs_p)
{
	struct file *filp = NULL;
	int errno = -1;

	filp = filp_open(path, O_RDONLY, 0644);

	if (!filp || IS_ERR(filp)) {
		if (!filp)
			errno = -ENOENT;
		else
			errno = PTR_ERR(filp);
		printk(KERN_ERR "The update file for Guitar open error.\n");
		return NULL;
	}
	*old_fs_p = get_fs();
	set_fs(get_ds());

	filp->f_op->llseek(filp, 0, 0);
	return filp;
}

static void update_file_close(struct file *filp, mm_segment_t old_fs)
{
	set_fs(old_fs);
	if (filp)
		filp_close(filp, NULL);
}

static int update_get_flen(char *path)
{
	struct file *file_ck = NULL;
	mm_segment_t old_fs;
	int length;

	file_ck = update_file_open(path, &old_fs);
	if (file_ck == NULL)
		return 0;

	length = file_ck->f_op->llseek(file_ck, 0, SEEK_END);
	if (length < 0)
		length = 0;
	update_file_close(file_ck, old_fs);
	return length;
}

static int update_file_check(char *path)
{
	unsigned char buffer[64] = { 0 };
	struct file *file_ck = NULL;
	mm_segment_t old_fs;
	int count, ret, length, check_version = 0;

	file_ck = update_file_open(path, &old_fs);

	if (path != NULL)
		printk("File Path:%s\n", path);

	if (file_ck == NULL)
		return -ERROR_NO_FILE;
	length = file_ck->f_op->llseek(file_ck, 0, SEEK_END);
#ifdef GUITAR_MESSAGE
	printk(KERN_INFO "gt801 update: File length: %d\n", length);
#endif

	if (length <= 0 || (length % 4) != 0) {
		update_file_close(file_ck, old_fs);
		return -ERROR_FILE_TYPE;
	}
	/*set file point to the begining of the file*/
	file_ck->f_op->llseek(file_ck, 0, SEEK_SET);
	oldcrc32 = 0xFFFFFFFF;
	init_crc32_table();
	while (length > 0) {
		ret =
		    file_ck->f_op->read(file_ck, buffer, sizeof(buffer),
					&file_ck->f_pos);
		if (ret > 0) {
			for (count = 0; count < ret; count++) {
				GenerateCRC32(&buffer[count], 1);
				if (buffer[count] == 0x47
				    && buffer[count + 1] == 0x54
				    && buffer[count + 2] == 0x38
				    && buffer[count + 3] == 0x31
				    && buffer[count + 4] == 0x58
				    && buffer[count + 5] == 0x4E
				    && buffer[count + 6] == 0x49
				    && buffer[count + 7] == 0x5F)
					check_version = 1;
			}
		} else {
			update_file_close(file_ck, old_fs);
			return -ERROR_FILE_READ;
		}
		length -= ret;
	}
	oldcrc32 = ~oldcrc32;
#ifdef GUITAR_MESSAGE
	printk("CRC_Check: %u\n", oldcrc32);
#endif
	update_file_close(file_ck, old_fs);
	return check_version;
}

unsigned char wait_slave_ready(struct goodix_ts_data *ts,
			       unsigned short *timeout)
{
	unsigned char i2c_state_buf[2] = { ADDR_STA, UNKNOWN_ERROR };
	int ret;
	while (*timeout < MAX_TIMEOUT) {
		ret = i2c_read_bytes(ts->client, i2c_state_buf, 2);
		if (ret <= 0)
			return ERROR_I2C_TRANSFER;
		if (i2c_state_buf[1] & SLAVE_READY) {
			return i2c_state_buf[1];
			/*return 1;*/
		}
		msleep(10);
		*timeout += 5;
	}
	return 0;
}

static int goodix_update_write(struct file *filp, const char __user * buff,
			       unsigned long len, void *data)
{
	unsigned char cmd[220];
	int ret = -1;

	static unsigned char update_path[100];
	static unsigned short time_count;
	static unsigned int file_len;

	unsigned char i2c_control_buf[2] = { ADDR_CMD, 0 };
	unsigned char i2c_states_buf[2] = { ADDR_STA, 0 };
	unsigned char i2c_data_buf[PACK_SIZE + 1 + 8] = { ADDR_DAT, };
	/*unsigned char i2c_rd_buf[1+4+PACK_SIZE+4];*/
	unsigned char i2c_rd_buf[160];
	unsigned char retries = 0;
	unsigned int rd_len;
	unsigned char i = 0;
	static unsigned char update_need_config;

	unsigned char checksum_error_times = 0;
#ifdef UPDATE_NEW_PROTOCOL
	unsigned int frame_checksum = 0;
	unsigned int frame_number = 0;
#else
	unsigned char send_crc = 0;
#endif

	struct file *file_data = NULL;
	mm_segment_t old_fs;
	struct goodix_ts_data *ts;

	ts = i2c_get_clientdata(i2c_connect_client);
	if (ts == NULL)
		return 0;

	if (copy_from_user(&cmd, buff, len)) {
		return -EFAULT;
	}
	switch (cmd[0]) {
	case STEP_SET_PATH:
		printk(KERN_INFO
		       "Write cmd is:%d,cmd arg is:%s,write len is:%ld\n",
		       cmd[0], &cmd[1], len);
		memset(update_path, 0, 100);
		strncpy(update_path, cmd + 1, 100);
		if (update_path[0] == 0)
			return 0;
		else
			return 1;
	case STEP_CHECK_FILE:
		printk(KERN_INFO "Begin to firmware update ......\n");
		ret = update_file_check(update_path);
		if (ret <= 0) {
			printk(KERN_INFO "failed to check update file!\n");
			return ret;
		}
		msleep(500);
		printk(KERN_INFO "Update check file success!\n");
		return 1;
	case STEP_WRITE_SYN:
		printk(KERN_INFO "STEP1:Write synchronization signal!\n");
		i2c_control_buf[1] = UPDATE_START;
		int_wakeup_green(ts, 0);
		ret = i2c_write_bytes(ts->client, i2c_control_buf, 2);
		if (ret <= 0) {
			ret = ERROR_I2C_TRANSFER;
			return ret;
		}
		/*the time include time(APROM -> LDROM) and time(LDROM init)*/
		msleep(1000);
		return 1;
	case STEP_WAIT_SYN:
		printk(KERN_INFO "STEP2:Wait synchronization signal!\n");
		while (retries < MAX_I2C_RETRIES) {
			i2c_states_buf[1] = UNKNOWN_ERROR;
			ret = i2c_read_bytes(ts->client, i2c_states_buf, 2);
			printk(KERN_INFO "The read byte is:%d\n",
			       i2c_states_buf[1]);
			if (i2c_states_buf[1] & UPDATE_START) {
				if (i2c_states_buf[1] & NEW_UPDATE_START) {
#ifdef UPDATE_NEW_PROTOCOL
					update_need_config = 1;
					return 2;
#else
					return 1;
#endif
				}
				break;
			}
			msleep(5);
			retries++;
			time_count += 10;
		}
		if ((retries >= MAX_I2C_RETRIES)
		    && (!(i2c_states_buf[1] & UPDATE_START))) {
			if (ret <= 0)
				return 0;
			else
				return -1;
		}
		return 1;
	case STEP_WRITE_LENGTH:
		printk(KERN_INFO "STEP3:Write total update file length!\n");
		file_len = update_get_flen(update_path);
		if (file_len <= 0) {
			printk(KERN_INFO "get update file length failed!\n");
			return -1;
		}
		file_len += 4;
		i2c_data_buf[1] = (file_len >> 24) & 0xff;
		i2c_data_buf[2] = (file_len >> 16) & 0xff;
		i2c_data_buf[3] = (file_len >> 8) & 0xff;
		i2c_data_buf[4] = file_len & 0xff;
		file_len -= 4;
		ret = i2c_write_bytes(ts->client, i2c_data_buf, 5);
		if (ret <= 0) {
			ret = ERROR_I2C_TRANSFER;
			return 0;
		}
		return 1;
	case STEP_WAIT_READY:
		printk(KERN_INFO "STEP4:Wait slave ready!\n");
		ret = wait_slave_ready(ts, &time_count);
		if (ret == ERROR_I2C_TRANSFER)
			return 0;
		if (!ret) {
			return -1;
		}
		printk(KERN_INFO "Slave ready!\n");
		return 1;
	case STEP_WRITE_DATA:
#ifdef UPDATE_NEW_PROTOCOL
		printk(KERN_INFO
		       "STEP5:Begin to send file data use NEW protocol!\n");
		file_data = update_file_open(update_path, &old_fs);
		if (file_data == NULL) {
			return -1;
		}
		frame_number = 0;
		while (file_len >= 0) {
			i2c_data_buf[0] = ADDR_DAT;
			rd_len = (file_len >= PACK_SIZE) ? PACK_SIZE : file_len;
			frame_checksum = 0;
			if (file_len) {
				ret =
				    file_data->f_op->read(file_data,
							  i2c_data_buf + 1 + 4,
							  rd_len,
							  &file_data->f_pos);
				if (ret <= 0) {
					printk
					    ("[GOODiX_ISP_NEW]:Read File Data Failed!\n");
					return -1;
				}
				i2c_data_buf[1] = (frame_number >> 24) & 0xff;
				i2c_data_buf[2] = (frame_number >> 16) & 0xff;
				i2c_data_buf[3] = (frame_number >> 8) & 0xff;
				i2c_data_buf[4] = frame_number & 0xff;
				frame_number++;
				frame_checksum = 0;
				for (i = 0; i < rd_len; i++) {
					frame_checksum += i2c_data_buf[5 + i];
				}
				frame_checksum = 0 - frame_checksum;
				i2c_data_buf[5 + rd_len + 0] =
				    frame_checksum & 0xff;
				i2c_data_buf[5 + rd_len + 1] =
				    (frame_checksum >> 8) & 0xff;
				i2c_data_buf[5 + rd_len + 2] =
				    (frame_checksum >> 16) & 0xff;
				i2c_data_buf[5 + rd_len + 3] =
				    (frame_checksum >> 24) & 0xff;
			}
		      rewrite:
			printk(KERN_INFO "[GOODiX_ISP_NEW]:%d\n", file_len);
			ret =
			    i2c_write_bytes(ts->client, i2c_data_buf,
					    1 + 4 + rd_len + 4);
			if (ret != 1) {
				printk
				    ("[GOODiX_ISP_NEW]:Write File Data Failed!Return:%d\n",
				     ret);
				return 0;
			}

			memset(i2c_rd_buf, 0x00, 1 + 4 + rd_len + 4);
			ret =
			    i2c_read_bytes(ts->client, i2c_rd_buf,
					   1 + 4 + rd_len + 4);
			if (ret <= 0) {
				printk
				    ("[GOODiX_ISP_NEW]:Read File Data Failed!Return:%d\n",
				     ret);
				return 0;
			}
			/*check communication*/
			for (i = 1; i < (1 + 4 + rd_len + 4); i++) {
				if (i2c_rd_buf[i] != i2c_data_buf[i]) {
					i = 0;
					break;
				}
			}
			if (!i) {
				i2c_control_buf[0] = ADDR_CMD;
				i2c_control_buf[1] = 0x03;
				i2c_write_bytes(ts->client, i2c_control_buf, 2);	/*communication error*/
				printk
				    ("[GOODiX_ISP_NEW]:File Data Frame readback check Error!\n");
			} else {
				i2c_control_buf[1] = 0x04;	/*let LDROM write flash*/
				i2c_write_bytes(ts->client, i2c_control_buf, 2);
			}

			/*Wait for slave ready signal.and read the checksum*/
			ret = wait_slave_ready(ts, &time_count);
			if ((ret & CHECKSUM_ERROR) || (!i)) {
				if (i) {
					printk
					    ("[GOODiX_ISP_NEW]:File Data Frame checksum Error!\n");
				}
				checksum_error_times++;
				msleep(20);
				if (checksum_error_times > 20)	/*max retry times.*/
					return 0;
				goto rewrite;
			}
			checksum_error_times = 0;
			if (ret & (FRAME_ERROR)) {
				printk
				    ("[GOODiX_ISP_NEW]:File Data Frame Miss!\n");
				return 0;
			}
			if (ret == ERROR_I2C_TRANSFER)
				return 0;
			if (!ret) {
				return -1;
			}
			if (file_len < PACK_SIZE) {
				update_file_close(file_data, old_fs);
				break;
			}
			file_len -= rd_len;
		}		/*end of while((file_len >= 0))*/
		return 1;
#else
		printk(KERN_INFO
		       "STEP5:Begin to send file data use OLD protocol!\n");
		file_data = update_file_open(update_path, &old_fs);
		if (file_data == NULL)	/*file_data has been opened at the last time*/
			return -1;

		while ((file_len >= 0) && (!send_crc)) {
			printk(KERN_INFO "[GOODiX_ISP_OLD]:%d\n", file_len);
			i2c_data_buf[0] = ADDR_DAT;
			rd_len = (file_len >= PACK_SIZE) ? PACK_SIZE : file_len;
			if (file_len) {
				ret =
				    file_data->f_op->read(file_data,
							  i2c_data_buf + 1,
							  rd_len,
							  &file_data->f_pos);
				if (ret <= 0) {
					return -1;
				}
			}
			if (file_len < PACK_SIZE) {
				send_crc = 1;
				update_file_close(file_data, old_fs);
				i2c_data_buf[file_len + 1] = oldcrc32 & 0xff;
				i2c_data_buf[file_len + 2] =
				    (oldcrc32 >> 8) & 0xff;
				i2c_data_buf[file_len + 3] =
				    (oldcrc32 >> 16) & 0xff;
				i2c_data_buf[file_len + 4] =
				    (oldcrc32 >> 24) & 0xff;
				ret =
				    i2c_write_bytes(ts->client, i2c_data_buf,
						    (file_len + 1 + 4));
				/*if(ret <= 0)*/
				if (ret != 1) {
					printk
					    ("[GOODiX_ISP_OLD]:Write File Data Failed!Return:%d\n",
					     ret);
					return 0;
				}
				break;
			} else {
				ret =
				    i2c_write_bytes(ts->client, i2c_data_buf,
						    PACK_SIZE + 1);
				/*if(ret <= 0)*/
				if (ret != 1) {
					printk
					    ("[GOODiX_ISP_OLD]:Write File Data Failed!Return:%d\n",
					     ret);
					return 0;
				}
			}
			file_len -= rd_len;

			/*Wait for slave ready signal.*/
			ret = wait_slave_ready(ts, &time_count);
			if (ret == ERROR_I2C_TRANSFER)
				return 0;
			if (!ret) {
				return -1;
			}
			/*Slave is ready.*/
		}		/*end of while((file_len >= 0) && (!send_crc))*/
		return 1;
#endif
	case STEP_READ_STATUS:
		printk(KERN_INFO "STEP6:Read update status!\n");
		while (time_count < MAX_TIMEOUT) {
			ret = i2c_read_bytes(ts->client, i2c_states_buf, 2);
			if (ret <= 0) {
				return 0;
			}
			if (i2c_states_buf[1] & SLAVE_READY) {
				if (!(i2c_states_buf[1] & 0xf0)) {
					printk(KERN_INFO
					       "The firmware updating succeed!update state:0x%x\n",
					       i2c_states_buf[1]);
					return 1;
				} else {
					printk(KERN_INFO
					       "The firmware updating failed!update state:0x%x\n",
					       i2c_states_buf[1]);
					return 0;

				}
			}
			msleep(1);
			time_count += 5;
		}
		return -1;
	case FUN_CLR_VAL:	/*clear the static val*/
		time_count = 0;
		file_len = 0;
		update_need_config = 0;
		return 1;
	case FUN_CMD:		/*functional command*/
		if (cmd[1] == CMD_DISABLE_TP) {
			printk(KERN_INFO "Disable TS int!\n");
			g_enter_isp = 1;
			if (ts->use_irq)
				disable_irq(ts->client->irq);
		} else if (cmd[1] == CMD_ENABLE_TP) {
			printk(KERN_INFO "Enable TS int!\n");
			g_enter_isp = 0;
			if (ts->use_irq) {
				disable_irq(ts->client->irq);
				enable_irq(ts->client->irq);
			}
		} else if (cmd[1] == CMD_READ_VER) {
			printk(KERN_INFO "Read version!\n");
			ts->read_mode = MODE_RD_VER;
		} else if (cmd[1] == CMD_READ_RAW) {
			printk(KERN_INFO "Read raw data!\n");
			ts->read_mode = MODE_RD_RAW;
			i2c_control_buf[1] = 201;
			ret = i2c_write_bytes(ts->client, i2c_control_buf, 2);	/*read raw data cmd*/
			if (ret <= 0) {
				printk(KERN_INFO
				       "Write read raw data cmd failed!\n");
				return 0;
			}
			msleep(200);
		} else if (cmd[1] == CMD_READ_DIF) {
			printk(KERN_INFO "Read diff data!\n");
			ts->read_mode = MODE_RD_DIF;
			i2c_control_buf[1] = 202;
			ret = i2c_write_bytes(ts->client, i2c_control_buf, 2);	/*read diff data cmd*/
			if (ret <= 0) {
				printk(KERN_INFO
				       "Write read raw data cmd failed!\n");
				return 0;
			}
			msleep(200);
		} else if (cmd[1] == CMD_READ_CFG) {
			printk(KERN_INFO "Read config info!\n");
			ts->read_mode = MODE_RD_CFG;
			rd_cfg_addr = cmd[2];
			rd_cfg_len = cmd[3];
		} else if (cmd[1] == CMD_SYS_REBOOT) {
			printk(KERN_INFO "System reboot!\n");
			sys_sync();
			msleep(200);
			kernel_restart(NULL);
		}
		return 1;
	case FUN_WRITE_CONFIG:

		printk(KERN_INFO "Begin write config info!Config length:%d\n",
		       cmd[1]);
		for (i = 3; i < cmd[1]; i++) {
			printk("(%d):0x%x ", i - 3, cmd[i]);
		}
		printk("\n");

		if ((cmd[2] > 83) && (cmd[2] < 240) && cmd[1]) {
			checksum_error_times = 0;
			if (!update_need_config)
				disable_irq(ts->client->irq);
			int_wakeup_green(ts, 0);
			if (!update_need_config)
				enable_irq(ts->client->irq);
		      reconfig:
			ret = i2c_write_bytes(ts->client, cmd + 2, cmd[1]);
			if (ret != 1) {
				printk("Write Config failed!return:%d\n", ret);
				return -1;
			}
			if (!update_need_config)
				return 1;

			i2c_rd_buf[0] = cmd[2];
			ret = i2c_read_bytes(ts->client, i2c_rd_buf, cmd[1]);
			if (ret <= 0) {
				printk("Read Config failed!return:%d\n", ret);
				return -1;
			}
			for (i = 0; i < cmd[1]; i++) {
				if (i2c_rd_buf[i] != cmd[i + 2]) {
					printk
					    ("Config readback check failed!\n");
					i = 0;
					break;
				}
			}
			if (!i) {
				i2c_control_buf[0] = ADDR_CMD;
				i2c_control_buf[1] = 0x03;
				i2c_write_bytes(ts->client, i2c_control_buf, 2);	/*communication error*/
				checksum_error_times++;
				msleep(20);
				if (checksum_error_times > 20)	/*max retry times.*/
					return 0;
				goto reconfig;
			} else {
				i2c_control_buf[0] = ADDR_CMD;
				i2c_control_buf[1] = 0x04;	/*let LDROM write flash*/
				i2c_write_bytes(ts->client, i2c_control_buf, 2);
				return 1;
			}

		} else {
			printk(KERN_INFO "Invalid config addr!\n");
			return -1;
		}
	default:
		return -ENOSYS;
	}
	return 0;
}

static int goodix_update_read(char *page, char **start, off_t off, int count,
			      int *eof, void *data)
{
	int ret = -1;
	struct goodix_ts_data *ts = NULL;
	int len = 0;
	char *version_info = NULL;
	char *read_data = (char *)vmalloc(1201);

	ts = i2c_get_clientdata(i2c_connect_client);
	if (ts == NULL)
		return 0;

	read_data[0] = 0x80;

	/*read version data*/
	if (ts->read_mode == MODE_RD_VER) {
		ret = goodix_read_version(ts, &version_info);
		if (ret <= 0) {
			printk(KERN_INFO "Read version data failed!\n");
			vfree(version_info);
			return 0;
		}

		for (len = 0; len < 100; len++) {
			if (*(version_info + len) == '\0')
				break;
		}

		strncpy(page, version_info + 1, len + 1);
		vfree(version_info);
		*eof = 1;
		return len + 1;
		/*read raw data or diff*/
	} else if ((ts->read_mode == MODE_RD_RAW) || (ts->read_mode == MODE_RD_DIF)) {
		/*printk(KERN_INFO"Read raw data\n");*/
		ret = i2c_read_bytes(ts->client, read_data, 1201);
		if (ret <= 0) {
			if (ts->read_mode == 2)
				printk(KERN_INFO "Read raw data failed!\n");
			if (ts->read_mode == 3)
				printk(KERN_INFO "Read diff data failed!\n");
			return 0;
		}
		memcpy(page, read_data + 1, 1200);
		*eof = 1;
		*start = NULL;
		vfree(read_data);
		return 1200;
	} else if (ts->read_mode == MODE_RD_CFG) {
		if ((rd_cfg_addr > 83) && (rd_cfg_addr < 240)) {
			read_data[0] = rd_cfg_addr;
			printk("read config addr is:%d\n", rd_cfg_addr);
		} else {
			read_data[0] = 101;
			printk("invalid read config addr,use default!\n");
		}
		if ((rd_cfg_len < 0) || (rd_cfg_len > 156)) {
			printk("invalid read config length,use default!\n");
			rd_cfg_len = 239 - read_data[0];
		}
		printk("read config length is:%d\n", rd_cfg_len);
		int_wakeup_green(ts, 1);
		ret = i2c_read_bytes(ts->client, read_data, rd_cfg_len);
		if (ret <= 0) {
			printk(KERN_INFO "Read config info failed!\n");
			vfree(read_data);
			return 0;
		}
		memcpy(page, read_data + 1, rd_cfg_len);
		vfree(read_data);
		return rd_cfg_len;
	}
	vfree(read_data);
	return len;
}

#endif
#ifdef CONFIG_ADB_UPDATE
static struct file *gt81x_file_open(char *path, mm_segment_t * old_fs_p)
{
	struct file *filp = NULL;
	int errno = -1;

	filp = filp_open(path, O_RDONLY, 0644);

	if (!filp || IS_ERR(filp)) {
		if (!filp)
			errno = -ENOENT;
		else
			errno = PTR_ERR(filp);
		printk(KERN_ERR "The update file for gt81x open error.\n");
		return NULL;
	}
	*old_fs_p = get_fs();
	set_fs(get_ds());

	filp->f_op->llseek(filp, 0, 0);
	return filp;
}

static void gt81x_file_close(struct file *filp, mm_segment_t old_fs)
{
	set_fs(old_fs);
	if (filp)
		filp_close(filp, NULL);
}

static int gt81x_get_file_length(char *path)
{
	struct file *file_ck = NULL;
	mm_segment_t old_fs;
	int length;

	file_ck = gt81x_file_open(path, &old_fs);
	if (file_ck == NULL)
		return 0;

	length = file_ck->f_op->llseek(file_ck, 0, SEEK_END);
	if (length < 0)
		length = 0;
	gt81x_file_close(file_ck, old_fs);
	return length;
}

static int gt81x_get_file_data(char *path)
{
	int fw_len, ret;
	struct file *file_ck = NULL;
	mm_segment_t old_fs;

	fw_len = gt81x_get_file_length(path);
	gt81x_fw = (char *)vmalloc(fw_len);
	file_ck = update_file_open(path, &old_fs);

	if (path != NULL)
		printk("File Path:%s\n", path);

	if (file_ck == NULL)
		return -ERROR_NO_FILE;

	if (fw_len <= 0 || (fw_len % 4) != 0) {
		update_file_close(file_ck, old_fs);
		return -ERROR_FILE_TYPE;
	}

	if (!update_file_check(path))
		return -ERROR_NO_FILE;

	/*set file point to the begining of the file*/
	file_ck->f_op->llseek(file_ck, 0, SEEK_SET);
	ret = file_ck->f_op->read(file_ck, gt81x_fw, fw_len, &file_ck->f_pos);
	if (ret < 0) {
		update_file_close(file_ck, old_fs);
		return -ERROR_FILE_READ;
	}

	update_file_close(file_ck, old_fs);
	return 1;
}

static int gt81x_read_regs(struct i2c_client *client, u8 reg, u8 buf[],
			   unsigned len)
{
	struct i2c_msg msgs[2];
	int ret = -1;
	int retries = 0;

	msgs[0].flags = !I2C_M_RD;
	msgs[0].addr = client->addr;
	msgs[0].len = 1;
	msgs[0].buf = &reg;

	msgs[1].flags = I2C_M_RD;
	msgs[1].addr = client->addr;
	msgs[1].len = len;
	msgs[1].buf = buf;

	while (retries < 5) {
		ret = i2c_transfer(client->adapter, msgs, 2);
		if (ret == 2)
			return ret;
		retries++;
	}
	return -1;
}

static int gt81x_set_regs(struct i2c_client *client, u8 reg, u8 const buf[],
			  unsigned short len)
{
	struct i2c_msg msg;
	int ret = -1;
	int retries = 0;
	u8 *data;		/*max 255*/
	data = (u8 *) vmalloc(len + 1);

	data[0] = reg;
	for (retries = 0; retries < len; retries++) {
		data[1 + retries] = buf[retries];
	}

	msg.flags = !I2C_M_RD;
	msg.addr = client->addr;
	msg.len = len + 1;
	msg.buf = data;

	retries = 0;
	while (retries < 5) {
		ret = i2c_transfer(client->adapter, &msg, 1);
		if (ret == 1)
			return ret;
		retries++;
		printk("i2c set failed\n");
	}
	vfree(data);
	return -1;
}

int gt81x_printf(char *buf, int len)
{
	int x, y, row = len / 8, mod = len % 8;
	for (y = 0; y < row; y++) {
		for (x = 0; x < 8; x++) {
			printk("0x%02x, ", buf[y * 8 + x]);
		}
		printk("\n");
	}
	for (x = 0; x < mod; x++) {
		printk("0x%02x, ", buf[row * 8 + x]);
	}
	printk("\n");
	return 0;
}

int gt81x_wait_for_slave(struct i2c_client *client, u8 status)
{
	unsigned char i2c_state_buf[2];
	int ret, i = 0;
	/*MAX_I2C_RETRIES)*/
	while (i < 5000) {
		ret = gt81x_read_regs(client, ADDR_STA, i2c_state_buf, 1);
		/*printk("i2c read state byte:0x%x\n",i2c_state_buf[0]);*/
		if (ret < 0)
			return ERROR_I2C_TRANSFER;
		if (i2c_state_buf[0] == 0xff)
			continue;
		if (i2c_state_buf[0] & status)
			return i2c_state_buf[0];
		msleep(10);
		i++;
	}
	return -ERROR_TIMEOUT;
}

static int hexToInt(char c)
{
	int x = (int)c;
	if (x >= 'a' && x <= 'f')
		x -= 'a' - 10;
	else if (x >= 'A' && x <= 'F')
		x -= 'A' - 10;
	else if (x >= '0' && x <= '9')
		x -= '0';
	else
		x = -1;
	return x;
}

int gt81x_update_write_config(struct i2c_client *client, char *cfg_path)
{
	int i, ret, len = 0;
	u8 cfg_rd_buf[200];
	/*u8 cfg_cmd_buf = 0x03;*/
	u8 retries = 0;
	char *config_info = NULL;
	int high, low;
	/**************file operation***************/
	struct file *file_ck = NULL;
	mm_segment_t old_fs;
	file_ck = gt81x_file_open(cfg_path, &old_fs);
	if (file_ck == NULL)
		return 0;
	len = file_ck->f_op->llseek(file_ck, 0, SEEK_END);
	gt81x_config_info = (char *)vmalloc(len);
	config_info = (char *)vmalloc(len / 5);

	file_ck->f_op->llseek(file_ck, 0, SEEK_SET);
	ret =
	    file_ck->f_op->read(file_ck, gt81x_config_info, len,
				&file_ck->f_pos);
	if (ret < 0) {
		update_file_close(file_ck, old_fs);
		return -ERROR_FILE_READ;
	}

	update_file_close(file_ck, old_fs);

	/**************end file op******************/
	for (i = 0; i < len; i += 5) {
		high = hexToInt(gt81x_config_info[i + 2]);
		low = hexToInt(gt81x_config_info[i + 3]);
		config_info[i / 5] = (char)(high * 16 + low);
	}
      reconfig:
	len = len / 5;
	printk("config length:%d\n", len);
	gt81x_printf(config_info, len);
	ret = gt81x_set_regs(client, 101, &config_info[1], len - 1);
	if (ret < 0) {
		printk("send config info failed!\n");
		return ret;
	}

	ret = gt81x_read_regs(client, 101, cfg_rd_buf, len - 1);
	if (ret < 0)
		return ret;
	if (memcmp(cfg_rd_buf, &config_info[1], len - 1)) {
		dev_info(&client->dev, "config info check error!\n");
		if (retries < 5) {
			retries++;
			goto reconfig;
		}
		return -1;
	}

	vfree(config_info);
	vfree(gt81x_config_info);
	return 0;
}

int gt81x_update_write_fw(struct i2c_client *client, char *fw_buf, int len)
{
	int ret, data_len, i, check_len, frame_checksum, frame_number = 0;
	unsigned char *p, i2c_data_buf[PACK_SIZE + 8];
	u8 i2c_rd_buf[PACK_SIZE + 8];

	u8 retries = 0;
	u8 check_state = 0;
	total_len = len;
	if (!client || !fw_buf)
		return -1;

	while (len) {
		frame_checksum = 0;
		retries = 0;
		check_len = (len >= PACK_SIZE) ? PACK_SIZE : len;
		data_len = check_len + 8;
		show_len = total_len - len;
		dev_info(&client->dev,
			 "PACK[%d]:prepare data,remained len = %d\n",
			 frame_number, len);
		p = &fw_buf[frame_number * PACK_SIZE];
		for (i = 0; i < check_len; i++)
			frame_checksum += *p++;
		frame_checksum = 0 - frame_checksum;
		p = i2c_data_buf;
		*p++ = (frame_number >> 24) & 0xff;
		*p++ = (frame_number >> 16) & 0xff;
		*p++ = (frame_number >> 8) & 0xff;
		*p++ = frame_number & 0xff;
		memcpy(p, &fw_buf[frame_number * PACK_SIZE], check_len);
		p += check_len;
		*p++ = frame_checksum & 0xff;
		*p++ = (frame_checksum >> 8) & 0xff;
		*p++ = (frame_checksum >> 16) & 0xff;
		*p++ = (frame_checksum >> 24) & 0xff;
		/*gt81x_printf(i2c_data_buf, data_len);*/
		dev_info(&client->dev, "PACK[%d]:write to slave\n",
			 frame_number);
	      resend:
		ret = gt81x_set_regs(client, ADDR_DAT, i2c_data_buf, data_len);
		if (ret < 0)
			return ret;
		/*gt81x_printf(i2c_data_buf, data_len);*/
		msleep(10);
		dev_info(&client->dev, "PACK[%d]:read data\n", frame_number);
		memset(i2c_rd_buf, 0, sizeof(i2c_rd_buf));
		ret = gt81x_read_regs(client, ADDR_DAT, i2c_rd_buf, data_len);
		if (ret < 0)
			return ret;
		/*gt81x_printf(i2c_data_buf, data_len);*/
		msleep(10);
		dev_info(&client->dev, "PACK[%d]:check data\n", frame_number);
		if (memcmp
		    (&i2c_rd_buf[4], &fw_buf[frame_number * PACK_SIZE],
		     check_len)) {
			dev_info(&client->dev,
				 "PACK[%d]:File Data Frame readback check Error!\n",
				 frame_number);
			i2c_rd_buf[0] = 0x03;
			ret = gt81x_set_regs(client, ADDR_CMD, i2c_rd_buf, 1);
			if (ret < 0)
				return ret;
			check_state = 0x01;
		} else {
			dev_info(&client->dev,
				 "PACK[%d]:tell slave check data pass\n",
				 frame_number);
			i2c_rd_buf[0] = 0x04;
			ret = gt81x_set_regs(client, ADDR_CMD, i2c_rd_buf, 1);
			if (ret < 0)
				return ret;
			dev_info(&client->dev,
				 "PACK[%d]:wait for slave to start next frame\n",
				 frame_number);
		}

		ret = gt81x_wait_for_slave(client, SLAVE_READY);
		if ((ret & CHECKSUM_ERROR) || (ret & FRAME_ERROR)
		    || (ret == ERROR_I2C_TRANSFER) || (ret < 0)
		    || (check_state == 0x01)) {

			if (((ret & CHECKSUM_ERROR) || (ret & FRAME_ERROR)
			     || (check_state == 0x01)) && (retries < 5)) {
				if (check_state != 0x01) {
					printk
					    ("checksum error or miss frame error!\n");
				}
				check_state = 0x00;
				retries++;
				msleep(20);
				goto resend;
			}
			printk("wait slave return state:%d\n", ret);
			show_len = total_len;
			return ret;
		}
		dev_info(&client->dev, "PACK[%d]:frame transfer finished\n",
			 frame_number);
		if (len < PACK_SIZE)
			return 0;
		frame_number++;
		len -= check_len;
	}
	total_len = -1;
	show_len = total_len;
	return 0;
}

int gt81x_update_fw(void *arg)
{
	int ret = 0, file_len, update_need_config;
	unsigned char i2c_control_buf[10];
	/*char version[17];*/
	char *version = NULL;
	const char version_base[17] = { "GT81XNI" };
	struct goodix_ts_data *ts;
	int fw_len;
	struct i2c_client *client = i2c_connect_client;
	ts = i2c_get_clientdata(i2c_connect_client);

	dev_info(&client->dev, "gt81x firmware update start...\n");
	fw_len = gt81x_get_file_length(fw_path);
	ret = gt81x_get_file_data(fw_path);
	if (ret != 1) {
		dev_info(&client->dev, "gt81x get file data failed\n");
		return 0;
	}

	dev_info(&client->dev, "step 1:read version...\n");
	ret = goodix_read_version(ts, &version);
	if (ret <= 0) {
		printk(KERN_INFO "Read version data failed!\n");
	} else {
		printk(KERN_INFO "Goodix TouchScreen Version:%s\n",
		       (version + 1));
	}
	vfree(version);
	g_enter_isp = 1;
	dev_info(&client->dev, "done!\n");
	dev_info(&client->dev, "step 2:disable irq...\n");
	disable_irq(client->irq);
	dev_info(&client->dev, "done!\n");
	dev_info(&client->dev, "step 3:set update start...\n");
	int_wakeup_green(ts, 0);
	i2c_control_buf[0] = UPDATE_START;
	ret = gt81x_set_regs(client, ADDR_CMD, i2c_control_buf, 1);
	if (ret < 0)
		return ret;
	/*the time include time(APROM -> LDROM) and time(LDROM init)*/
	msleep(1000);
	dev_info(&client->dev, "done!\n");
	dev_info(&client->dev, "step 4:wait for slave start...\n");
	ret = gt81x_wait_for_slave(client, UPDATE_START);
	if (ret < 0)
		return ret;
	if (!(ret & UPDATE_START))
		return -1;
	if (!(ret & NEW_UPDATE_START))
		update_need_config = 1;
	dev_info(&client->dev, "done!\n");
	dev_info(&client->dev, "step 5:write the fw length...\n");
	file_len = /*sizeof(gt81x_fw) */ fw_len + 4;
	dev_info(&client->dev, "file length is:%d\n", file_len);
	i2c_control_buf[0] = (file_len >> 24) & 0xff;
	i2c_control_buf[1] = (file_len >> 16) & 0xff;
	i2c_control_buf[2] = (file_len >> 8) & 0xff;
	i2c_control_buf[3] = file_len & 0xff;
	ret = gt81x_set_regs(client, ADDR_DAT, i2c_control_buf, 4);
	if (ret < 0)
		return ret;
	dev_info(&client->dev, "done!\n");
	dev_info(&client->dev, "step 6:wait for slave ready\n");
	ret = gt81x_wait_for_slave(client, SLAVE_READY);
	if (ret < 0)
		return ret;
	dev_info(&client->dev, "done!\n");
	dev_info(&client->dev, "step 7:write data\n");
	ret =
	    gt81x_update_write_fw(client, gt81x_fw, /*sizeof(gt81x_fw) */
				  fw_len);
	if (ret < 0)
		return ret;
	dev_info(&client->dev, "done!\n");
	dev_info(&client->dev, "step 9:wait for slave ready\n");
	ret = gt81x_wait_for_slave(client, SLAVE_READY);
	if (ret < 0)
		return ret;
	if (ret & SLAVE_READY)
		dev_info(&client->dev,
			 "The firmware updating succeed!update state:0x%x\n",
			 ret);
	dev_info(&client->dev, "step 10:enable irq...\n");
	enable_irq(client->irq);
	dev_info(&client->dev, "done!\n");
	msleep(1000);		/*wait slave reset*/
	vfree(gt81x_fw);
	dev_info(&client->dev, "step 11:read version...\n");
	ret = goodix_read_version(ts, &version);
	if (ret < 0)
		return ret;
	dev_info(&client->dev, "New Version:%s\n", version + 1);
	g_enter_isp = 0;
	version[7] = '\0';
	if (strcmp(version, version_base) == 0) {
		show_len = -1;
		total_len = -1;
		dev_info(&client->dev, "Update Failed!\n");
		return -1;
	}
	show_len = total_len;
	vfree(version);
	vfree(fw_path);
	return 0;
}
#endif
/******************************End of firmware update surpport*******************************/
static const struct i2c_device_id goodix_ts_id[] = {
	{GOODIX_I2C_NAME, 0},
	{}
};

static struct i2c_driver goodix_ts_driver = {
	.probe = goodix_ts_probe,
	.remove = goodix_ts_remove,
#ifndef CONFIG_HAS_EARLYSUSPEND
	.suspend = goodix_ts_suspend,
	.resume = goodix_ts_resume,
#endif
	.id_table = goodix_ts_id,
	.driver = {
		   .name = GOODIX_I2C_NAME,
		   .owner = THIS_MODULE,
		   },
};

/*******************************************************
Description:
	Driver Install function.
return:
	Executive Outcomes. 0---succeed.
********************************************************/
static int __devinit goodix_ts_init(void)
{
	int ret;

	ret = i2c_add_driver(&goodix_ts_driver);
	return ret;
}

/*******************************************************
Description:
	Driver uninstall function.
return:
	Executive Outcomes. 0---succeed.
********************************************************/
static void __exit goodix_ts_exit(void)
{
	printk(KERN_ALERT "Touchscreen driver of guitar exited.\n");
	i2c_del_driver(&goodix_ts_driver);
}

late_initcall(goodix_ts_init);
module_exit(goodix_ts_exit);

MODULE_DESCRIPTION("Goodix Touchscreen Driver");
MODULE_LICENSE("GPL");
