/*
 * kernel/drivers/media/video/tegra
 *
 * CE1502 ISP driver
 *
 * Copyright (C) 2011 Pegatron Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <media/yuv_sensor_common.h>
#include <media/ce1502.h>
#include <linux/firmware.h>

#define SENSOR_NAME	CE1502_NAME
#define SENSOR_MAX_RETRIES	3
#define SENSOR_MAX_POLLING	250
#define SENSOR_POLLING_DELAY	20

#define DEBUG_MODE_CAPTURE_FULL_PREVIEW	1

#define USE_INTERRUPT_AT_BOOT	1
#define FORCE_FACE_DETECT	0
#define FACE_FOCUS_IF_FACE_DETECT_ON	0

#define CMD_STATUS		0x6C
#define STATUS_STOP		0
#define STATUS_PREVIEWING	8
#define STATUS_FULL_PREVIEWING	11
#define STATUS_CAPTURING	1
#define STATUS_OUTPUT		20

#define CMD_AF_MODE		0x20
#define AF_MODE_INVALID		0xFF
#define AF_MODE_NORMAL		0
#define AF_MODE_MACRO		1
#define AF_MODE_CONTINUOUS	2

#define CMD_FLASH_CONTROL	0xB2
#define FLASH_CONTROL_OFF	0
#define FLASH_CONTROL_ON	1
#define FLASH_CONTROL_AUTO	2

#define AREA_LINK_AE		0x2
#define AREA_LINK_AF		0x1
#define AREA_LINK_ALL		(AREA_LINK_AE | AREA_LINK_AF)

enum {
	CE1502_IMAGE_LOADER_HEADER,
	CE1502_IMAGE_LOADER,
	CE1502_IMAGE_FW_HEADER,
	CE1502_IMAGE_FW,
	CE1502_IMAGE_END,
};

const char fw_file_name[CE1502_IMAGE_END][256] = {
	"CE150F00.bin",
	"CE150F01.bin",
	"CE150F02.bin",
	"CE150F03.bin",
};

struct sensor_info {
	int mode_index;
	int preview_xres;
	int preview_yres;
	int still_count;
	int flash_mode;
	int af_mode;
	u8 reg_value_af_mode;
	int face_detect;
	struct yuvsensor_region af_rect;
	struct yuvsensor_region ae_rect;
	int ae_lock;
	int wb_lock;
	u8 lock_setting;
	unsigned int debug_mode;
	struct i2c_client *i2c_client;
	struct yuvsensor_platform_data *pdata;
	struct work_struct work;
	struct mutex lock;
};

#define SENSOR_MSG_MAX_LEN	10

enum {
	SENSOR_MODE_UNINIT,
	SENSOR_MODE_INITED,
	SENSOR_MODE_3264x2448,
	SENSOR_MODE_1920x1080,
	SENSOR_MODE_1280x960,
	SENSOR_MODE_1280x720,
	SENSOR_MODE_640x480,
};

static struct sensor_info *s_info;
static unsigned int debug_mode;

/*
 * Sensor-specific implementation
 */

static int sensor_i2c_transfer(struct i2c_client *client, struct i2c_msg *msg,
	int num)
{
	int err, i;
	int retry = 0;

	if ((!client) || (!client->adapter))
		return -ENODEV;

	for (i = 0; i < num; i++)
		msg[i].addr = client->addr;

	do {
		err = i2c_transfer(client->adapter, msg, num);
		if (err == num)
			return 0;
		retry++;
		pr_err(SENSOR_NAME" %s: i2c transfer failed, retrying\n",
			__func__);
		msleep(20);
	} while (retry <= SENSOR_MAX_RETRIES);

	return err;
}
#if 0
static int sensor_i2c_dump(struct i2c_client *client, struct i2c_msg *msg,
	int num)
{
	int i;

	for (i = 0; i < msg[0].len; i++)
		pr_info(SENSOR_NAME" %s: 0x%02X", __func__, msg[0].buf[i]);

	return 0;
}
#endif
static int sensor_cmd(struct i2c_client *client, u8 cmd)
{
	struct i2c_msg msg;
	unsigned char buf;
	buf = cmd;

	msg.flags = 0;
	msg.len = 1;
	msg.buf = &buf;

	return sensor_i2c_transfer(client, &msg, 1);
}

static int sensor_cmd_data(struct i2c_client *client, u8 cmd, u8 data)
{
	struct i2c_msg msg;
	unsigned char buf[2];

	buf[0] = cmd;
	buf[1] = data;

	msg.flags = 0;
	msg.len = 2;
	msg.buf = buf;

	return sensor_i2c_transfer(client, &msg, 1);
}

static int sensor_cmd_res(struct i2c_client *client, u8 cmd, u8 *res)
{
	struct i2c_msg msg[2];
	unsigned char buf;

	buf = cmd;

	msg[0].flags = 0;
	msg[0].len = 1;
	msg[0].buf = &buf;

	msg[1].flags = I2C_M_RD;
	msg[1].len = 1;
	msg[1].buf = res;

	return sensor_i2c_transfer(client, msg, 2);
}

static int sensor_poll_res(struct i2c_client *client, u8 cmd, u8 *res,
	u8 expect_val)
{
	int i, err;
	for (i = 0; i < SENSOR_MAX_POLLING; i++) {
		err = sensor_cmd_res(client, cmd, res);
		if (!err && (expect_val == *res)) {
			pr_info(SENSOR_NAME
				" %s polling 0x%x = 0x%x on %dth polling\n",
				__func__, cmd, expect_val, i);
			return 0;
		}
		msleep(SENSOR_POLLING_DELAY);
	}
	pr_err(SENSOR_NAME" %s polling 0x%x = 0x%x timeout, 0x%x\n", __func__,
		cmd, expect_val, *res);
	return -EINVAL;
}

static int sensor_poll_af_status(struct i2c_client *client)
{
	u8 res;
	int i, err;
	for (i = 0; i < SENSOR_MAX_POLLING; i++) {
		err = sensor_cmd_res(client, 0x24, &res);
		if (!err && ((res & 0x01) == 0x00)) {
			pr_info(SENSOR_NAME
				" %s polling 0x24 = 0x%x on %dth polling\n",
				__func__, res, i);
			return 0;
		}
		msleep(SENSOR_POLLING_DELAY);
	}
	pr_err(SENSOR_NAME" %s timeout: last read %d\n", __func__, res);
	return -EINVAL;
}

static int sensor_boot_check(struct sensor_info *info)
{
#if USE_INTERRUPT_AT_BOOT
	int err;
	struct i2c_msg msg[2];
	unsigned char buf;
	unsigned char res[8];
	if (!(info->pdata->check_interrupt)) {
#endif
		/* Referenced from P7P10, should be reviewed */
		msleep(1000);
		return 1;
#if USE_INTERRUPT_AT_BOOT
	}
	if (info->pdata->check_interrupt(0)) {
		pr_info(SENSOR_NAME" %s: got interrupt\n", __func__);
		buf = 0xD0;
		msg[0].flags = 0;
		msg[0].len = 1;
		msg[0].buf = &buf;

		msg[1].flags = I2C_M_RD;
		msg[1].len = 8;
		msg[1].buf = res;

		err = sensor_i2c_transfer(info->i2c_client, msg, 2);
		if (err) {
			pr_err(SENSOR_NAME" %s: i2c error\n", __func__);
			return 0;
		}
		if ((0x01 & res[0]) == 0x01)
			return 1;
	}
#endif
	return 0;
}

static int sensor_boot_check_loop(struct sensor_info *info)
{
	int err = -EINVAL;
	int i;
	for (i = 0; i < 50; i++) {
		msleep(SENSOR_POLLING_DELAY);
		if (sensor_boot_check(info)) {
			pr_info(SENSOR_NAME" %s: boot ok at %dth poll\n",
				__func__, i);
			return 0;
		}
	}
	pr_err(SENSOR_NAME" %s: retry timeout\n", __func__);
	return err;
}

static int sensor_read_fw_version(struct sensor_info *info)
{
	int err;
	struct i2c_msg msg[2];
	unsigned char buf[2];
	unsigned char res[4];

	buf[0] = 0x00;
	buf[1] = 0x00;

	msg[0].flags = 0;
	msg[0].len = 2;
	msg[0].buf = buf;

	msg[1].flags = I2C_M_RD;
	msg[1].len = 4;
	msg[1].buf = res;

	err = sensor_i2c_transfer(info->i2c_client, msg, 2);
	if (err)
		pr_err(SENSOR_NAME" %s: read error\n", __func__);
	else
		pr_info(SENSOR_NAME" %s: 0x%x 0x%x 0x%x 0x%x\n",
			__func__, res[0], res[1], res[2], res[3]);
	return 0;
}

static int sensor_init_seq(struct sensor_info *info)
{
	int err;
	err = sensor_cmd(info->i2c_client, 0xF0);
	if (err)
		return err;

	err = sensor_boot_check_loop(info);
	if (err)
		return err;
	err = sensor_read_fw_version(info);
	if (err)
		return err;
	info->mode_index = SENSOR_MODE_INITED;
	return 0;
}

static int sensor_init_seq_loop(struct sensor_info *info)
{
	int err = -EINVAL;
	int retry = 0;
	do {
		retry++;
		pr_info(SENSOR_NAME" %s: round %d\n", __func__, retry);
		err = sensor_init_seq(info);
		if (!err)
			return 0;
		if (info->pdata && info->pdata->power_off)
			info->pdata->power_off();
		if (info->pdata && info->pdata->power_on)
			info->pdata->power_on();
	} while (retry <= SENSOR_MAX_RETRIES);
	pr_err(SENSOR_NAME" %s: retry timeout\n", __func__);
	return err;
}

static int sensor_start_full_preview(struct sensor_info *info)
{
	int err;
	u8 res;
	pr_info(SENSOR_NAME" %s\n", __func__);

	err = sensor_cmd_data(info->i2c_client, 0x6A, 0x01);
	if (err)
		return err;
	err = sensor_poll_res(info->i2c_client, CMD_STATUS, &res,
		STATUS_FULL_PREVIEWING);
	if (err)
		return err;
	return 0;
}

static int sensor_stop_full_preview(struct sensor_info *info)
{
	int err;
	u8 res;
	pr_info(SENSOR_NAME" %s\n", __func__);

	err = sensor_cmd_data(info->i2c_client, 0x6A, 0x00);
	if (err)
		return err;
	err = sensor_poll_res(info->i2c_client, CMD_STATUS, &res,
		STATUS_PREVIEWING);
	if (err)
		return err;
	return 0;
}

static int sensor_af_t_control(struct sensor_info *info, u8 value)
{
	int err;
	u8 res;
	if ((info->still_count != 0)
		|| (info->mode_index == SENSOR_MODE_INITED))
		return 0;
	value &= 1;
	err = sensor_cmd_data(info->i2c_client, 0x2C, value);
	if (err)
		return err;
	err = sensor_poll_res(info->i2c_client, 0x2D, &res, value);
	if (err)
		return err;
	return 0;
}

static int sensor_set_mode_capture(struct sensor_info *info, int index)
{
	int err;
	u8 res;
	struct i2c_msg msg;
	unsigned char buf[5];
	pr_info(SENSOR_NAME" %s %d\n", __func__, index);

/* Stop AF */
	if (info->af_mode == YUVSENSOR_AFMODE_CONTINUOUS_PICTURE) {
		err = sensor_af_t_control(info, 0);
		if (err)
			return err;
	}
	if (info->still_count == 0) {
		err = sensor_cmd(info->i2c_client, 0x35);
		if (err)
			return err;
	}
	err = sensor_poll_af_status(info->i2c_client);
	if (err)
		return err;

/* Capture size setting */
	buf[0] = 0x73;
	if (index == SENSOR_MODE_3264x2448)
		buf[1] = 0x21;
	else if (index == SENSOR_MODE_1280x720)
		buf[1] = 0x1A;
	else if (index == SENSOR_MODE_1920x1080)
		buf[1] = 0x1E;
	else if (index == SENSOR_MODE_640x480)
		buf[1] = 0x0B;
	else
		buf[1] = 0x1C;
	buf[2] = 0;
	buf[3] = 0;
	buf[4] = 0;

	msg.flags = 0;
	msg.len = 5;
	msg.buf = buf;

	err = sensor_i2c_transfer(info->i2c_client, &msg, 1);
	if (err)
		return err;

/* LED fire judgement */
	err = sensor_cmd_data(info->i2c_client, 0x11, 0x13);
	if (err)
		return err;

/* Lock AE/WB */
	err = sensor_cmd_data(info->i2c_client, 0x11, 0x11);
	if (err)
		return err;

/* Start capture */
	if (info->debug_mode == DEBUG_MODE_CAPTURE_FULL_PREVIEW)
		return sensor_start_full_preview(info);

	err = sensor_cmd(info->i2c_client, 0x74);
	if (err)
		return err;
	err = sensor_poll_res(info->i2c_client, CMD_STATUS, &res,
		STATUS_STOP);
	if (err)
		return err;

/* Data output setting */
	buf[0] = 0x67;
	buf[1] = 0;
	buf[2] = 1;
	buf[3] = 0;
	buf[4] = 0;

	msg.flags = 0;
	msg.len = 5;
	msg.buf = buf;

	err = sensor_i2c_transfer(info->i2c_client, &msg, 1);
	if (err)
		return err;

/* Ask ISP to send capture data */
	err = sensor_cmd(info->i2c_client, 0x68);
	if (err)
		return err;

	return 0;
}

static int sensor_set_facedetect(struct sensor_info *info, int value)
{
	int err;
	struct i2c_msg msg;
	unsigned char buf[11];
	pr_info(SENSOR_NAME" %s %d\n", __func__, value);

	if (value == YUVSENSOR_FACEDETECT_ON) {
		buf[0] = 0x41;
		buf[1] = 0x00;
		buf[2] = 0x0A;
		buf[3] = 0x03;
		buf[4] = 0x00;
		buf[5] = 0x00;
		buf[6] = 0x00;
		buf[7] = 0x00;
		buf[8] = 0x00;
		buf[9] = 0x00;
		buf[10] = 0x00;

		msg.flags = 0;
		msg.len = 11;
		msg.buf = buf;
		err = sensor_i2c_transfer(info->i2c_client, &msg, 1);
		if (err)
			return err;

		buf[0] = 0x4A;
		buf[1] = 0x00;
		buf[2] = 0x01;
		buf[3] = 0x00;
		buf[4] = 0x00;
		buf[5] = 0x00;
		buf[6] = 0x00;
		buf[7] = 0x00;
		buf[8] = 0x00;
		buf[9] = 0x00;
		buf[10] = 0x00;

		msg.flags = 0;
		msg.len = 11;
		msg.buf = buf;
		err = sensor_i2c_transfer(info->i2c_client, &msg, 1);
		if (err)
			return err;
		value = 1;
	} else
		value = 0;

	buf[0] = 0x42;
	buf[1] = value;

	msg.flags = 0;
	msg.len = 2;
	msg.buf = buf;
	err = sensor_i2c_transfer(info->i2c_client, &msg, 1);
	if (err)
		return err;

	return 0;
}

static int sensor_set_arealink(struct sensor_info *info,
	struct yuvsensor_region *rect, u8 link_option)
{
	int err, left, top, right, bottom;
	u8 res, value;
	struct i2c_msg msg;
	unsigned char buf[11];

	if (NULL == rect)
		pr_info(SENSOR_NAME" %s NULL\n", __func__);
	else
		pr_info(SENSOR_NAME" %s rel: (%d, %d) (%d, %d), %d\n", __func__,
		rect->left, rect->top, rect->right, rect->bottom, link_option);

	if (NULL == rect)
		value = 0;
	else if ((rect->left == rect->right) &&
		(rect->top == rect->bottom))
		value = 0;
	else {
		value = 0x05;
#if 0
		buf[0] = 0x4A;
		buf[1] = 0x00;
		buf[2] = 0x01;
		buf[3] = 0x00;
		buf[4] = 0x00;
		buf[5] = 0x00;
		buf[6] = 0x00;
		buf[7] = 0x00;
		buf[8] = 0x00;
		buf[9] = 0x00;
		buf[10] = 0x00;

		msg.flags = 0;
		msg.len = 11;
		msg.buf = buf;
		err = sensor_i2c_transfer(info->i2c_client, &msg, 1);
		if (err)
			return err;
#endif
		left = ((rect->left * info->preview_xres) / 2000)
			+ (info->preview_xres / 2);
		top = ((rect->top * info->preview_yres) / 2000)
			+ (info->preview_yres / 2);
		right = ((rect->right * info->preview_xres) / 2000)
			+ (info->preview_xres / 2);
		bottom = ((rect->bottom * info->preview_yres) / 2000)
			+ (info->preview_yres / 2);

		pr_info(SENSOR_NAME" %s abs: (%d, %d) (%d, %d)\n", __func__,
			left, top, right, bottom);
		buf[0] = 0x41;
		buf[1] = 0x05;
		buf[2] = (link_option & AREA_LINK_ALL);
		buf[3] = (left & 0xFF);
		buf[4] = ((left & 0xFF00) >> 8);
		buf[5] = (top & 0xFF);
		buf[6] = ((top & 0xFF00) >> 8);
		buf[7] = (right & 0xFF);
		buf[8] = ((right & 0xFF00) >> 8);
		buf[9] = (bottom & 0xFF);
		buf[10] = ((bottom & 0xFF00) >> 8);

		msg.flags = 0;
		msg.len = 11;
		msg.buf = buf;
		err = sensor_i2c_transfer(info->i2c_client, &msg, 1);
		if (err)
			return err;
	}

	buf[0] = 0x42;
	buf[1] = value;

	msg.flags = 0;
	msg.len = 2;
	msg.buf = buf;
	err = sensor_i2c_transfer(info->i2c_client, &msg, 1);
	if (err)
		return err;
	if ((info->mode_index != SENSOR_MODE_INITED) &&
		(info->still_count == 0)) {
		err = sensor_poll_res(info->i2c_client, 0x43, &res, value);
		if (err)
			return err;
	}
	return 0;
}

static int sensor_set_mode_preview(struct sensor_info *info, int index,
	int xres, int yres)
{
	int err;
	u8 res;
	u8 size_preset_num, drive_preset_num, isp_mode_num;
	struct i2c_msg msg;
	unsigned char buf[3];

	drive_preset_num = 0x01;
	isp_mode_num = 0x00;
	if (index == SENSOR_MODE_1280x720)
		size_preset_num = 0x1A;
	else if (index == SENSOR_MODE_1280x960)
		size_preset_num = 0x1C;
	else if (index == SENSOR_MODE_640x480)
		size_preset_num = 0x0B;
	else if (index == SENSOR_MODE_1920x1080) {
		size_preset_num = 0x1E;
		drive_preset_num = 0x03;
		isp_mode_num = 0x01;
	} else {
		pr_err(SENSOR_NAME" %s: unsupported size index %d\n", __func__,
			index);
		return -EINVAL;
	}

	pr_info(SENSOR_NAME" %s %d\n", __func__, index);

	err = sensor_poll_af_status(info->i2c_client);
	if (err)
		return err;

	if ((info->mode_index != SENSOR_MODE_INITED)
		&& (info->still_count == 0)) {
		pr_info(SENSOR_NAME" %s: stop previous preview first\n",
			__func__);
		/* Should stop previous preview first */
		err =  sensor_cmd_data(info->i2c_client, 0x6B, 0x00);
		if (err)
			return err;
		err = sensor_poll_res(info->i2c_client, CMD_STATUS, &res,
			STATUS_STOP);
		if (err)
			return err;
	}

/* Change ISP mode */
	err = sensor_cmd_data(info->i2c_client, 0x03, isp_mode_num);
	if (err)
		return err;
	mdelay(5);

	if ((info->debug_mode == DEBUG_MODE_CAPTURE_FULL_PREVIEW)
		&& (info->still_count != 0)) {
		err = sensor_stop_full_preview(info);
		if (err)
			return err;
		/* Unlock AE/WB */
		err = sensor_cmd_data(info->i2c_client, 0x11, 0x00);
		if (err)
			return err;
		return 0;
	}

/* Make a 200ms delay between (SND)68 & ISP sending picture */
	buf[0] = 0x05;
	buf[1] = 0x00;
	buf[2] = 0x06;

	msg.flags = 0;
	msg.len = 3;
	msg.buf = buf;

	err = sensor_i2c_transfer(info->i2c_client, &msg, 1);
	if (err)
		return err;

/* Preview size setting */
	buf[0] = 0x54;
	buf[1] = size_preset_num;
	buf[2] = drive_preset_num;

	msg.flags = 0;
	msg.len = 3;
	msg.buf = buf;

	err = sensor_i2c_transfer(info->i2c_client, &msg, 1);
	if (err)
		return err;

/* Flicker Reduction Auto */
	err = sensor_cmd_data(info->i2c_client, 0x14, 0x01);
	if (err)
		return err;

/* WB Auto */
	err = sensor_cmd_data(info->i2c_client, 0x1A, 0x00);
	if (err)
		return err;

/* Unlock AE/WB */
	err = sensor_cmd_data(info->i2c_client, 0x11, info->lock_setting);
	if (err)
		return err;
#if FORCE_FACE_DETECT
	info->face_detect = YUVSENSOR_FACEDETECT_ON;
#endif
	if (info->face_detect == YUVSENSOR_FACEDETECT_ON) {
		err = sensor_set_facedetect(info, YUVSENSOR_FACEDETECT_ON);
		if (err)
			return err;
	}

/* Start preview */
	err =  sensor_cmd_data(info->i2c_client, 0x6B, 0x01);
	if (err)
		return err;
	err = sensor_poll_res(info->i2c_client, CMD_STATUS, &res,
		STATUS_PREVIEWING);
	if (err)
		return err;

	info->preview_xres = xres;
	info->preview_yres = yres;

	if (info->af_mode == YUVSENSOR_AFMODE_CONTINUOUS_PICTURE) {
		/* We are already in preview now,
		but info->still_count is not updated yet */
		err = sensor_cmd_data(info->i2c_client, 0x2C, 1);
		if (err)
			return err;
		err = sensor_poll_res(info->i2c_client, 0x2D, &res, 1);
		if (err)
			return err;
	} else if (info->af_mode == YUVSENSOR_AFMODE_CONTINUOUS_VIDEO) {
		err = sensor_cmd(info->i2c_client, 0x23);
		if (err)
			return err;
	}

	err = sensor_set_arealink(info, &(info->ae_rect), AREA_LINK_AE);
	if (err)
		return err;

	return err;
}

static int sensor_set_mode(struct sensor_info *info,
	struct yuvsensor_mode *mode)
{
	int index, err;

	pr_info(SENSOR_NAME" %s: xres %d yres %d, still count=%d\n", __func__,
		mode->xres, mode->yres, mode->still_count);

	if (mode->xres == 3264 && mode->yres == 2448)
		index = SENSOR_MODE_3264x2448;
	else if (mode->xres == 1920 && mode->yres == 1080)
		index = SENSOR_MODE_1920x1080;
	else if (mode->xres == 1280 && mode->yres == 960)
		index = SENSOR_MODE_1280x960;
	else if (mode->xres == 1280 && mode->yres == 720)
		index = SENSOR_MODE_1280x720;
	else if (mode->xres == 640 && mode->yres == 480)
		index = SENSOR_MODE_640x480;
	else {
		pr_err(SENSOR_NAME" %s: invalid resolution supplied to set"
			" mode %d %d\n", __func__, mode->xres, mode->yres);
		return -EINVAL;
	}

	if ((info->mode_index == index) &&
		(info->still_count == mode->still_count))
		return 0;

	if ((mode->still_count >= 1) &&
		(info->mode_index == SENSOR_MODE_INITED)) {
		/* Case for nvtest*/
		err = sensor_set_mode_preview(info, SENSOR_MODE_1280x960,
			mode->xres, mode->yres);
		if (err)
			return err;
		err = sensor_set_mode_capture(info, index);
		if (err)
			return err;

	} else if ((mode->still_count >= 1) &&
		(info->mode_index != SENSOR_MODE_INITED))
		err = sensor_set_mode_capture(info, index);
	else
		err = sensor_set_mode_preview(info, index, mode->xres,
			mode->yres);

	if (err)
		return err;

	info->mode_index = index;
	info->still_count = mode->still_count;

	pr_info(SENSOR_NAME" %s --\n", __func__);
	return 0;
}

#if FACE_FOCUS_IF_FACE_DETECT_ON
static int sensor_trigger_face_af(struct sensor_info *info, int timeout)
{
	int err;
	struct i2c_msg msg;
	unsigned char buf[11];
	pr_info(SENSOR_NAME" %s %d\n", __func__, timeout);
	if (timeout == 0) {
		/* Stop AF */
		err = sensor_cmd(info->i2c_client, 0x35);
		if (err)
			return err;

		buf[0] = 0x4B;
		buf[1] = 0x00;
		buf[2] = 0x00;
		buf[3] = 0x00;
		buf[4] = 0x00;
		buf[5] = 0x00;
		buf[6] = 0x00;
		buf[7] = 0x00;
		buf[8] = 0x00;
		buf[9] = 0x00;
		buf[10] = 0x00;

		msg.flags = 0;
		msg.len = 11;
		msg.buf = buf;
		err = sensor_i2c_transfer(info->i2c_client, &msg, 1);
		if (err)
			return err;
		err = sensor_poll_af_status(info->i2c_client);
		if (err)
			return err;
		/* Push back to infinity */
		buf[0] = 0x33;
		buf[1] = 3;
		buf[2] = 0;
		buf[3] = 0;

		msg.flags = 0;
		msg.len = 4;
		msg.buf = buf;
		return sensor_i2c_transfer(info->i2c_client, &msg, 1);
	} else {
		buf[0] = 0x4B;
		buf[1] = 0x00;
		buf[2] = 0x01;
		buf[3] = 0x00;
		buf[4] = 0x00;
		buf[5] = 0x00;
		buf[6] = 0x00;
		buf[7] = 0x00;
		buf[8] = 0x00;
		buf[9] = 0x00;
		buf[10] = 0x00;

		msg.flags = 0;
		msg.len = 11;
		msg.buf = buf;
		err = sensor_i2c_transfer(info->i2c_client, &msg, 1);
		if (err)
			return err;
		/* Ensure lens is stopped before starting AF */
		err = sensor_poll_af_status(info->i2c_client);
		if (err)
			return err;
		/* Start AF */
		return sensor_cmd(info->i2c_client, 0x23);
	}
	return 0;
}
#endif

static int sensor_cmd_af_mode(struct sensor_info *info, u8 af_mode)
{
	int err;
	if ((AF_MODE_INVALID != af_mode) ||
		(af_mode != info->reg_value_af_mode)) {
		err = sensor_cmd_data(info->i2c_client, CMD_AF_MODE, af_mode);
		if (err)
			return err;
	}
	info->reg_value_af_mode = af_mode;
	return 0;
}

static int sensor_trigger_af(struct sensor_info *info, int timeout)
{
	int err;
	u8 af_mode;
	struct i2c_msg msg;
	unsigned char buf[4];
	if (info->mode_index == SENSOR_MODE_INITED)
		return 0;
#if FACE_FOCUS_IF_FACE_DETECT_ON
	if (info->face_detect == YUVSENSOR_FACEDETECT_ON)
		return sensor_trigger_face_af(info, timeout);
#endif
	pr_info(SENSOR_NAME" %s %d\n", __func__, timeout);
	if (timeout == 0) {
		if (info->af_mode == YUVSENSOR_AFMODE_CONTINUOUS_PICTURE)
			return sensor_af_t_control(info, 1);

		if (info->still_count == 0) {
			/* Stop AF */
			err = sensor_cmd(info->i2c_client, 0x35);
			if (err)
				return err;
		}
		err = sensor_poll_af_status(info->i2c_client);
		if (err)
			return err;
		/* Push back to infinity */
		buf[0] = 0x33;
		buf[1] = 3;
		buf[2] = 0;
		buf[3] = 0;

		msg.flags = 0;
		msg.len = 4;
		msg.buf = buf;
		err = sensor_i2c_transfer(info->i2c_client, &msg, 1);
		if (err)
			return err;
		err = sensor_poll_af_status(info->i2c_client);
		if (err)
			return err;
	} else {
		if (info->still_count != 0)
			return 0;
		else if (info->af_mode == YUVSENSOR_AFMODE_AUTO)
			af_mode = AF_MODE_NORMAL;
		else if (info->af_mode == YUVSENSOR_AFMODE_MACRO)
			af_mode = AF_MODE_MACRO;
		else if (info->af_mode == YUVSENSOR_AFMODE_CONTINUOUS_PICTURE)
			return sensor_af_t_control(info, 0);
		else
			return 0;
		err = sensor_poll_af_status(info->i2c_client);
		if (err)
			return err;
		err = sensor_cmd_af_mode(info, af_mode);
		if (err)
			return err;
		/* Ensure lens is stopped before starting AF */
		err = sensor_poll_af_status(info->i2c_client);
		if (err)
			return err;
		err = sensor_set_arealink(info, &(info->af_rect),
			AREA_LINK_ALL);
		if (err)
			return err;
		/* Start AF */
		err = sensor_cmd(info->i2c_client, 0x23);
		if (err)
			return err;
	}
	return 0;
}

static int sensor_get_af_status(struct sensor_info *info, int *status)
{
	int err;
	u8 res;

/* Get AF status */
	err = sensor_cmd_res(info->i2c_client, 0x24, &res);

/* Bit 2:0 == 0X0 */
	if ((res & 0x05) == 0x00)
		*status = 1;
	else
		*status = 0;
	pr_info(SENSOR_NAME" %s %d %d\n", __func__, *status, res);
	return 0;
}

static int sensor_get_face_status(struct sensor_info *info, int *status)
{
	int err;
	struct i2c_msg msg[2];
	unsigned char buf;
	unsigned char res[84];
	int num, i, x1, y1, x2, y2;

	buf = 0x9B;

	msg[0].flags = 0;
	msg[0].len = 1;
	msg[0].buf = &buf;

	msg[1].flags = I2C_M_RD;
	msg[1].len = 84;
	msg[1].buf = res;

	err = sensor_i2c_transfer(info->i2c_client, msg, 2);
	if (err) {
		pr_info(SENSOR_NAME" %s: read error\n", __func__);
		return err;
	}

	num = (res[0] | (res[1] << 8));
	pr_info(SENSOR_NAME" %s: %d faces detected\n", __func__, num);
	if (num > 10)
		num = 10;
	for (i = 0; i < num; i++) {
		x1 = (res[4 + 8 * i + 1] | (res[4 + 8 * i + 2] << 8));
		y1 = (res[4 + 8 * i + 3] | (res[4 + 8 * i + 4] << 8));
		x2 = (res[4 + 8 * i + 5] | (res[4 + 8 * i + 6] << 8));
		y2 = (res[4 + 8 * i + 7] | (res[4 + 8 * i + 8] << 8));
		pr_info(SENSOR_NAME" %s: face #%d : (%d, %d) (%d, %d)\n",
			__func__, i, x1, y1, x2, y2);
	}
	return 0;
}

static int sensor_store_afarea(struct sensor_info *info,
	struct yuvsensor_region *rect)
{
	pr_info(SENSOR_NAME" %s (%d, %d) (%d, %d)\n", __func__, rect->left,
		rect->top, rect->right, rect->bottom);

	info->af_rect.left = rect->left;
	info->af_rect.top = rect->top;
	info->af_rect.right = rect->right;
	info->af_rect.bottom = rect->bottom;

	return 0;
}

static int sensor_store_aearea(struct sensor_info *info,
	struct yuvsensor_region *rect)
{
	int err;
	pr_info(SENSOR_NAME" %s (%d, %d) (%d, %d)\n", __func__, rect->left,
		rect->top, rect->right, rect->bottom);

	info->ae_rect.left = rect->left;
	info->ae_rect.top = rect->top;
	info->ae_rect.right = rect->right;
	info->ae_rect.bottom = rect->bottom;

	if ((info->mode_index != SENSOR_MODE_INITED) &&
		(info->still_count == 0)) {
		err = sensor_set_arealink(info, &(info->ae_rect), AREA_LINK_AE);
		if (err)
			return err;
	}
	return 0;
}

static int sensor_batch_reflect(struct i2c_client *client)
{
	u8 res;
	int err;
	err = sensor_cmd(client, 0x01);
	if (err)
		return err;
	return sensor_poll_res(client, 0x02, &res, 0);
}

static int sensor_set_item_coloreffect(struct sensor_info *info, int value)
{
	int err;
	struct i2c_msg msg;
	unsigned char buf[3];
	pr_info(SENSOR_NAME" %s %d\n", __func__, value);
	buf[2] = 0;
	switch (value) {
	case YUVSENSOR_COLOREFFECT_NONE:
		buf[2] = 0;
		break;
	case YUVSENSOR_COLOREFFECT_MONO:
		buf[2] = 1;
		break;
	case YUVSENSOR_COLOREFFECT_SEPIA:
		buf[2] = 3;
		break;
	case YUVSENSOR_COLOREFFECT_NEGATIVE:
		buf[2] = 5;
		break;
	case YUVSENSOR_COLOREFFECT_SOLARIZE:
		buf[2] = 5;
		break;
	default:
		return -EINVAL;
	}
	buf[0] = 0x3D;
	buf[1] = 0x05;

	msg.flags = 0;
	msg.len = 3;
	msg.buf = buf;
	err = sensor_i2c_transfer(info->i2c_client, &msg, 1);
	if (err)
		return err;
	return sensor_batch_reflect(info->i2c_client);
}

static int sensor_set_item_wb(struct sensor_info *info, int value)
{
	int err;
	struct i2c_msg msg;
	unsigned char buf[3];
	pr_info(SENSOR_NAME" %s %d\n", __func__, value);
	buf[2] = 0;
	switch (value) {
	case YUVSENSOR_WB_AUTO:
		buf[2] = 0;
		break;
	case YUVSENSOR_WB_SUNLIGHT:
		buf[2] = 1;
		break;
	case YUVSENSOR_WB_CLOUDY:
		buf[2] = 2;
		break;
	case YUVSENSOR_WB_FLUORESCENT:
		buf[2] = 4;
		break;
	case YUVSENSOR_WB_INCANDESCENT:
		buf[2] = 3;
		break;
	default:
		return -EINVAL;
	}
	buf[0] = 0x1A;
	buf[1] = 0x00;

	msg.flags = 0;
	msg.len = 2;
	msg.buf = buf;
	err = sensor_i2c_transfer(info->i2c_client, &msg, 1);
	if (err)
		return err;

	buf[0] = 0x04;
	buf[1] = 0x11;

	msg.flags = 0;
	msg.len = 3;
	msg.buf = buf;
	err = sensor_i2c_transfer(info->i2c_client, &msg, 1);
	if (err)
		return err;
	return sensor_batch_reflect(info->i2c_client);
}

static int sensor_set_item_brightness(struct sensor_info *info, int value)
{
	int err;
	struct i2c_msg msg;
	unsigned char buf[3];
	pr_info(SENSOR_NAME" %s %d\n", __func__, value);
	buf[2] = 6;
	switch (value) {
	case YUVSENSOR_BRIGHTNESS_0:
		buf[2] = 6;
		break;
	case YUVSENSOR_BRIGHTNESS_P1:
		buf[2] = 9;
		break;
	case YUVSENSOR_BRIGHTNESS_P2:
		buf[2] = 12;
		break;
	case YUVSENSOR_BRIGHTNESS_N1:
		buf[2] = 3;
		break;
	case YUVSENSOR_BRIGHTNESS_N2:
		buf[2] = 0;
		break;
	default:
		return -EINVAL;
	}

	buf[0] = 0x04;
	buf[1] = 0x02;

	msg.flags = 0;
	msg.len = 3;
	msg.buf = buf;
	err = sensor_i2c_transfer(info->i2c_client, &msg, 1);
	if (err)
		return err;
	return sensor_batch_reflect(info->i2c_client);
}

static int sensor_set_item_scene(struct sensor_info *info, int value)
{
	int err;
	struct i2c_msg msg;
	unsigned char buf[3];
	unsigned char ev_curve;
	unsigned char photometry;
	unsigned char rgb_gamma;
	unsigned char uv_mapping;
	pr_info(SENSOR_NAME" %s %d\n", __func__, value);
	switch (value) {
	case YUVSENSOR_SCENE_AUTO:
		ev_curve = 0x0;
		photometry = 0x0;
		rgb_gamma = 0x0;
		uv_mapping = 0x0;
		break;
	case YUVSENSOR_SCENE_ACTION:
		ev_curve = 0xA;
		photometry = 0x2;
		rgb_gamma = 0x0;
		uv_mapping = 0x0;
		break;
	case YUVSENSOR_SCENE_NIGHT:
		ev_curve = 0xC;
		photometry = 0x0;
		rgb_gamma = 0x4;
		uv_mapping = 0x4;
		break;
	default:
		return -EINVAL;
	}

	buf[0] = 0x04;
	buf[1] = 0x01;
	buf[2] = ev_curve;
	msg.flags = 0;
	msg.len = 3;
	msg.buf = buf;
	err = sensor_i2c_transfer(info->i2c_client, &msg, 1);
	if (err)
		return err;

	buf[0] = 0x04;
	buf[1] = 0x00;
	buf[2] = photometry;
	msg.flags = 0;
	msg.len = 3;
	msg.buf = buf;
	err = sensor_i2c_transfer(info->i2c_client, &msg, 1);
	if (err)
		return err;

	buf[0] = 0x3D;
	buf[1] = 0x01;
	buf[2] = rgb_gamma;
	msg.flags = 0;
	msg.len = 3;
	msg.buf = buf;
	err = sensor_i2c_transfer(info->i2c_client, &msg, 1);
	if (err)
		return err;

	buf[0] = 0x3D;
	buf[1] = 0x03;
	buf[2] = uv_mapping;
	msg.flags = 0;
	msg.len = 3;
	msg.buf = buf;
	err = sensor_i2c_transfer(info->i2c_client, &msg, 1);
	if (err)
		return err;

	return sensor_batch_reflect(info->i2c_client);
}

static int sensor_set_strobe(struct sensor_info *info, u8 value)
{
	int err;
	struct i2c_msg msg;
	unsigned char buf[5];

	pr_info(SENSOR_NAME" %s %d\n", __func__, value);
	if (value != 0) {
		buf[0] = 0xB3;
		buf[1] = 0;
		buf[2] = 0;
		buf[3] = 0x01;
		buf[4] = 0;

		msg.flags = 0;
		msg.len = 5;
		msg.buf = buf;
		err = sensor_i2c_transfer(info->i2c_client, &msg, 1);
		if (err)
			return err;
	}
	buf[0] = 0x06;
	buf[1] = 0;
	buf[2] = value;

	msg.flags = 0;
	msg.len = 3;
	msg.buf = buf;
	return sensor_i2c_transfer(info->i2c_client, &msg, 1);
}

static int sensor_set_flash_af_assist(struct sensor_info *info, u8 value)
{
	int err;
	struct i2c_msg msg;
	unsigned char buf[3];

	buf[0] = CMD_FLASH_CONTROL;
	buf[1] = 1;
	buf[2] = value;

	msg.flags = 0;
	msg.len = 3;
	msg.buf = buf;
	err = sensor_i2c_transfer(info->i2c_client, &msg, 1);
	if (err)
		return err;

	return 0;
}

static int sensor_set_flash(struct sensor_info *info, u8 value)
{
	int err;
	struct i2c_msg msg;
	unsigned char buf[5];

	pr_info(SENSOR_NAME" %s %d\n", __func__, value);
	buf[0] = 0xB3;
	buf[1] = 0x01;
	buf[2] = 0;
	buf[3] = 0x10;
	buf[4] = 0;

	msg.flags = 0;
	msg.len = 5;
	msg.buf = buf;
	err = sensor_i2c_transfer(info->i2c_client, &msg, 1);
	if (err)
		return err;

	buf[0] = 0xB3;
	buf[1] = 0x03;
	buf[2] = 0;
	buf[3] = 0x10;
	buf[4] = 0;

	msg.flags = 0;
	msg.len = 5;
	msg.buf = buf;
	err = sensor_i2c_transfer(info->i2c_client, &msg, 1);
	if (err)
		return err;

	buf[0] = 0x07;
	buf[1] = 0x02;
	buf[2] = 0;

	msg.flags = 0;
	msg.len = 3;
	msg.buf = buf;
	err = sensor_i2c_transfer(info->i2c_client, &msg, 1);
	if (err)
		return err;

	buf[0] = 0x07;
	buf[1] = 0x03;
	buf[2] = 0;

	msg.flags = 0;
	msg.len = 3;
	msg.buf = buf;
	err = sensor_i2c_transfer(info->i2c_client, &msg, 1);
	if (err)
		return err;

	buf[0] = 0x07;
	buf[1] = 0x04;
	buf[2] = 0;

	msg.flags = 0;
	msg.len = 3;
	msg.buf = buf;
	err = sensor_i2c_transfer(info->i2c_client, &msg, 1);
	if (err)
		return err;

	/* Disable AF assistant flash for CAF */
	if (info->af_mode != YUVSENSOR_AFMODE_CONTINUOUS_PICTURE)
		err = sensor_set_flash_af_assist(info, value);
	else
		err = sensor_set_flash_af_assist(info, FLASH_CONTROL_OFF);
	if (err)
		return err;

	buf[0] = CMD_FLASH_CONTROL;
	buf[1] = 3;
	buf[2] = value;

	msg.flags = 0;
	msg.len = 3;
	msg.buf = buf;
	return sensor_i2c_transfer(info->i2c_client, &msg, 1);
}

static int sensor_set_item_flashmode(struct sensor_info *info, int value)
{
	int err;
	pr_info(SENSOR_NAME" %s %d\n", __func__, value);
	switch (value) {
	case YUVSENSOR_FLASHMODE_TORCH:
		/* CE1502: flash off command will cause torch off too. */
		err = sensor_set_flash(info, FLASH_CONTROL_OFF);
		if (err)
			return err;
		err = sensor_set_strobe(info, 1);
		if (err)
			return err;
		break;
	case YUVSENSOR_FLASHMODE_ON:
		err = sensor_set_strobe(info, 0);
		if (err)
			return err;
		err = sensor_set_flash(info, FLASH_CONTROL_ON);
		if (err)
			return err;
		break;
	case YUVSENSOR_FLASHMODE_AUTO:
		err = sensor_set_strobe(info, 0);
		if (err)
			return err;
		err = sensor_set_flash(info, FLASH_CONTROL_AUTO);
		if (err)
			return err;
		break;
	case YUVSENSOR_FLASHMODE_OFF:
		err = sensor_set_strobe(info, 0);
		if (err)
			return err;
		err = sensor_set_flash(info, FLASH_CONTROL_OFF);
		if (err)
			return err;
		break;
	default:
		return -EINVAL;
	}
	info->flash_mode = value;
	return 0;
}

static int sensor_set_item_afmode(struct sensor_info *info, int value)
{
/*
 * INFINITY & CONTINUOUS should take effect immediately;
 * for other cases, delay the applying process
 * to the time of triggering AF.
 */
	int err;
	u8 af_mode;
	pr_info(SENSOR_NAME" %s %d -> %d\n", __func__, info->af_mode, value);
	if (info->af_mode == value)
		return 0;
	switch (value) {
	case YUVSENSOR_AFMODE_INFINITY:
	case YUVSENSOR_AFMODE_AUTO:
	case YUVSENSOR_AFMODE_CONTINUOUS_PICTURE:
		af_mode = AF_MODE_NORMAL;
		break;
	case YUVSENSOR_AFMODE_MACRO:
		af_mode = AF_MODE_MACRO;
		break;
	case YUVSENSOR_AFMODE_CONTINUOUS_VIDEO:
		af_mode = AF_MODE_CONTINUOUS;
		break;
	default:
		return -EINVAL;
	}
	if (info->af_mode == YUVSENSOR_AFMODE_CONTINUOUS_PICTURE) {
		u8 flash_value = FLASH_CONTROL_OFF;
		err = sensor_af_t_control(info, 0);
		if (err)
			return err;
		/* leaving CAF, enable AF assistant flash if necessary */
		switch (info->flash_mode) {
		case YUVSENSOR_FLASHMODE_TORCH:
		case YUVSENSOR_FLASHMODE_OFF:
			flash_value = FLASH_CONTROL_OFF;
			break;
		case YUVSENSOR_FLASHMODE_ON:
			flash_value = FLASH_CONTROL_ON;
			break;
		case YUVSENSOR_FLASHMODE_AUTO:
			flash_value = FLASH_CONTROL_AUTO;
			break;
		}
		err = sensor_set_flash_af_assist(info, flash_value);
		if (err)
			return err;
	} else if ((info->af_mode == YUVSENSOR_AFMODE_CONTINUOUS_VIDEO) &&
		(info->still_count == 0)) {
		err = sensor_cmd(info->i2c_client, 0x35);
		if (err)
			return err;
	}
	if ((value == YUVSENSOR_AFMODE_INFINITY) ||
		(value == YUVSENSOR_AFMODE_CONTINUOUS_VIDEO)) {
		struct i2c_msg msg;
		unsigned char buf[4];
		err = sensor_poll_af_status(info->i2c_client);
		if (err)
			return err;
		err = sensor_cmd_af_mode(info, af_mode);
		if (err)
			return err;
		err = sensor_poll_af_status(info->i2c_client);
		if (err)
			return err;
		/* Push back to infinity */
		buf[0] = 0x33;
		buf[1] = 3;
		buf[2] = 0;
		buf[3] = 0;

		msg.flags = 0;
		msg.len = 4;
		msg.buf = buf;
		err = sensor_i2c_transfer(info->i2c_client, &msg, 1);
		if (err)
			return err;
		err = sensor_poll_af_status(info->i2c_client);
		if (err)
			return err;
		if ((value == YUVSENSOR_AFMODE_CONTINUOUS_VIDEO) &&
			(info->still_count == 0)) {
			err = sensor_cmd(info->i2c_client, 0x23);
			if (err)
				return err;
		}
	} else if (value == YUVSENSOR_AFMODE_CONTINUOUS_PICTURE) {
		err = sensor_poll_af_status(info->i2c_client);
		if (err)
			return err;
		err = sensor_set_arealink(info, NULL, 0);
		if (err)
			return err;
		err = sensor_cmd_af_mode(info, af_mode);
		if (err)
			return err;
		err = sensor_poll_af_status(info->i2c_client);
		if (err)
			return err;
		/* Disable AF assistant flash for CAF */
		err = sensor_set_flash_af_assist(info, FLASH_CONTROL_OFF);
		if (err)
			return err;
		err = sensor_af_t_control(info, 1);
		if (err)
			return err;
	}
	info->af_mode = value;
	return 0;
}

static int sensor_set_item_aelock(struct sensor_info *info, int value)
{
	int err;
	u8 lock_setting = info->lock_setting;
	if (info->ae_lock == value)
		return 0;
	switch (value) {
	case YUVSENSOR_AELOCK_OFF:
		lock_setting &= (~0x01);
		break;
	case YUVSENSOR_AELOCK_ON:
		lock_setting |= 0x01;
		break;
	default:
		return -EINVAL;
	}
	pr_info(SENSOR_NAME" %s %d -> %d, 0x%x\n", __func__, info->ae_lock,
		value, lock_setting);
	err = sensor_cmd_data(info->i2c_client, 0x11, lock_setting);
	if (err)
		return err;
	info->ae_lock = value;
	info->lock_setting = lock_setting;
	return 0;
}

static int sensor_set_effect(struct sensor_info *info,
				struct yuvsensor_effect *effect)
{
	switch (effect->item) {
	case YUVSENSOR_ITEM_COLOREFFECT:
		return sensor_set_item_coloreffect(info, effect->value);
	case YUVSENSOR_ITEM_WB:
		return sensor_set_item_wb(info, effect->value);
	case YUVSENSOR_ITEM_BRIGHTNESS:
		return sensor_set_item_brightness(info, effect->value);
	case YUVSENSOR_ITEM_SCENE:
		return sensor_set_item_scene(info, effect->value);
	case YUVSENSOR_ITEM_FLASHMODE:
		return sensor_set_item_flashmode(info, effect->value);
	case YUVSENSOR_ITEM_AFMODE:
		return sensor_set_item_afmode(info, effect->value);
	case YUVSENSOR_ITEM_AELOCK:
		return sensor_set_item_aelock(info, effect->value);
	default:
		return -EINVAL;
	}
	return 0;
}

static int sensor_get_exposure_time(struct sensor_info *info,
	struct yuvsensor_exposure *exposure)
{
/*
	int err;
	struct i2c_msg msg[2];
	unsigned char buf[5];
	unsigned char res[10];
	pr_info(SENSOR_NAME" %s: still_count = %d\n", __func__,
		info->still_count);

	if (info->still_count >= 1)
		err = sensor_poll_res(info->i2c_client, CMD_STATUS,
			res, STATUS_STOP);

	err = sensor_cmd_res(info->i2c_client, 0xBD, res);
	if (err)
		return err;
	exposure->flash_is_fired = res[0];

	buf[0] = 0x13;
	buf[1] = 0x02;
	buf[2] = 0;
	buf[3] = 0;
	buf[4] = 0;

	msg[0].flags = 0;
	msg[0].len = 5;
	msg[0].buf = buf;

	msg[1].flags = I2C_M_RD;
	msg[1].len = 10;
	msg[1].buf = res;

	err = sensor_i2c_transfer(info->i2c_client, msg, 2);
	if (err)
		return -EINVAL;
*/
/* Fill res[0]~res[9] into struct yuvsensor_exposure */

	return 0;
}

#define CE1502_HEADER_SIZE	4
#define CE1502_PAGE_SIZE	129

static int sensor_cmd_write_fw_page(struct i2c_client *client,
	const char *firmware_buf, size_t size, u8 *res)
{
	struct i2c_msg msg[2];
	unsigned char buf[CE1502_PAGE_SIZE + 1] = {0};

	if (size > CE1502_PAGE_SIZE)
		size = CE1502_PAGE_SIZE;

	buf[0] = 0xF3;
	memcpy(buf + 1, firmware_buf, size);

	msg[0].flags = 0;
	msg[0].len = CE1502_PAGE_SIZE + 1;
	msg[0].buf = buf;

	msg[1].flags = I2C_M_RD;
	msg[1].len = 1;
	msg[1].buf = res;

	return sensor_i2c_transfer(client, msg, 2);
}

static int update_firmware_imp(struct sensor_info *info,
	const char *firmware_buf, size_t size, int index)
{
	int err = 0;
	pr_info(SENSOR_NAME" %s++ index=%d, size=%u\n", __func__, index, size);
	if (!firmware_buf)
		return -EFAULT;
	switch (index) {
	case CE1502_IMAGE_LOADER_HEADER:
	case CE1502_IMAGE_FW_HEADER:
	{
		struct i2c_msg msg;
		unsigned char buf[CE1502_HEADER_SIZE + 1];

		if (size != CE1502_HEADER_SIZE) {
			pr_err(SENSOR_NAME" %s size wrong\n", __func__);
			return -EFAULT;
		}

		buf[0] = 0xF2;
		memcpy(buf + 1, firmware_buf, size);

		msg.flags = 0;
		msg.len = CE1502_HEADER_SIZE + 1;
		msg.buf = buf;

		err = sensor_i2c_transfer(info->i2c_client, &msg, 1);
		if (err)
			return err;
		break;
	}
	case CE1502_IMAGE_LOADER:
	case CE1502_IMAGE_FW:
	{
		int i;
		u8 res;
		u8 expect_val;
		int num_pages = size / CE1502_PAGE_SIZE;
		for (i = 0; i < num_pages; i++) {
			err = sensor_cmd_write_fw_page(info->i2c_client,
				firmware_buf + i * CE1502_PAGE_SIZE,
				CE1502_PAGE_SIZE, &res);
			if (err)
				return err;
			else if (res) {
				pr_err(SENSOR_NAME" %s F3 res!=0 on page %d\n",
					__func__, i);
				return -EFAULT;
			}
		}
		mdelay(10);

		if (index == CE1502_IMAGE_LOADER)
			expect_val = 0x05;
		else
			expect_val = 0x06;

		err = sensor_poll_res(info->i2c_client, 0xF5, &res, expect_val);
		if (err)
			return err;
		if (index == CE1502_IMAGE_FW) {
			err = sensor_init_seq_loop(info);
			if (err)
				return err;
		}
		break;
	}
	default:
		break;
	}
	pr_info(SENSOR_NAME" %s--\n", __func__);
	return 0;
}

static int update_firmware_loop(struct sensor_info *info)
{
	const struct firmware *fw_entry;
	int ret = -1;
	struct i2c_client *i2c = info->i2c_client;
	int i;
	for (i = CE1502_IMAGE_LOADER_HEADER; i < CE1502_IMAGE_END; i++) {
		ret = request_firmware(&fw_entry, fw_file_name[i], &i2c->dev);
		if (ret == 0) {
			ret = update_firmware_imp(info, fw_entry->data,
				fw_entry->size, i);
			release_firmware(fw_entry);
		}
		if (ret != 0)
			return ret;
	}
	return ret;
}

static int update_firmware(struct sensor_info *info)
{
	char event[32] = {0};
	char *envp[] = {event, NULL};
	int ret = -1;
	struct i2c_client *i2c = info->i2c_client;
	pr_info(SENSOR_NAME" %s\n", __func__);

	ret = update_firmware_loop(info);
	if (ret == 0) {
		snprintf(event, sizeof(event), "EVENT=FIRMWARE_UPDATE_SUCCESS");
		kobject_uevent_env(&i2c->dev.kobj, KOBJ_CHANGE, envp);
	} else {
		snprintf(event, sizeof(event), "EVENT=FIRMWARE_UPDATE_FAIL");
		kobject_uevent_env(&i2c->dev.kobj, KOBJ_CHANGE, envp);
	}
	pr_err(SENSOR_NAME" %s ret = %d\n", __func__, ret);
	return ret;
}

/*
 * General Interface
 *
 * Can be directly copy-paste if no special requirement.
 */

static long sensor_ioctl(struct file *file,
			 unsigned int cmd, unsigned long arg)
{
	int err;
	struct sensor_info *info = file->private_data;

	if (info->mode_index == SENSOR_MODE_UNINIT) {
		err = sensor_init_seq_loop(info);
		if (err)
			return err;
	}

	switch (cmd) {
	case YUVSENSOR_IOCTL_SET_MODE:
	{
		struct yuvsensor_mode mode;
		if (copy_from_user(&mode,
				   (const void __user *)arg,
				   sizeof(struct yuvsensor_mode))) {
			pr_err(SENSOR_NAME" %s err on line %d\n", __func__,
				__LINE__);
			return -EFAULT;
		}
		return sensor_set_mode(info, &mode);
	}
	case YUVSENSOR_IOCTL_TRIGGER_AF:
	{
		int timeout;
		if (copy_from_user(&timeout,
				   (const void __user *)arg, sizeof(int))) {
			pr_err(SENSOR_NAME" %s err on line %d\n", __func__,
				__LINE__);
			return -EFAULT;
		}
		return sensor_trigger_af(info, timeout);
	}
	case YUVSENSOR_IOCTL_GET_AF_STATUS:
	{
		int status;
		err = sensor_get_af_status(info, &status);
		if (err)
			return err;
		if (copy_to_user((void __user *)arg, &status, sizeof(int))) {
			pr_err(SENSOR_NAME" %s err on line %d\n", __func__,
				__LINE__);
			return -EFAULT;
		}
		return 0;
	}
	case YUVSENSOR_IOCTL_SET_EFFECT:
	{
		struct yuvsensor_effect effect;
		if (copy_from_user(&effect,
				   (const void __user *)arg,
				   sizeof(struct yuvsensor_effect))) {
			pr_err(SENSOR_NAME" %s err on line %d\n", __func__,
				__LINE__);
			return -EFAULT;
		}
		return sensor_set_effect(info, &effect);
	}
	case YUVSENSOR_IOCTL_GET_EXPOSURE_TIME:
	{
		struct yuvsensor_exposure exposure;
		err = sensor_get_exposure_time(info, &exposure);
		if (err)
			return err;
		if (copy_to_user((void __user *)arg, &exposure,
				 sizeof(struct yuvsensor_exposure))) {
			pr_err(SENSOR_NAME" %s err on line %d\n", __func__,
				__LINE__);
			return -EFAULT;
		}
		return 0;
	}
	case YUVSENSOR_IOCTL_GET_FACE_STATUS:
	{
		int status;
		err = sensor_get_face_status(info, &status);
		if (err)
			return err;
		if (copy_to_user((void __user *)arg, &status, sizeof(int))) {
			pr_err(SENSOR_NAME" %s err on line %d\n", __func__,
				__LINE__);
			return -EFAULT;
		}
		return 0;
	}
	case YUVSENSOR_IOCTL_SET_AF_AREA:
	{
		struct yuvsensor_region rect;
		if (copy_from_user(&rect,
				   (const void __user *)arg,
				   sizeof(struct yuvsensor_region))) {
			pr_err(SENSOR_NAME" %s err on line %d\n", __func__,
				__LINE__);
			return -EFAULT;
		}
		return sensor_store_afarea(info, &rect);
	}
	case YUVSENSOR_IOCTL_SET_AE_AREA:
	{
		struct yuvsensor_region rect;
		if (copy_from_user(&rect,
				   (const void __user *)arg,
				   sizeof(struct yuvsensor_region))) {
			pr_err(SENSOR_NAME" %s err on line %d\n", __func__,
				__LINE__);
			return -EFAULT;
		}
		return sensor_store_aearea(info, &rect);
	}
	default:
		pr_err(SENSOR_NAME" %s: unknown ioctl\n", __func__);
		return -EINVAL;
	}
	return 0;
}

static int sensor_open(struct inode *inode, struct file *file)
{
	pr_info(SENSOR_NAME" %s\n", __func__);
	file->private_data = s_info;
	if (s_info->pdata && s_info->pdata->power_on)
		s_info->pdata->power_on();
	s_info->mode_index = SENSOR_MODE_UNINIT;
	s_info->preview_xres = 0;
	s_info->preview_yres = 0;
	s_info->flash_mode = YUVSENSOR_FLASHMODE_OFF;
	s_info->af_mode = YUVSENSOR_AFMODE_INFINITY;
	s_info->face_detect = 0;
	s_info->debug_mode = debug_mode;
	s_info->reg_value_af_mode = AF_MODE_INVALID;
	s_info->ae_lock = YUVSENSOR_AELOCK_OFF;
	s_info->wb_lock = YUVSENSOR_AELOCK_OFF;
	s_info->lock_setting = 0;
	s_info->af_rect.left = 0;
	s_info->af_rect.top = 0;
	s_info->af_rect.right = 0;
	s_info->af_rect.bottom = 0;
	s_info->ae_rect.left = 0;
	s_info->ae_rect.top = 0;
	s_info->ae_rect.right = 0;
	s_info->ae_rect.bottom = 0;
	return 0;
}

static int sensor_release(struct inode *inode, struct file *file)
{
	pr_info(SENSOR_NAME" %s\n", __func__);
/*
 * It is always true that we want flash be turned off
 * when sensor being power off.
 */
	if (s_info->flash_mode == YUVSENSOR_FLASHMODE_TORCH)
		sensor_set_strobe(s_info, 0);
	if (s_info->pdata && s_info->pdata->power_off)
		s_info->pdata->power_off();
	return 0;
}


static const struct file_operations sensor_fileops = {
	.owner = THIS_MODULE,
	.open = sensor_open,
	.unlocked_ioctl = sensor_ioctl,
	.release = sensor_release,
};

static struct miscdevice sensor_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = SENSOR_NAME,
	.fops = &sensor_fileops,
};

static void ota_work_func(struct work_struct *work)
{
	struct sensor_info *info =
	    container_of(work, struct sensor_info, work);

	mutex_lock(&info->lock);
	update_firmware(info);
	mutex_unlock(&info->lock);
}

static ssize_t request_ota_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	ssize_t	status;
	struct i2c_client *i2c = to_i2c_client(dev);
	struct sensor_info *info = i2c_get_clientdata(i2c);

	pr_info(SENSOR_NAME" %s++\n", __func__);

	if (sysfs_streq(buf, "1"))
		status = schedule_work(&info->work);
	else
		status = -EINVAL;
	pr_info(SENSOR_NAME" %s-- %d\n", __func__, status);

	return status ? : size;
}

static ssize_t debug_mode_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	int ret;
	unsigned int param;
	pr_info(SENSOR_NAME" %s++\n", __func__);
	ret = sscanf(buf, "%u", &param);
	if (ret != 1)
		return -EFAULT;
	debug_mode = param;
	pr_info(SENSOR_NAME" %s %u\n", __func__, param);
	return size;
}

static DEVICE_ATTR(request_ota, 0644, 0, request_ota_store);
static DEVICE_ATTR(debug_mode, 0644, 0, debug_mode_store);

static int sensor_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	int err;

	pr_info(SENSOR_NAME" %s\n", __func__);

	s_info = kzalloc(sizeof(struct sensor_info), GFP_KERNEL);

	if (!s_info) {
		pr_err(SENSOR_NAME" %s: Unable to allocate memory!\n",
			__func__);
		return -ENOMEM;
	}

	err = misc_register(&sensor_device);
	if (err) {
		pr_err(SENSOR_NAME" %s: Unable to register misc device!\n",
			__func__);
		kfree(s_info);
		return err;
	}

	mutex_init(&s_info->lock);
	INIT_WORK(&s_info->work, ota_work_func);
	s_info->pdata = client->dev.platform_data;
	s_info->i2c_client = client;
	debug_mode = 0;

	i2c_set_clientdata(client, s_info);

	err = device_create_file(&client->dev, &dev_attr_request_ota);
	err = device_create_file(&client->dev, &dev_attr_debug_mode);
	return 0;
}

static int sensor_remove(struct i2c_client *client)
{
	struct sensor_info *info;

	pr_info(SENSOR_NAME" %s\n", __func__);
	info = i2c_get_clientdata(client);
	mutex_destroy(&info->lock);
	misc_deregister(&sensor_device);
	kfree(info);
	return 0;
}

static const struct i2c_device_id sensor_id[] = {
	{ SENSOR_NAME, 0 },
	{ },
};

MODULE_DEVICE_TABLE(i2c, sensor_id);

static struct i2c_driver sensor_i2c_driver = {
	.driver = {
		.name = SENSOR_NAME,
		.owner = THIS_MODULE,
	},
	.probe = sensor_probe,
	.remove = sensor_remove,
	.id_table = sensor_id,
};

static int __init sensor_init(void)
{
	pr_info(SENSOR_NAME" %s\n", __func__);
	return i2c_add_driver(&sensor_i2c_driver);
}

static void __exit sensor_exit(void)
{
	pr_info(SENSOR_NAME" %s\n", __func__);
	i2c_del_driver(&sensor_i2c_driver);
}

module_init(sensor_init);
module_exit(sensor_exit);
