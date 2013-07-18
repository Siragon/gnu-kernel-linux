/*
 * kernel/drivers/media/video/tegra
 *
 * Aptina MT9D115 sensor driver
 *
 * Copyright (C) 2010 NVIDIA Corporation
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
#include <media/mt9d115.h>

#include "mt9d115_reg.h"

struct sensor_info {
	int mode;
	int still_count;
	int inited;
	struct i2c_client *i2c_client;
	struct mt9d115_platform_data *pdata;
};

static struct sensor_info *s_info;

#if 1
static int sensor_read_reg(struct i2c_client *client, u16 addr, u16 *val)
{
	int err;
	struct i2c_msg msg[2];
	unsigned char data[4];

	if (!client->adapter)
		return -ENODEV;

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].len = 2;
	msg[0].buf = data;

	/* high byte goes out first */
	data[0] = (u8) (addr >> 8);;
	data[1] = (u8) (addr & 0xff);

	msg[1].addr = client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = 2;
	msg[1].buf = data + 2;

	err = i2c_transfer(client->adapter, msg, 2);

	if (err != 2)
		return -EINVAL;

	*val = (data[2] << 8) | data[3];

	return 0;
}
#endif


static int sensor_write_reg(struct i2c_client *client, u16 addr, u16 val)
{
	int err;
	struct i2c_msg msg;
	unsigned char data[4];
	int retry = 0;

	if (!client->adapter)
		return -ENODEV;

	data[0] = (u8) (addr >> 8);
	data[1] = (u8) (addr & 0xff);
	data[2] = (u8) (val >> 8);
	data[3] = (u8) (val & 0xff);

	msg.addr = client->addr;
	msg.flags = 0;
	msg.len = 4;
	msg.buf = data;

	do {
		err = i2c_transfer(client->adapter, &msg, 1);
		if (err == 1)
			return 0;
		retry++;
		pr_err("yuv_sensor : i2c transfer failed, retrying %x %x\n",
		       addr, val);
		msleep(20);
	} while (retry <= MT9D115_MAX_RETRIES);

	return err;
}

static int sensor_poll_status(struct i2c_client *client, u16 *val,
	u16 expect_val, u16 delay_ms, u16 count)
{
	int err, i;
	for (i = 0; i < count; i++) {
		msleep(delay_ms);
		err = sensor_write_reg(client, 0x098C, 0xA104);
		if (err != 0)
			continue;
		err = sensor_read_reg(client, 0x0990, val);
		if (err != 0)
			continue;
		if (*val == expect_val) {
			pr_info("%s polling =0x%x on %d th polling\n",
				__func__, expect_val, i);
			return 0;
		}
	}
	pr_info("%s polling =0x%x timeout, 0x%x\n", __func__, expect_val, *val);
	return -EINVAL;
}

static int sensor_write_table(struct i2c_client *client,
			      const struct sensor_reg table[])
{
	int err;
	const struct sensor_reg *next;
	u16 val;

	pr_info("mt9d115 %s\n", __func__);
	for (next = table; next->addr != MT9D115_TABLE_END; next++) {
		if (next->addr == MT9D115_WAIT_MS) {
			msleep(next->val);
			continue;
		}

		val = next->val;

		err = sensor_write_reg(client, next->addr, val);
		if (err)
			return err;
	}
	return 0;
}

static int sensor_set_mode_preview(struct sensor_info *info, int index)
{
	int err;
	u16 val;
	if ((index == SENSOR_MODE_1600x1200) ||
		(index == SENSOR_MODE_1280x720)) {
		pr_info("%s: unsupported size index %d\n", __func__, index);
		return -EINVAL;
	}
	pr_info("%s %d\n", __func__, index);
	err = sensor_write_table(info->i2c_client,
		mt9d115_mode_table[index]);
	if (err)
		return err;
	if (info->still_count >= 1) {
		err = sensor_write_table(info->i2c_client,
			mt9d115_back_to_preview);
		if (err)
			return err;
	}
	err = sensor_poll_status(info->i2c_client, &val, 3, 50, 50);
	if (err)
		return err;
	return 0;
}

static int sensor_set_mode_capture(struct sensor_info *info, int index)
{
	int err;
	u16 val;
	pr_info("%s %d\n", __func__, index);
	err = sensor_write_table(info->i2c_client,
		mt9d115_mode_table_capture[index]);
	if (err)
		return err;
	err = sensor_poll_status(info->i2c_client, &val, 7, 50, 50);
	if (err)
		return err;
	return 0;
}

static int sensor_init_seq(struct sensor_info *info)
{
	int err;
	u16 val;
	pr_info("%s\n", __func__);
	err = sensor_write_table(info->i2c_client, mt9d115_init);
	if (err)
		return err;
	err = sensor_poll_status(info->i2c_client, &val, 3, 50, 50);
	if (err)
		return err;
	info->inited = 1;
	return 0;
}

static int sensor_set_mode(struct sensor_info *info, struct mt9d115_mode *mode)
{
	int sensor_table;
	int err;

	pr_info("%s: xres %u yres %u, still count=%d\n", __func__, mode->xres,
		mode->yres, mode->still_count);

	if (mode->xres == 1600 && mode->yres == 1200)
		sensor_table = SENSOR_MODE_1600x1200;
	else if (mode->xres == 1280 && mode->yres == 720)
		sensor_table = SENSOR_MODE_1280x720;
	else if (mode->xres == 800 && mode->yres == 600)
		sensor_table = SENSOR_MODE_800x600;
	else if (mode->xres == 800 && mode->yres == 480)
		sensor_table = SENSOR_MODE_800x480;
	else if (mode->xres == 768 && mode->yres == 432)
		sensor_table = SENSOR_MODE_768x432;
	else if (mode->xres == 640 && mode->yres == 480)
		sensor_table = SENSOR_MODE_640x480;
	else {
		pr_err("%s: invalid resolution supplied to set mode %d %d\n",
		       __func__, mode->xres, mode->yres);
		return -EINVAL;
	}

	if (mode->still_count >= 1)
		err = sensor_set_mode_capture(info, sensor_table);
	else
		err = sensor_set_mode_preview(info, sensor_table);

	if (err)
		return err;
	info->mode = sensor_table;
	info->still_count = mode->still_count;
	return 0;
}

static int sensor_set_item_effect(struct sensor_info *info, int value)
{
	pr_info("%s %d\n", __func__, value);
	switch (value) {
	case MT9D115_EFFECT_MONO:
		return sensor_write_table(info->i2c_client,
			mt9d115_effect_mono);
	case MT9D115_EFFECT_SEPIA:
		return sensor_write_table(info->i2c_client,
			mt9d115_effect_sepia);
	case MT9D115_EFFECT_NEGATIVE:
		return sensor_write_table(info->i2c_client,
			mt9d115_effect_negative);
	case MT9D115_EFFECT_SOLARIZE:
		return sensor_write_table(info->i2c_client,
			mt9d115_effect_solarize);
	case MT9D115_EFFECT_POSTERIZE:
		return sensor_write_table(info->i2c_client,
			mt9d115_effect_posterize);
	default:
		return sensor_write_table(info->i2c_client,
			mt9d115_effect_none);
	}
	return 0;
}

static int sensor_set_item_wb(struct sensor_info *info, int value)
{
	pr_info("%s %d\n", __func__, value);
	switch (value) {
	case MT9D115_WB_SUNLIGHT:
	case MT9D115_WB_CLOUDY:
		return sensor_write_table(info->i2c_client,
			mt9d115_wb_sunlight);
	case MT9D115_WB_FLUORESCENT:
		return sensor_write_table(info->i2c_client,
			mt9d115_wb_fluorescent);
	case MT9D115_WB_INCANDESCENT:
		return sensor_write_table(info->i2c_client,
			mt9d115_wb_incandescent);
	default:
		return sensor_write_table(info->i2c_client,
			mt9d115_wb_auto);
	}
	return 0;
}

static int sensor_set_item_brightness(struct sensor_info *info, int value)
{
	pr_info("%s %d\n", __func__, value);
	switch (value) {
	case MT9D115_BRIGHTNESS_P1:
		return sensor_write_table(info->i2c_client,
			mt9d115_brightness_p1);
	case MT9D115_BRIGHTNESS_P2:
		return sensor_write_table(info->i2c_client,
			mt9d115_brightness_p2);
	case MT9D115_BRIGHTNESS_N1:
		return sensor_write_table(info->i2c_client,
			mt9d115_brightness_n1);
	case MT9D115_BRIGHTNESS_N2:
		return sensor_write_table(info->i2c_client,
			mt9d115_brightness_n2);
	default:
		return sensor_write_table(info->i2c_client,
			mt9d115_brightness_0);
	}
	return 0;
}

static int sensor_set_item_scene(struct sensor_info *info, int value)
{
	pr_info("%s %d\n", __func__, value);
	switch (value) {
	case MT9D115_SCENE_ACTION:
		return sensor_write_table(info->i2c_client,
			mt9d115_scene_action);
	case MT9D115_SCENE_NIGHT:
		return sensor_write_table(info->i2c_client,
			mt9d115_scene_night);
	default:
		return sensor_write_table(info->i2c_client,
			mt9d115_scene_auto);
	}
	return 0;
}

static int sensor_set_effect(struct sensor_info *info,
				struct mt9d115_effect *effect)
{
	switch (effect->item) {
	case MT9D115_ITEM_EFFECT:
		return sensor_set_item_effect(info, effect->value);
	case MT9D115_ITEM_WB:
		return sensor_set_item_wb(info, effect->value);
	case MT9D115_ITEM_BRIGHTNESS:
		return sensor_set_item_brightness(info, effect->value);
	case MT9D115_ITEM_SCENE:
		return sensor_set_item_scene(info, effect->value);
	default:
		return -EINVAL;
	}
	return 0;
}

static int sensor_set_ae_rect(struct sensor_info *info,
				struct mt9d115_ae_rect *rect)
{
	u16 val1, val2;
	int err;
	pr_info("%s %d %d %d %d\n", __func__, rect->x, rect->y, rect->width,
		rect->height);
	if (rect->width == 0 && rect->height == 0) {
		/* set back to default */
		val1 = mt9d115_ae_rect_default[1].val;
		val2 = mt9d115_ae_rect_default[3].val;
	} else {
		val1 = ((rect->x & 0x00F0) >> 4) | (rect->y & 0x00F0);
		val2 = ((rect->width & 0x00F0) >> 4) | (rect->height & 0x00F0);
	}
	if ((val1 != mt9d115_ae_rectangular[1].val)
		|| (val2 != mt9d115_ae_rectangular[3].val)) {
		mt9d115_ae_rectangular[1].val = val1;
		mt9d115_ae_rectangular[3].val = val2;
		err = sensor_write_table(info->i2c_client,
					 mt9d115_ae_rectangular);
		if (err)
			return err;
	}
	return 0;
}

static long sensor_ioctl(struct file *file,
			 unsigned int cmd, unsigned long arg)
{
	int err;
	struct sensor_info *info = file->private_data;

	pr_info("mt9d115 %s\n", __func__);

	if (info->inited == 0) {
		err = sensor_init_seq(info);
		if (err)
			return err;
	}

	switch (cmd) {
	case MT9D115_IOCTL_SET_MODE:
	{
		struct mt9d115_mode mode;
		if (copy_from_user(&mode,
				   (const void __user *)arg,
				   sizeof(struct mt9d115_mode))) {
			return -EFAULT;
		}

		return sensor_set_mode(info, &mode);
	}
	case MT9D115_IOCTL_GET_STATUS:
	{

		return 0;
	}
	case MT9D115_IOCTL_SET_EFFECT:
	{
		struct mt9d115_effect effect;
		if (copy_from_user(&effect,
				   (const void __user *)arg,
				   sizeof(struct mt9d115_effect))) {
			return -EFAULT;
		}
		return sensor_set_effect(info, &effect);
	}
	case MT9D115_IOCTL_SET_AE_RECT:
	{
		struct mt9d115_ae_rect rect;
		if (copy_from_user(&rect, (const void __user *)arg,
				sizeof(struct mt9d115_ae_rect)))
			return -EFAULT;
		return sensor_set_ae_rect(info, &rect);
	}
	default:
		return -EINVAL;
	}
	return 0;
}

static int sensor_open(struct inode *inode, struct file *file)
{
	int err;
	pr_info("mt9d115 %s\n", __func__);
	file->private_data = s_info;
	if (s_info->pdata && s_info->pdata->power_on)
		s_info->pdata->power_on();
	s_info->inited = 0;
	s_info->still_count = 0;
	mt9d115_ae_rectangular[1].val = mt9d115_ae_rect_default[1].val;
	mt9d115_ae_rectangular[3].val = mt9d115_ae_rect_default[3].val;
	err = sensor_init_seq(s_info);
	if (err)
		return err;
	return 0;
}

int sensor_release(struct inode *inode, struct file *file)
{
	pr_info("mt9d115 %s\n", __func__);
	if (s_info->pdata && s_info->pdata->power_off)
		s_info->pdata->power_off();
	file->private_data = NULL;
	s_info->inited = 0;
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
	.name = MT9D115_NAME,
	.fops = &sensor_fileops,
};

static int sensor_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	int err;

	pr_info("mt9d115 %s\n", __func__);

	s_info = kzalloc(sizeof(struct sensor_info), GFP_KERNEL);

	if (!s_info) {
		pr_err("yuv_sensor : Unable to allocate memory!\n");
		return -ENOMEM;
	}

	err = misc_register(&sensor_device);
	if (err) {
		pr_err("yuv_sensor : Unable to register misc device!\n");
		kfree(s_info);
		return err;
	}

	s_info->pdata = client->dev.platform_data;
	s_info->i2c_client = client;

	i2c_set_clientdata(client, s_info);
	return 0;
}

static int sensor_remove(struct i2c_client *client)
{
	struct sensor_info *info;

	pr_info("mt9d115 %s\n", __func__);
	info = i2c_get_clientdata(client);
	misc_deregister(&sensor_device);
	kfree(info);
	return 0;
}

static const struct i2c_device_id sensor_id[] = {
	{ MT9D115_NAME, 0 },
	{ },
};

MODULE_DEVICE_TABLE(i2c, sensor_id);

static struct i2c_driver sensor_i2c_driver = {
	.driver = {
		.name = MT9D115_NAME,
		.owner = THIS_MODULE,
	},
	.probe = sensor_probe,
	.remove = sensor_remove,
	.id_table = sensor_id,
};

static int __init sensor_init(void)
{
	pr_info("mt9d115 %s\n", __func__);
	return i2c_add_driver(&sensor_i2c_driver);
}

static void __exit sensor_exit(void)
{
	pr_info("mt9d115 %s\n", __func__);
	i2c_del_driver(&sensor_i2c_driver);
}

module_init(sensor_init);
module_exit(sensor_exit);

