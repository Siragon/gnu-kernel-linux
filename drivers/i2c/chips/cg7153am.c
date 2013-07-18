/*
 * cg7153am.c  --  Cypress CG7153AM MCU driver
 *
 * Copyright 2011 Pegatron
 *
 * Author: Andrew Hsiao<andrew_hsiao@pegatron.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/i2c/cg7153am.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/firmware.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/miscdevice.h>

#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/fcntl.h>
#include <linux/uaccess.h>


#define MCU_RETRY_MAX	3
#define MCU_RETRY_DELAY 10
#define MCU_WAKEUP_RETRY_MAX	5
#define MCU_WAKE_DELAY_TIME_MSEC	6
#define MCU_COMMAND_GAP_TIME_MSEC	1

/* Ex: 0x41 */
#define MCU_FW_STR_LEN 4

#ifdef DEBUG
#define LOG_FUNC() printk(KERN_INFO "%s\n", __func__)
#define LOG_MSG(x...) pr_err(x)
#else
#define LOG_FUNC() do {} while (0);
#define LOG_MSG(x...) do {} while (0);
#endif

static struct miscdevice cg7153am_misc_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "cg7153am",
};


#define OTA_STATUS_FAIL          -1
#define OTA_STATUS_IDLE          0
#define OTA_STATUS_UPGRADING     1
#define OTA_STATUS_COMPLETE      2
static atomic_t ota_status;

#define CG7153_FIRMWARE_UPDATE_EACH_WRITE 16


#define CG7153AM_REG_VERSION		0x30
/*VER:x*/
#define CG7153AM_REG_AMBER_LED_ON_OFF	0x60
/*OFF:0, ON:1*/
#define CG7153AM_REG_WHITE_LED_ON_OFF	0x70
/*OFF:0, ON:1*/
#define CG7153AM_REG_DISABLE_CHARGING	0x80
/*enable charging: 0, disable charging: 1*/

#define CG7153AM_DATA(_addr, _min_value, _max_value, _default_value)	\
[_addr] =							\
	{								\
			.addr = CG7153AM_REG_##_addr,			\
			.min_value = _min_value,			\
			.max_value = _max_value,			\
			.default_value = _default_value			\
	}

enum cg7153am_attribute {
	AMBER_LED_ON_OFF,
	VERSION,
	WHITE_LED_ON_OFF,
};

static struct cg7153am_device_data {
	u8 addr;
	int min_value;
	int max_value;
	int default_value;
} cg7153am_data[] = {
	CG7153AM_DATA(AMBER_LED_ON_OFF, 0x0, 0x1, 0x1),
	CG7153AM_DATA(VERSION, 0x0, 0xFF, 0x0),
	CG7153AM_DATA(WHITE_LED_ON_OFF, 0x0, 0x1, 0x1),
	/* all judgements are finished */
};

struct cg7153am_alert_condition {
	u8 low_bat_alert_1;
	u8 low_bat_alert_2;
	u8 low_bat_alert_3;
	u8 low_temp_alert;
	u8 high_temp_alert;
	u8 low_temp_shutdown;
	u8 recharge_temp_low;
	u8 recharge_temp_high;
};

struct cg7153am_drvdata {
	int fw_version;
	struct bin_attribute fw_version_attr;
};

#define FW_FILENAME_MAX_LENGTH   32

#define ROW_COUNT                462
#define COL_COUNT                16
#define FILE_SIZE                15316
#define DATA_SIZE                (ROW_COUNT * COL_COUNT * sizeof(char))

#define FW_COL_COUNT             16
#define FW_PAGE_COUNT            94  /*include head and tail*/

#define FW_PAGE_SIZE_EOL       2
#define FW_PAGE_SIZE_HEAD_DATA 20
#define FW_PAGE_SIZE_HEAD      (FW_PAGE_SIZE_HEAD_DATA + FW_PAGE_SIZE_EOL)
#define FW_PAGE_SIZE_BODY_ROW1_DATA    28
#define FW_PAGE_SIZE_BODY_ROW2_5_DATA  32
#define FW_PAGE_SIZE_BODY              (FW_PAGE_SIZE_EOL * 5 + \
	FW_PAGE_SIZE_BODY_ROW1_DATA + FW_PAGE_SIZE_BODY_ROW2_5_DATA * 4)
#define FW_PAGE_SIZE_TAIL_DATA  20
#define FW_PAGE_SIZE_TAIL      (FW_PAGE_SIZE_TAIL_DATA + FW_PAGE_SIZE_EOL)
#define FW_PAGE_SIZE                   FW_PAGE_SIZE_BODY
enum {
	PAGE_HEAD = 0,
	PAGE_BODY = 1,
	PAGE_TAIL = 2,
};

static void power_on_reset(struct i2c_client *i2c);

static void wakeup_mcu_imp(struct cg7153am_platform_data *pd)
{
	/*
	gpio_set_value(pd->gpio_wake, pd->gpio_polarity_wake);
	mdelay(MCU_WAKE_DELAY_TIME_MSEC);
	*/
}

void wakeup_mcu(struct i2c_client *i2c)
{
	struct cg7153am_platform_data *pd = i2c->dev.platform_data;
	wakeup_mcu_imp(pd);
}
EXPORT_SYMBOL(wakeup_mcu);

int suspend_mcu(struct i2c_client *i2c)
{
	int ret = 0;
	/*
	struct cg7153am_platform_data *pd = i2c->dev.platform_data;
	gpio_set_value(pd->gpio_wake, !pd->gpio_polarity_wake);
	mdelay(MCU_WAKE_DELAY_TIME_MSEC);
	*/
	return ret;
}
EXPORT_SYMBOL(suspend_mcu);


typedef int (*I2C_ACCESS_FUNC)(const struct i2c_client *client,
	char *buf, int count);
static int i2c_access_imp(struct i2c_client *i2c, char *data,
	int len, I2C_ACCESS_FUNC i2c_func)
{
	int i, ret = 0;

	mdelay(MCU_COMMAND_GAP_TIME_MSEC);
	for (i = 0; i < MCU_RETRY_MAX; i++) {
		ret = i2c_func(i2c, data, len);
		if (ret >= 0)
			break;
		else {
			printk(KERN_ERR "%s, i2c err, ret:%d, retry:%d\n",
				__func__, ret, i);
			/*i2c transfer error*/
			if (i == MCU_RETRY_MAX)
				power_on_reset(i2c);
			wakeup_mcu(i2c);
		}
	}

	return ret;
}

static int read_data(struct i2c_client *i2c, char *data, int len)
{
	return i2c_access_imp(i2c, data, len, i2c_master_recv);
}


static int write_data(struct i2c_client *i2c, const char *data, int len)
{

	return i2c_access_imp(i2c, (char *)data, len,
		(I2C_ACCESS_FUNC)i2c_master_send);
}

static int write_reg(struct i2c_client *i2c, u8 addr, u8 data)
{
	unsigned char buf[] = {addr, data};
	return write_data(i2c, buf, sizeof(buf));
}

static int read_reg(struct i2c_client *i2c, u8 addr, u8* data)
{
	struct i2c_adapter *adap = i2c->adapter;
	struct i2c_msg msg[2];
	int i, ret = 0;
	mdelay(MCU_COMMAND_GAP_TIME_MSEC);

	/*write*/
	msg[0].addr = i2c->addr;
	msg[0].flags = i2c->flags & I2C_M_TEN;
	msg[0].len = 1;
	msg[0].buf = (char *)&addr;

	/*read*/
	msg[1].addr = i2c->addr;
	msg[1].flags = i2c->flags & I2C_M_TEN;
	msg[1].flags |= I2C_M_RD;
	msg[1].len = 1;
	msg[1].buf = (char *)data;

	for (i = 0; i < MCU_RETRY_MAX; i++) {
		ret = i2c_transfer(adap, msg, 2);
		if (ret >= 0)
			break;
	}

	return ret;
}

static int get_version(struct i2c_client *i2c)
{
	int retry = 0;
	int ret = -EINVAL;
	char data;
	int version = 0;
	LOG_FUNC();

	for (retry = 0; retry < MCU_RETRY_MAX; retry++) {
		ret = read_reg(i2c, cg7153am_data[VERSION].addr, &data);
		if (ret >= 0) {
			version = data;
			break;
		}
	}
	return version;
}

static ssize_t cg7153am_attr_get_version(struct file *filp,
	struct kobject *kobj, struct bin_attribute *bin_attr,
	char *buf, loff_t off, size_t count)
{
	char fw_str[MCU_FW_STR_LEN+1];
	struct i2c_client *client = to_i2c_client(container_of(kobj,
		struct device, kobj));
	struct cg7153am_drvdata *devdata = i2c_get_clientdata(client);

	if (unlikely(!count))
		return count;

	if (off != 0)
		return 0;
	if (count != MCU_FW_STR_LEN)
		return 0;

	if (devdata) {
		devdata->fw_version = get_version(client);
		printk(KERN_INFO "%s:mcu version:0x%x\n",
			__func__, devdata->fw_version);
		sprintf(fw_str, "0x%02x", devdata->fw_version);
		memcpy(buf, fw_str, MCU_FW_STR_LEN);
	}
	return MCU_FW_STR_LEN;
}

static int enter_mcu_mode(struct i2c_client *i2c,
	struct cg7153am_platform_data *pd, int is_mcu_mode)
{
	int ret = 0;

	LOG_FUNC();
	if (is_mcu_mode) {
		gpio_set_value(pd->gpio_wake, pd->gpio_polarity_wake);
		mdelay(MCU_WAKE_DELAY_TIME_MSEC);
	} else {
		gpio_set_value(pd->gpio_wake, !pd->gpio_polarity_wake);
	}
	return ret;
}

static int set_led(struct i2c_client *i2c, enum cg7153am_led_mode mode,
	int on_off_reg_offset)
{
	int ret = -EINVAL;
	LOG_FUNC();

	wakeup_mcu(i2c);

	switch (mode) {
	case MCU_LED_ON:
		/*Turn on LED*/
		ret = write_reg(i2c,
			cg7153am_data[on_off_reg_offset].addr, 0x01);
	break;
	case MCU_LED_OFF:
		/*Turn off LED*/
		ret = write_reg(i2c,
			cg7153am_data[on_off_reg_offset].addr, 0x00);
	break;
	default:
	break;
	}

	suspend_mcu(i2c);
	return ret;
}

int cg7153am_led_set(struct device *dev, int index,
	enum cg7153am_led_mode mode)
{
	struct i2c_client *i2c = to_i2c_client(dev);

	LOG_FUNC();
	/*To check i2c at first in all EXPORT function*/
	if (i2c == 0)
		return -ENODEV;
	switch (index) {
	case AMBER:
		return set_led(i2c, mode, AMBER_LED_ON_OFF);
	break;
	case WHITE:
		return set_led(i2c, mode, WHITE_LED_ON_OFF);
	break;
	}
	return -ENODEV;
}
EXPORT_SYMBOL(cg7153am_led_set);

int cg7153am_version_get(struct i2c_client *i2c)
{
	if (i2c == 0)
		return -ENODEV;
	return get_version(i2c);
}
EXPORT_SYMBOL(cg7153am_version_get);

static int destroy_each_dev(struct device *each_dev, void *unused)
{
	LOG_FUNC();
	platform_device_unregister(to_platform_device(each_dev));
	return 0;
}

static int destroy_sub_devs(struct device *dev)
{
	LOG_FUNC();
	return device_for_each_child(dev, NULL, destroy_each_dev);
}

static int create_sub_devs(struct device *dev)
{
	int i, ret;
	struct cg7153am_platform_data *pd = dev->platform_data;
	struct cg7153am_subdev_info *each_dev;
	struct platform_device *pdev;

	LOG_FUNC();
	for (i = 0; i < pd->num_subdevs; i++) {
		each_dev  = &pd->sub_devs[i];

		LOG_MSG("%s:name:%s, id:%d, num_subdevs:%d\n", __func__,
			each_dev->name, each_dev->id, pd->num_subdevs);

		pdev = platform_device_alloc(each_dev->name, each_dev->id);
		/*associate parent and platform_data for each sub_dev*/
		pdev->dev.parent = dev;
		pdev->dev.platform_data = each_dev->platform_data;
		ret = platform_device_add(pdev);
		if (ret) {
			destroy_sub_devs(dev);
			return ret;
		}
	}
	return ret;
}

struct ota_data {
	struct i2c_client *this_client;
	struct cg7153am_platform_data *pd;
	struct work_struct work;
	struct mutex lock;
};

static char ascii_to_hex(const char ch)
{
	if (('0' <= ch) && (ch <= '9'))
		return ch - '0';
	else if (('A' <= ch) && (ch <= 'F'))
		return 10 + ch - 'A';
	else if (('a' <= ch) && (ch <= 'f'))
		return 10 + ch - 'a';
	else
		return 0;
}

/*return 0 for success, others for fail*/
static int write_firmware_page(struct i2c_client *i2c, int blockType,
	const char *buf, int *index)
{
	char write[FW_COL_COUNT];
	char ack;
	int i, r;
	int idx = 0;
	int ret = 0;

	switch (blockType) {
	case PAGE_HEAD:
		for (i = 0; i < FW_PAGE_SIZE_HEAD_DATA / 2; i++) {
			write[i] = (ascii_to_hex(*(buf + idx)) << 4) |
				ascii_to_hex(*(buf + idx + 1));
			idx += 2;
		}
		idx += FW_PAGE_SIZE_EOL;
		write_data(i2c, write, FW_PAGE_SIZE_HEAD_DATA / 2);
		/*Use sleep will occure timing issue make fw upgrade fail*/
		/*usleep_range(1000, 5000);*/
		mdelay(2);
		read_data(i2c, &ack, sizeof(ack));
		/*usleep_range(1000, 5000);*/
		mdelay(2);
		if (ack != 0x20) {
			printk(KERN_ERR "write head ak=%02x\n", ack);
			ret = 1;
		}
		break;
	case PAGE_BODY:
		/*first row*/
		for (i = 0; i < FW_PAGE_SIZE_BODY_ROW1_DATA / 2; i++) {
			write[i] = (ascii_to_hex(*(buf + idx)) << 4) |
				ascii_to_hex(*(buf + idx + 1));
			idx += 2;
		}
		idx += FW_PAGE_SIZE_EOL;

		write_data(i2c, write, FW_PAGE_SIZE_BODY_ROW1_DATA / 2);
		/*Use sleep will occure timing issue make fw upgrade fail*/
		/*usleep_range(10000, 15000);*/
		mdelay(12);
		/*2nd to 5th row*/
		for (r = 2; r <= 5; r++) {
			for (i = 0; i < FW_PAGE_SIZE_BODY_ROW2_5_DATA / 2;
				i++) {
				write[i] = (ascii_to_hex(*(buf + idx)) << 4)
					| ascii_to_hex(*(buf + idx + 1));
				idx += 2;
			}
			idx += FW_PAGE_SIZE_EOL;
			write_data(i2c, write,
				FW_PAGE_SIZE_BODY_ROW2_5_DATA / 2);
		}
		/*msleep(100);*/
		/*Use sleep will occure timing issue make fw upgrade fail*/
		mdelay(100);
		read_data(i2c, &ack, sizeof(ack));
		if (ack != 0x20) {
			printk(KERN_ERR "write body ak=%02x\n", ack);
			ret = 1;
		}
		break;

	case PAGE_TAIL:
		for (i = 0; i < FW_PAGE_SIZE_TAIL_DATA / 2; i++) {
			write[i] = (ascii_to_hex(*(buf + idx)) << 4)
				| ascii_to_hex(*(buf + idx + 1));
			idx += 2;
		}
		idx += FW_PAGE_SIZE_EOL;
		/*Use sleep will occure timing issue make fw upgrade fail*/
		/*usleep_range(10000, 15000);*/
		mdelay(12);
		write_data(i2c, write, FW_PAGE_SIZE_TAIL_DATA / 2);
		/*ssleep(1);*/
		mdelay(1000);
		read_data(i2c, &ack, sizeof(ack));
		if (ack != 0x21) {
			printk(KERN_ERR "write tail ak=%02x\n", ack);
			ret = 1;
		}
		break;
	default:
		printk(KERN_ERR "XD\n");
		break;
	}
	*index = idx;
	return ret;
}

static int update_firmware_imp(struct i2c_client *i2c, size_t size,
	const char *firmware_buf)
{
	int i = 0;
	int ret = 0;
	int index = 0;
	u8 ack;
	char *buf;
	struct cg7153am_platform_data *pd = i2c->dev.platform_data;

	buf = firmware_buf;
	LOG_FUNC();
	if (size != FILE_SIZE) {
		printk(KERN_ERR "mcu firmware size is not valid: %d\n", size);
		return -1;
	}
	/*make sure MCU is alive*/
	for (i = 0; i < MCU_WAKEUP_RETRY_MAX; i++) {
		ret = read_data(i2c, &ack, sizeof(ack));
		if (ret < 0) {
				LOG_MSG("%s:wakeup mcu fail,ack=%#x,rty=%d\n",
					__func__, ack, i);
				wakeup_mcu_imp(pd);
		} else {
				LOG_MSG("%s:wakeup mcu ok, ack=%#x, retry=%d\n",
					__func__, ack, i);
				break;
		}
	}
	ret = 0;
	for (i = 0; i < FW_PAGE_COUNT; i++) {
		if (i == 0)
			ret += write_firmware_page(i2c, PAGE_HEAD, buf, &index);
		else if (i == FW_PAGE_COUNT - 1)
			ret += write_firmware_page(i2c, PAGE_TAIL, buf, &index);
		else
			ret += write_firmware_page(i2c, PAGE_BODY, buf, &index);
		buf += index;
	}

	if (ret == 0)
		printk(KERN_INFO "mcu firmware is upgraded successfully\n");
	else
		printk(KERN_ERR "mcu firmware upgrade fail\n");

	return ret;
}

#define MCU_FIRMWARE "CY8C20324.i2c"
static int update_firmware(struct i2c_client *i2c)
{
	const struct firmware *fw_entry;
	char *mcu_fw = MCU_FIRMWARE;
	char event[32] = {0};
	char *envp[] = {event, NULL};
	int ret = -1;

	LOG_FUNC();

	switch (atomic_read (&ota_status)) {
	case OTA_STATUS_FAIL:
	case OTA_STATUS_UPGRADING:
		snprintf(event, sizeof(event),
			"EVENT=FIRMWARE_UPDATE_FAIL");
		kobject_uevent_env(&i2c->dev.kobj, KOBJ_CHANGE, envp);
		return -EFAULT;
	case OTA_STATUS_IDLE:
	case OTA_STATUS_COMPLETE:
	default:
		break;
	}

	atomic_set(&ota_status, OTA_STATUS_UPGRADING);

	ret = request_firmware(&fw_entry, mcu_fw, &i2c->dev);

	if (ret == 0) {
		printk(KERN_ERR "%s:prepare to update firmeware: size:%d\n",
			__func__, fw_entry->size);
		ret = update_firmware_imp(i2c, fw_entry->size, fw_entry->data);
		release_firmware(fw_entry);
		if (ret == 0) {
			atomic_set(&ota_status, OTA_STATUS_COMPLETE);
			snprintf(event, sizeof(event),
				"EVENT=FIRMWARE_UPDATE_SUCCESS");
			kobject_uevent_env(&i2c->dev.kobj, KOBJ_CHANGE, envp);
		} else {
			atomic_set(&ota_status, OTA_STATUS_FAIL);
			snprintf(event, sizeof(event),
				"EVENT=FIRMWARE_UPDATE_FAIL");
			kobject_uevent_env(&i2c->dev.kobj, KOBJ_CHANGE, envp);
		}
		return 0;
	}
	LOG_MSG("%s: request_firmware ret = %d!\n", __func__, ret);
	atomic_set(&ota_status, OTA_STATUS_FAIL);
	snprintf(event, sizeof(event), "EVENT=FIRMWARE_UPDATE_FAIL");
	kobject_uevent_env(&i2c->dev.kobj, KOBJ_CHANGE, envp);
	return -EFAULT;
}

static void ota_work_func(struct work_struct *work)
{
	struct ota_data *ota =
		container_of(work, struct ota_data, work);

	LOG_FUNC();
	mutex_lock(&ota->lock);
	update_firmware(ota->this_client);
	mutex_unlock(&ota->lock);
}

static ssize_t request_ota_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	ssize_t	status;
	struct i2c_client *i2c = to_i2c_client(dev);
	struct ota_data *ota = i2c_get_clientdata(i2c);

	LOG_FUNC();
	LOG_MSG("%s: i2c:%x\n", __func__, (unsigned int)i2c);

	if (sysfs_streq(buf, "1"))
		status = schedule_work(&ota->work);
	else
		status = -EINVAL;

	return status ? : size;
}

static ssize_t show_ota_status(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int i = 0;
	i += snprintf(buf + i, PAGE_SIZE - i, "%d\n", atomic_read(&ota_status));
	return i;
}

static /* const */ DEVICE_ATTR(request_ota, 0644, 0, request_ota_store);
static /* const */ DEVICE_ATTR(ota_status, S_IRUGO , show_ota_status, 0);


static void power_on_reset(struct i2c_client *i2c)
{
	printk(KERN_INFO "%s\n", __func__);
}

static irqreturn_t cg7153am_interrupt_isr(int irq, void *devdata)
{
	return IRQ_HANDLED;
}


static __devinit int cg7153am_probe(struct i2c_client *i2c,
				      const struct i2c_device_id *id)
{
	int ret;
	struct cg7153am_platform_data *pd = i2c->dev.platform_data;
	struct ota_data *ota;
	struct cg7153am_drvdata *devdata;
	LOG_FUNC();

	if (i2c == 0 || i2c->dev.platform_data == 0) {
		printk(KERN_ERR "%s:invalid param\n", __func__);
		return -EINVAL;
	}

	power_on_reset(i2c);
	wakeup_mcu_imp(pd);

	if (pd->is_fw_update_mode) {
		ota = kzalloc(sizeof(struct ota_data), GFP_KERNEL);
		if (!ota) {
			dev_err(&i2c->dev,
				"failed to allocate memory for module data\n");
			return -ENOMEM;
		}
		ota->this_client = i2c;
		ota->pd = i2c->dev.platform_data;
		mutex_init(&ota->lock);
		INIT_WORK(&ota->work, ota_work_func);
		i2c_set_clientdata(i2c, ota);

		/*create attributes for firmware upgrade*/
		ret = device_create_file(&i2c->dev, &dev_attr_request_ota);

		ret = device_create_file(&i2c->dev, &dev_attr_ota_status);

		atomic_set(&ota_status, OTA_STATUS_IDLE);

		return ret;
	}

	devdata = kzalloc(sizeof(struct cg7153am_drvdata), GFP_KERNEL);
	if (!devdata) {
		dev_err(&i2c->dev,
			"failed to allocate memory for module data\n");
		return -ENOMEM;
	}

	devdata->fw_version = get_version(i2c);
	i2c_set_clientdata(i2c, devdata);

	cg7153am_misc_device.parent = &i2c->dev;
	ret = misc_register(&cg7153am_misc_device);
	if (ret) {
		pr_err("%s: misc_register() register failed\n", __func__);
		goto fail0;
	}
	sysfs_bin_attr_init(&devdata->fw_version_attr);
	devdata->fw_version_attr.attr.name = "fw_version";
	devdata->fw_version_attr.attr.mode = S_IRUGO;
	devdata->fw_version_attr.size =  MCU_FW_STR_LEN;
	devdata->fw_version_attr.read = cg7153am_attr_get_version;

	ret = sysfs_create_bin_file(&i2c->dev.kobj, &devdata->fw_version_attr);
	if (ret) {
		sysfs_remove_bin_file(&i2c->dev.kobj,
			&devdata->fw_version_attr);
		goto fail1;
	}

	ret = request_irq(gpio_to_irq(pd->gpio_mcu_int), cg7153am_interrupt_isr,
		IRQF_SHARED|IRQF_TRIGGER_HIGH, "cg7153am_int", devdata);
	if (ret < 0) {
		dev_err(&i2c->dev, "%s: Failed to request_irq(): gpio=%d\n",
		 __func__, pd->gpio_mcu_int);
		goto fail1;
	}

	device_init_wakeup(&i2c->dev, 1);

	if (device_may_wakeup(&i2c->dev))
		enable_irq_wake(gpio_to_irq(pd->gpio_mcu_int));

	create_sub_devs(&i2c->dev);
	printk(KERN_ERR "%s:mcu version:0x%x\n", __func__,
		devdata->fw_version);

	WARN_ON(enter_mcu_mode(i2c, pd , 1) < 0);
	return ret;

fail1:
	misc_deregister(&cg7153am_misc_device);
fail0:
	i2c_set_clientdata(i2c, 0);
	kfree(devdata);
	return ret;
}

static __devexit int cg7153am_remove(struct i2c_client *i2c)
{
	struct cg7153am_platform_data *pd = i2c->dev.platform_data;
	struct cg7153am_drvdata *devdata = i2c_get_clientdata(i2c);

	LOG_FUNC();
	sysfs_remove_bin_file(&i2c->dev.kobj,  &devdata->fw_version_attr);
	misc_deregister(&cg7153am_misc_device);
	destroy_sub_devs(&i2c->dev);

	device_init_wakeup(&i2c->dev, 0);
	free_irq(gpio_to_irq(pd->gpio_mcu_int), devdata);

	kfree(devdata);
	devdata = NULL;
	i2c_set_clientdata(i2c, 0);

	return 0;
}

static int cg7153am_suspend(struct i2c_client *i2c, pm_message_t mesg)
{
	LOG_FUNC();

	WARN_ON(enter_mcu_mode(i2c, i2c->dev.platform_data, 0) < 0);
	return 0;
}

static int cg7153am_resume(struct i2c_client *i2c)
{
	LOG_FUNC();

	WARN_ON(enter_mcu_mode(i2c, i2c->dev.platform_data, 1) < 0);
	return 0;
}

static const struct i2c_device_id i2c_id[] = {
	{ "cg7153am", 0 },
	{}
};

static struct i2c_driver cg7153am_driver = {
	.driver = {
		.name = "cg7153am",
		.owner = THIS_MODULE,
	},
	.probe		= cg7153am_probe,
	.remove		= __devexit_p(cg7153am_remove),
	.suspend	= cg7153am_suspend,
	.resume		= cg7153am_resume,
	.id_table	= i2c_id,
};

static int __init cg7153am_module_init(void)
{
	LOG_FUNC();
	return i2c_add_driver(&cg7153am_driver);
}
module_init(cg7153am_module_init);

static void __exit cg7153am_module_exit(void)
{
	LOG_FUNC();
	i2c_del_driver(&cg7153am_driver);
}
module_exit(cg7153am_module_exit);


MODULE_AUTHOR("Andrew Hsiao <Andrew_Hsiao@pegatroncorp.com>");
MODULE_DESCRIPTION("CG7153AM MCU Driver");
MODULE_LICENSE("GPL");
