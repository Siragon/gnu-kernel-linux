/*
 * drivers/power/cg7153am_battery.c
 *
 * Gas Gauge driver for TI's CG7153AM
 *
 * Copyright (c) 2010, NVIDIA Corporation.
 * Copyright (c) 2011, PEGATRON Corporation.
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

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/err.h>
#include <linux/debugfs.h>
#include <linux/power_supply.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/i2c/cg7153am_power.h>
#include <linux/irq.h>
#include <mach/gpio.h>
#include <linux/i2c/cg7153am.h>
#include <linux/device.h>
#include <linux/platform_device.h>

#include <linux/kthread.h>

enum {
	PDA_PSY_OFFLINE = 0,
	PDA_PSY_ONLINE = 1,
	PDA_PSY_TO_CHANGE,
};

#define BATTERY_RETRY_MAX	5

#define BATTERY_MANUFACTURER_SIZE	12
#define BATTERY_NAME_SIZE		8

#define TEMPERATURE_K_TO_C -2731

/* manufacturer access defines */
#define MANUFACTURER_ACCESS_STATUS	0x0006
#define MANUFACTURER_ACCESS_SLEEP	0x0011

/* battery status value bits */
#define BATTERY_INIT_DONE	0x80
#define BATTERY_DISCHARGING	0x40
#define BATTERY_FULL_CHARGED	0x20
#define BATTERY_FULL_DISCHARGED	0x10
#define KMAXTEMPERATURE 600
#define KMINTEMPERATURE -200
#define KRECHARGETEMPH  400
#define KRECHARGETEMPL  50

#define POLLING_TIME_PER_INTERVAL                25

#define CG7153AM_REG_BATTERY_POLLLING_INTERVAL   0x10
#define CG7153AM_REG_BATTERY_CELL1_VOL		 0x98
#define CG7153AM_REG_BATTERY_CELL2_VOL		 0x9A
#define CG7153AM_REG_BATTERY_MANUFACTURER_ACCESS 0xA0
#define CG7153AM_REG_BATTERY_TEMPERATURE         0xA2
#define CG7153AM_REG_BATTERY_VOLTAGE             0xA4
#define CG7153AM_REG_BATTERY_CURRENT             0xA6
#define CG7153AM_REG_BATTERY_CAPACITY            0xA8
#define CG7153AM_REG_BATTERY_CHARGING_CURRENT    0xAA
#define CG7153AM_REG_BATTERY_CHARGING_VOLTAGE    0xAC
#define CG7153AM_REG_BATTERY_STATUS              0xAE
#define CG7153AM_REG_BATTERY_REMAIN_CAPACITY     0xB0
#define CG7153AM_REG_BATTERY_FULL_CAPACITY       0xB2
#define CG7153AM_REG_BATTERY_SHIP_MOD		 0x24
#define CG7153AM_BATTERY_INFO_BLOCK_START_ADDR \
		CG7153AM_REG_BATTERY_TEMPERATURE
#define CG7153AM_BATTERY_INFO_BLOCK_SIZE \
		(CG7153AM_REG_BATTERY_FULL_CAPACITY - \
		CG7153AM_REG_BATTERY_MANUFACTURER_ACCESS)

#ifdef DEBUG
	#define BATT_MSG(x...) pr_err("<BATT>" x)
	#define LOG_FUNC() printk(KERN_ERR "<%s>\n", __func__)
#else
	#define BATT_MSG(x...) do {} while (0)
	#define LOG_FUNC() do {} while (0)
#endif
#define TMP_RESUME(suspended, temp_max, data) \
	(((suspended == true) && (data > temp_max)) ? 1 : 0)
#define VOL_CAP_RESUME(suspended, vol_max, vol_min, data) \
	(((suspended == true) && ((data > vol_max) || \
	(data < vol_min))) ? 1 : 0)
#define DIFF_CMP(suspended, diff, threshold, tolerence, count) \
	(((diff > threshold) && (count < tolerence) && (!suspended)) \
	? 1 : 0)

#define TMP_CMP(data, max_tolerence, min_tolerence) \
(((data > max_tolerence) || (data < min_tolerence)) ? 1 : 0)

#define absval(a) (((a) > 0) ? (a) : -(a))

#define CG7153AM_DATA(_psp, _addr, _min_value, _max_value)	\
{							\
		.psp = POWER_SUPPLY_PROP_##_psp,\
		.addr = _addr,					\
		.min_value = _min_value,			\
		.max_value = _max_value,			\
}

#define CG7153AM_DATAS(_addr, _min_value, _max_value)	\
{							\
		.addr = _addr,					\
		.min_value = _min_value,			\
		.max_value = _max_value,			\
}

#define TOTAL_BATTEREY_CAPACITY  3300
#define RESERVED_BATTEREY_CAPACITY  99 /*reserve capacity = 3300*3%*/
#define TMP_MAX_TOLERENCE 1200 /*It is used to ignore abnormal data*/
#define TMP_MIN_TOLERENCE -400 /*It is used to ignore abnormal data*/
#define TMP_MAX 650 /*sepc defined max temperature at 65 degree */
#define VOL_MAX 8400 /*sepc defined max voltage at 8400mV*/
#define VOL_MIN 6417 /*sepc defined min voltage at 6417mV*/
#define CAP_MAX 100 /*spec defined max capacity at 100% */
#define CAP_MIN 1 /*spec defined min capacity at 1%*/
#define CUR_MAX 2135 /*spec defined max charging current at 2135mA */
#define CUR_MIN -1500 /*spec defined min charging current at -1500mA */
#define TMP_THRESHOLD 10 /*The change of temp doesn't surpass 1 degree in 3s*/
#define VOL_THREADHOLD 1000 /*The change of vol doesn't surpass 1v in 3s*/
#define CAP_THREADHOLD 2 /*The change of cap doesn't surprass 2% in 3s*/
#define POLL_INIT 2
#define FAULT_TOLERENCE 2
#define LOW_CAP_RESERVE 2 /*Accurately it is 3% for low boarder*/
#define HIGH_CAP_RESERVE 5
#define FW_VERSIOR_BORDER 79
#define FW_VERSIOR_BORDER_GAUGE_FW 86
#define FW_VERSIOR_BORDER_SHIP_FUNC 96
#define GAUGE_ATTRIBUTE_NON_SHIP 1
#define GAUGE_ATTRIBUTE_SIZE 3
#define SHIP_ENABLE 1
#define IMBALANCE_VOL 500

static atomic_t poll_interval;
static atomic_t debug_enable;
static int cg7153am_polling_thread(void *data);

static enum power_supply_property cg7153am_properties[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	/*POWER_SUPPLY_PROP_CYCLE_COUNT,*/
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_TEMP,
};

static enum power_supply_property cg7153am_ac_properties[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static char *power_supplied_to[] = {
	"battery",
};

static enum power_supply_property cg7153am_properties_sequence[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_TECHNOLOGY,
};

#define PROPERTY_SEQUENCE_SIZE ARRAY_SIZE(cg7153am_properties_sequence)
#define GET_PROPERTY_SEQUENCE_START(psp) \
	((psp == cg7153am_properties_sequence[0]) ? 1 : 0)
#define GET_PROPERTY_SEQUENCE_END(psp) \
	((psp == cg7153am_properties_sequence[PROPERTY_SEQUENCE_SIZE-1]) \
	? 1 : 0)

static struct cg7153am_status_backup
{
	bool ac_status;
	int batt_is_present;
	int batt_voltage;
	int batt_capacity;
	int batt_temperature;
	int batt_health;
	int register_status;
	int batt_status;
} batt_status_backup;

static struct cg7153am_device_info {
	struct i2c_client	*client;
	struct CG7153AM_power_pdata *charger_pdata;
	struct task_struct *polling_thread;
	wait_queue_head_t wait_q_update;
	int batt_voltage;
	int batt_capacity;
	int register_remain_capacity;
	int register_full_capacity;
	int fw_version;
	int batt_temperature;
	int charge_current;
	int charge_voltage;
	int register_status;
	int batt_status;
	int batt_health;
	int batt_is_present;
	bool state_changed;
	unsigned long update_time;
	atomic_t is_in_suspend;
	bool suspended;
	unsigned long ac_latest_update_time;
	bool ac_status;
	int sequence_index;
	int tmp_count;
	int vol_count;
	int cur_count;
	int cap_count;
	int data_id;
	atomic_t backup_lock;
	u16 gauge_cell_vol[2];
	u8 gauge_fw_ascii[2];
	struct wake_lock wake_lock;
	struct work_struct work;
} *cg7153am_device;

static int cg7153am_get_property(struct power_supply *psy,
	enum power_supply_property psp, union power_supply_propval *val);
static int cg7153am_ac_get_property(struct power_supply *psy,
	enum power_supply_property psp, union power_supply_propval *val);
static ssize_t cg7153am_battery_show_property(struct device *dev,
					  struct device_attribute *attr,
					  char *buf);
static ssize_t cg7153am_battery_store_property(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count);
static void cg7153am_battery_status_update(struct cg7153am_device_info *info);

enum supply_type {
	SUPPLY_TYPE_BATTERY = 0,
	SUPPLY_TYPE_AC,
};

static struct power_supply cg7153am_supply[] = {
	[SUPPLY_TYPE_BATTERY] = {
		.name		= "battery",
		.type		= POWER_SUPPLY_TYPE_BATTERY,
		.properties	= cg7153am_properties,
		.num_properties	= ARRAY_SIZE(cg7153am_properties),
		.get_property	= cg7153am_get_property,
	},
	[SUPPLY_TYPE_AC] = {
		.name		= "ac",
		.type		= POWER_SUPPLY_TYPE_MAINS,
		.supplied_to	= power_supplied_to,
		.num_supplicants = ARRAY_SIZE(power_supplied_to),
		.properties	= cg7153am_ac_properties,
		.num_properties = ARRAY_SIZE(cg7153am_ac_properties),
		.get_property	= cg7153am_ac_get_property,
	},
};

static s32 i2c_smbus_read_word_data_retry(struct i2c_client *client, u8 command)
{
	int retry = 0;
	s32 res = 0;
	wakeup_mcu(client);
	for (retry = 0; retry < BATTERY_RETRY_MAX; retry++) {
		res = i2c_smbus_read_word_data(client, command);
		if (res >= 0)
			break;
		printk(KERN_ERR "%s-retry:%d\n", __func__, retry);
	}

	suspend_mcu(client);
	return res;
}

static s32 i2c_smbus_write_word_data_retry(struct i2c_client *client,
	u8 command, u16 value)
{
	int retry = 0;
	s32 res = 0;
	wakeup_mcu(client);
	for (retry = 0; retry < BATTERY_RETRY_MAX; retry++) {
		res = i2c_smbus_write_word_data(client, command, value);
		if (res >= 0)
			break;
		printk(KERN_ERR "%s-retry:%d\n", __func__, retry);
	}
	suspend_mcu(client);
	return res;
}

static s32 i2c_smbus_read_i2c_block_data_retry(struct i2c_client *client,
	u8 command, u8 length, u8 *value)
{
	s32 ret = 0;
	int retry = 0;
	wakeup_mcu(client);
	for (retry = 0; retry < BATTERY_RETRY_MAX; retry++) {
		ret = i2c_smbus_read_i2c_block_data(
			client, command, length, value);
		if (ret >= 0)
			break;
	}
	suspend_mcu(client);
	return ret;
}

static void check_property_sequence(enum power_supply_property batt_prop)
{
	BATT_MSG("+%s\n", __func__);

	if (batt_prop == cg7153am_properties_sequence[
		cg7153am_device->sequence_index]) {
		BATT_MSG("%s:sequence in : %d\n", __func__,
			cg7153am_device->sequence_index);
		cg7153am_device->sequence_index++;

		if (GET_PROPERTY_SEQUENCE_START(batt_prop))	{
			atomic_set(&cg7153am_device->backup_lock, 1);
		} else if (GET_PROPERTY_SEQUENCE_END(batt_prop)) {
			atomic_set(&cg7153am_device->backup_lock, 0);
			cg7153am_device->sequence_index = 0;
		}
	} else {
		BATT_MSG("%s:fail sequence : %d\n", __func__,
			cg7153am_device->sequence_index);
		atomic_set(&cg7153am_device->backup_lock, 0);
		cg7153am_device->sequence_index = 0;

		if (GET_PROPERTY_SEQUENCE_START(batt_prop))	{
			atomic_set(&cg7153am_device->backup_lock, 1);
			cg7153am_device->sequence_index++;
		}
	}

	BATT_MSG("-%s\n", __func__);
}

static int cg7153am_get_ac_status(void)
{
	return cg7153am_device->charger_pdata->is_ac_online();
}

static int cg7153am_ac_get_property(struct power_supply *psy,
	enum power_supply_property psp,
	union power_supply_propval *val)
{
	BATT_MSG("+%s: psp:%d\n", __func__, psp);

	check_property_sequence(psp);

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = batt_status_backup.ac_status;
		break;
	default:
		dev_err(&cg7153am_device->client->dev,
			"%s: INVALID property\n", __func__);
		return -EINVAL;
	}

	BATT_MSG("-%s: psp:%d, val:%d\n", __func__, psp, val->intval);
	return 0;
}

static int get_psy_prop_status(int ac_status, int capacity,
	int gauge_status, int charge_current, int batt_is_present)
{
	/* gauge status:
	 * bit7 bit6 bit5 bit4 bit3 bit2 bit1 bit0
	 * INIT DSG  FC   FD   EC3  EC2  EC1  EC0
	 * INIT: 1
	 * DSG : 1 = discharging
	 * FC  : 1 = full charged
	 * FD  : 1 = full discharged
	 * EC3 EC2 EC1 EC0: current battery is always read 0111
	 *    0000: OK
	 *    0001: BUSY
	 *    0010: Reserved
	 *    0011: Unsupported
	 *    0100: AccessDenied
	 *    0101: Over/Underflow
	 *    0110: Badsize
	 *    0111: UnknownError gauge_status*/
	if (!batt_is_present)
		return POWER_SUPPLY_STATUS_UNKNOWN;
	switch (gauge_status & 0x60) {
	case 0x40:
		if (ac_status == 1) {
			return capacity == 100 ? POWER_SUPPLY_STATUS_FULL :
				POWER_SUPPLY_STATUS_CHARGING;
		} else {
			return POWER_SUPPLY_STATUS_DISCHARGING;
		}
	case 0x20:
		return POWER_SUPPLY_STATUS_FULL;
	default:
		if (ac_status) {
			return capacity == 100 ? POWER_SUPPLY_STATUS_FULL :
				POWER_SUPPLY_STATUS_CHARGING;
		} else {
			/* if we have external battery, ex: dock-bat
			 * and power is provided by external,
			 * so the status is not charging */
			return POWER_SUPPLY_STATUS_NOT_CHARGING;
		}
	}
}

static int cg7153am_get_property(struct power_supply *psy,
	enum power_supply_property psp,
	union power_supply_propval *val)
{
	BATT_MSG("+%s: psp:%d\n", __func__, psp);

	check_property_sequence(psp);

	switch (psp) {
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = batt_status_backup.batt_is_present;
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = batt_status_backup.batt_health;
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		break;

	case POWER_SUPPLY_PROP_CAPACITY:
		if (batt_status_backup.batt_status == POWER_SUPPLY_STATUS_FULL)
			val->intval = 100;
		else
			val->intval = batt_status_backup.batt_capacity;
		break;
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = batt_status_backup.batt_status;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = batt_status_backup.batt_voltage * 1000;
		/* in micro voltage for framework */
				break;
	case POWER_SUPPLY_PROP_TEMP:
		val->intval = batt_status_backup.batt_temperature;
		break;
	default:
		dev_err(&cg7153am_device->client->dev,
				"%s: INVALID property\n", __func__);
		return -EINVAL;
	}
	BATT_MSG("-%s: psp:%d, value:%d\n", __func__, psp, val->intval);
	return 0;
}


#define CG7153AM_BATTERY_ATTR(_name)	\
{	\
	.attr = { .name = #_name, .mode = S_IRUGO|S_IWUSR,}, \
	.show = cg7153am_battery_show_property, \
	.store = cg7153am_battery_store_property, \
}

#define CG7153AM_BATTERY_ATTR_ATS(_name, _store)	\
{	\
	.attr = { .name = #_name, .mode = S_IRUGO | S_IWUSR,}, \
	.show = cg7153am_battery_show_property, \
	.store = _store, \
}


static struct device_attribute cg7153am_battery_attrs[] = {
/*	CG7153AM_BATTERY_ATTR(charging_enabled),*/
	CG7153AM_BATTERY_ATTR(charge_current),
	CG7153AM_BATTERY_ATTR(lowbatt),
	CG7153AM_BATTERY_ATTR(poll_int),
	CG7153AM_BATTERY_ATTR(debug_en),
	CG7153AM_BATTERY_ATTR(gauge_fw),
	CG7153AM_BATTERY_ATTR(cell_vol),
	CG7153AM_BATTERY_ATTR(ship_mod),
};
enum {
/*	CHARGING_ENABLED = 0,*/
	CHARGE_CURRENT = 0,
	LOWBATT,
	POLLING_INTERVAL,
	DEBUG_ENABLE,
	GAUGE_FW,
	CELL_VOL,
	SHIP_MOD,
};


static int cg7153am_battery_create_attrs(struct device *dev)
{
	int i, rc, arry_size;
	printk(KERN_INFO "%s\n", __func__);
	if (cg7153am_device->fw_version >= FW_VERSIOR_BORDER_SHIP_FUNC)
		arry_size = ARRAY_SIZE(cg7153am_battery_attrs);
	else if (cg7153am_device->fw_version >= FW_VERSIOR_BORDER_GAUGE_FW)
		arry_size = ARRAY_SIZE(cg7153am_battery_attrs)
		- GAUGE_ATTRIBUTE_NON_SHIP;
	else
		arry_size = ARRAY_SIZE(cg7153am_battery_attrs)
		- GAUGE_ATTRIBUTE_SIZE;
	for (i = 0 ; i < arry_size ; i++) {
		rc = device_create_file(dev, &cg7153am_battery_attrs[i]);
		if (rc)
			goto cg7153am_attrs_failed;
	}
	goto succeed;
cg7153am_attrs_failed:
	while (i--)
		device_remove_file(dev, &cg7153am_battery_attrs[i]);
succeed:
	return rc;
}

static void cg7153am_battery_remove_attrs(struct device *dev)
{
	int i;
	printk(KERN_INFO "%s\n", __func__);
	for (i = 0; i < ARRAY_SIZE(cg7153am_battery_attrs); i++)
		device_remove_file(dev, &cg7153am_battery_attrs[i]);
}

/* in: value: polling interval in ms
 * out: the nearest available interval   */
static int cg7153am_battery_set_polling_interval(int value)
{

	/*cg7153am_device->fw_version =
		cg7153am_version_get(cg7153am_device->client);*/

	if (cg7153am_device->fw_version < CG7153AM_PVT_FW_VERSION)
		return -1;
	value = (int)(value / POLLING_TIME_PER_INTERVAL);
	value = value > 0 ? value - 1 : 0;

	i2c_smbus_write_byte_data(cg7153am_device->client,
		CG7153AM_REG_BATTERY_POLLLING_INTERVAL, value);

	value = (value + 1) * POLLING_TIME_PER_INTERVAL;

	atomic_set(&poll_interval, value);

	return value;
}

static ssize_t cg7153am_battery_store_property(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	int value;

	printk(KERN_INFO "%s\n", __func__);
	const ptrdiff_t off = attr - cg7153am_battery_attrs;
	switch (off) {
	case POLLING_INTERVAL:
		if (sscanf(buf, "%d\n", &value) == 1)
			cg7153am_battery_set_polling_interval(value);
		break;
	case DEBUG_ENABLE:
		if (sscanf(buf, "%d\n", &value) == 1)
			atomic_set(&debug_enable, value);
		break;
	case SHIP_MOD:
		if (sscanf(buf, "%d\n", &value) == 1)
			i2c_smbus_write_word_data_retry(cg7153am_device->client,
			CG7153AM_REG_BATTERY_SHIP_MOD, SHIP_ENABLE);
		break;
	default:
		break;
	}
	return count;
}

static ssize_t cg7153am_battery_show_property(struct device *dev,
					  struct device_attribute *attr,
					  char *buf)
{
	int i = 0;
	u16 res = 0;
	printk(KERN_INFO "%s\n", __func__);
	const ptrdiff_t off = attr - cg7153am_battery_attrs;
	switch (off) {
/*
	case CHARGING_ENABLED:
		i += scnprintf(buf + i, PAGE_SIZE - i, "un-supported\n");
		break;
*/
	case CHARGE_CURRENT:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%dmA\n",
		cg7153am_device->charge_current);
		break;
/*
	case LOWBATT:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
			!gpio_get_value(
				cg7153am_device->charger_pdata->lowbatt_gpio));
		break;
*/
	case POLLING_INTERVAL:
		i += scnprintf(buf + i, PAGE_SIZE - i , "%d\n",
			atomic_read(&poll_interval));
		break;
	case DEBUG_ENABLE:
		i += scnprintf(buf + i, PAGE_SIZE - i , "%d\n",
			atomic_read(&debug_enable));
		break;
	case GAUGE_FW:
		i += scnprintf(buf + i, PAGE_SIZE - i , "V%c.%c\n",
			cg7153am_device->gauge_fw_ascii[0],
			cg7153am_device->gauge_fw_ascii[1]);
		break;
	case CELL_VOL:
		i += scnprintf(buf + i, PAGE_SIZE - i ,
			"cell1:%dmV cell2:%dmV\n",
			cg7153am_device->gauge_cell_vol[0],
			cg7153am_device->gauge_cell_vol[1]);
		break;
	case SHIP_MOD:
		res = i2c_smbus_read_word_data_retry(cg7153am_device->client,
			CG7153AM_REG_BATTERY_SHIP_MOD);
		i += scnprintf(buf + i, PAGE_SIZE - i ,
			"SHIP_FLAG: %d\n",
			res);
		break;
	default:
		i = -EINVAL;
	}
	printk(KERN_ERR "%s:attr_off:%d, value:%s\n", __func__, off, buf);
	return i;
}
static int cg7153am_capacity_calculate(int remain, int full)
{
	int remap_capacity = 0;
	int remap_temp = 0;
	int reserve_temp = 0;
	int low_reserve_cap = 0;
	int high_reserve_cap = 0;
	int high_border = 0;
	low_reserve_cap = (full * LOW_CAP_RESERVE) / CAP_MAX;
	high_reserve_cap = (full * HIGH_CAP_RESERVE) / CAP_MAX;
	high_border = full - high_reserve_cap;
	reserve_temp = low_reserve_cap + high_reserve_cap;
	remap_temp = full - reserve_temp;
	if (remain < low_reserve_cap)
		remap_capacity = 0;
	else if (remain >= high_border)
		remap_capacity = CAP_MAX;
	else
		remap_capacity = ((remain - low_reserve_cap) * 100) / remap_temp;
	return remap_capacity;
}

static int cg7153am_data_filter(int new_data, int old_data, int id)
{
	bool data_status = true;
	bool temp1 = true;
	bool temp2 = true;
	bool temp3 = false;
	int diff = abs(new_data - old_data);
	switch (id) {
	case CG7153AM_REG_BATTERY_TEMPERATURE:
		temp1 = TMP_RESUME(cg7153am_device->suspended,
			TMP_MAX, new_data);
		temp2 = DIFF_CMP(cg7153am_device->suspended,
			diff, TMP_THRESHOLD,
			FAULT_TOLERENCE, cg7153am_device->tmp_count);
		temp3 = TMP_CMP(new_data, TMP_MAX_TOLERENCE, TMP_MIN_TOLERENCE);
		if (temp3 == true) {
			printk(KERN_INFO "%s :: Temperature abnormal\n",
				__func__);
			cg7153am_device->data_id = 0;
			data_status = false;
		} else if (temp1 || temp2) {
			cg7153am_device->data_id =
				CG7153AM_REG_BATTERY_TEMPERATURE;
			cg7153am_device->tmp_count++;
			data_status = false;
		} else {
			cg7153am_device->tmp_count = 0;
			data_status = true;
		}
		break;
	case CG7153AM_REG_BATTERY_VOLTAGE:
		temp1 = VOL_CAP_RESUME(cg7153am_device->suspended,
			VOL_MAX, VOL_MIN, new_data);
		temp2 = DIFF_CMP(cg7153am_device->suspended,
			diff, VOL_THREADHOLD,
			FAULT_TOLERENCE, cg7153am_device->vol_count);
		if (temp1 || temp2) {
			cg7153am_device->data_id =
				CG7153AM_REG_BATTERY_VOLTAGE;
			cg7153am_device->vol_count++;
			data_status = false;
		} else {
			cg7153am_device->vol_count = 0;
			data_status = true;
		}
		break;
	case CG7153AM_REG_BATTERY_CURRENT:
		if (((new_data > CUR_MAX) || (new_data < CUR_MIN)) &&
			(cg7153am_device->cur_count < FAULT_TOLERENCE)) {
			cg7153am_device->data_id =
				CG7153AM_REG_BATTERY_CURRENT;
			cg7153am_device->cur_count++;
			data_status = false;
		} else {
			cg7153am_device->cur_count = 0;
			data_status = true;
		}
		break;
	case CG7153AM_REG_BATTERY_CAPACITY:
		temp1 = VOL_CAP_RESUME(cg7153am_device->suspended,
			CAP_MAX, CAP_MIN, new_data);
		temp2 = DIFF_CMP(cg7153am_device->suspended,
			diff, CAP_THREADHOLD,
			FAULT_TOLERENCE, cg7153am_device->cap_count);
		if (temp1 || temp2) {
			cg7153am_device->data_id =
				CG7153AM_REG_BATTERY_CAPACITY;
			cg7153am_device->cap_count++;
			data_status = false;
		} else {
			cg7153am_device->cap_count = 0;
			data_status = true;
		}
		break;
	default:
		break;
	}
	if ((cg7153am_device->data_id == id) && (!data_status)) {
		cg7153am_battery_set_polling_interval(
		cg7153am_device->charger_pdata->interval_faster);
	 } else if ((cg7153am_device->data_id == id) && (data_status)) {
		cg7153am_device->data_id = 0;
		cg7153am_battery_set_polling_interval(
		cg7153am_device->charger_pdata->interval_slow);
	 }
	 return (data_status == true) ? new_data : old_data;
}

static int cg7153am_battery_is_present(int volt, int temp)
{
	/* now we only check voltage and temperature if there are
	 * left for future implemetation if MCU firmware is upgraded*/
	return ((volt == 0) && (temp == TEMPERATURE_K_TO_C)) ? 0 : 1;
}

static int cg7153am_get_battery_health(void)
{
	/* left for future implementation if MCU firmware is upgraded */
	return POWER_SUPPLY_HEALTH_GOOD;
}

static void cg7153am_battery_status_backup(
	struct cg7153am_device_info *batt_info)
{
	BATT_MSG("+%s:\n", __func__);

	batt_status_backup.ac_status = batt_info->ac_status;
	batt_status_backup.batt_voltage = batt_info->batt_voltage;
	batt_status_backup.batt_temperature = batt_info->batt_temperature;
	batt_status_backup.batt_capacity = batt_info->batt_capacity;
	batt_status_backup.register_status = batt_info->register_status;
	batt_status_backup.batt_status = batt_info->batt_status;
	batt_status_backup.batt_is_present = batt_info->batt_is_present;
	batt_status_backup.batt_health = batt_info->batt_health;

	BATT_MSG("ac = %d, status = %d, volt = %d, temp = %d, capa = %d\n",
		batt_status_backup.ac_status, batt_status_backup.batt_status,
		batt_status_backup.batt_voltage,
		batt_status_backup.batt_temperature,
		batt_status_backup.batt_capacity);
	BATT_MSG("-%s:\n", __func__);
}

static void cg7153am_battery_status_update(struct cg7153am_device_info *info)
{
	BATT_MSG("+%s:\n", __func__);

	u8 block[CG7153AM_BATTERY_INFO_BLOCK_SIZE];
	int tmp, i;
	int temp_data;
	if (i2c_smbus_read_i2c_block_data_retry(cg7153am_device->client,
			CG7153AM_BATTERY_INFO_BLOCK_START_ADDR,
			CG7153AM_BATTERY_INFO_BLOCK_SIZE, block) > 0) {
		/* WARNING:
		 * You should make sure tmp is always less than BLOCK_SIZE */
		tmp = CG7153AM_REG_BATTERY_TEMPERATURE -
			CG7153AM_BATTERY_INFO_BLOCK_START_ADDR;
		/*info->batt_temperature = (int)(block[tmp + 1] << 8) +
			(int)block[tmp] + TEMPERATURE_K_TO_C;*/
		temp_data = (int)(block[tmp + 1] << 8) +
			(int)block[tmp] + TEMPERATURE_K_TO_C;
		info->batt_temperature = cg7153am_data_filter(temp_data,
			info->batt_temperature,
			CG7153AM_REG_BATTERY_TEMPERATURE);

		tmp = CG7153AM_REG_BATTERY_VOLTAGE -
			CG7153AM_BATTERY_INFO_BLOCK_START_ADDR;
		/*info->batt_voltage = (int)(block[tmp + 1] << 8)
			+ (int)block[tmp];*/
		temp_data = (int)(block[tmp + 1] << 8)
			+ (int)block[tmp];
		info->batt_voltage = cg7153am_data_filter(temp_data,
			info->batt_voltage,
			CG7153AM_REG_BATTERY_VOLTAGE);

		tmp = CG7153AM_REG_BATTERY_CURRENT -
			CG7153AM_BATTERY_INFO_BLOCK_START_ADDR;
		/*info->charge_current = (int)((s16)(block[tmp + 1] << 8)
			+ (s16)block[tmp]);*/
		temp_data = (int)((s16)(block[tmp + 1] << 8)
			+ (s16)block[tmp]);
		info->charge_current = cg7153am_data_filter(temp_data,
			info->charge_current,
			CG7153AM_REG_BATTERY_CURRENT);

		tmp = CG7153AM_REG_BATTERY_CAPACITY
			- CG7153AM_BATTERY_INFO_BLOCK_START_ADDR;

		/*info->batt_capacity = (int)block[tmp];*/
		temp_data = (int)block[tmp];

		/*If fw version biger than 0x4F then use capacity remap*/
		if (cg7153am_device->fw_version > FW_VERSIOR_BORDER) {
			tmp = CG7153AM_REG_BATTERY_REMAIN_CAPACITY
			- CG7153AM_BATTERY_INFO_BLOCK_START_ADDR;
			info->register_remain_capacity =
			(int)(block[tmp + 1] << 8) + (int)block[tmp];

			tmp = CG7153AM_REG_BATTERY_FULL_CAPACITY
			- CG7153AM_BATTERY_INFO_BLOCK_START_ADDR;
			info->register_full_capacity =
			(int)(block[tmp + 1] << 8) + (int)block[tmp];
			temp_data = cg7153am_capacity_calculate(
				info->register_remain_capacity,
			info->register_full_capacity);
		}

		info->batt_capacity = cg7153am_data_filter(temp_data,
			info->batt_capacity,
			CG7153AM_REG_BATTERY_CAPACITY);
		cg7153am_device->suspended = false;

		tmp = CG7153AM_REG_BATTERY_STATUS
			- CG7153AM_BATTERY_INFO_BLOCK_START_ADDR;
		info->register_status = (int)(block[tmp + 1] << 8)
			+ (int)block[tmp];

		info->ac_status = cg7153am_get_ac_status();
		info->batt_is_present = cg7153am_battery_is_present(
			info->batt_voltage, info->batt_temperature);
		info->batt_status = get_psy_prop_status(info->ac_status,
			info->batt_capacity, info->register_status,
			info->charge_current, info->batt_is_present);
		info->batt_health = cg7153am_get_battery_health();
	} else {
		printk(KERN_ERR "%s: failed to get battery info\n", __func__);
	}
	if (cg7153am_device->fw_version >= FW_VERSIOR_BORDER_GAUGE_FW) {
		for (i = 0 ; i < 2 ; i++) {
			cg7153am_device->gauge_cell_vol[i] =
				i2c_smbus_read_word_data_retry(
				cg7153am_device->client,
				i == 0 ? CG7153AM_REG_BATTERY_CELL1_VOL :
				CG7153AM_REG_BATTERY_CELL2_VOL);
			if (cg7153am_device->gauge_cell_vol[i] < 0)
				printk(KERN_ERR "%s gauge cell %d vol read Fail",
				__func__, i+1);
			if (cg7153am_device->gauge_cell_vol[i] < 1500)
				printk(KERN_ERR "%s gauge cell %d vol SUV=%dmv",
				__func__, i+1,
				cg7153am_device->gauge_cell_vol[i]);
		}
	temp_data = cg7153am_device->gauge_cell_vol[0] -
		cg7153am_device->gauge_cell_vol[1];
	if (absval(temp_data) >= IMBALANCE_VOL)
		printk(KERN_ERR "%s:CELL IMBALANCE cell1/2:%dmv/%dmv\n", __func__,
		cg7153am_device->gauge_cell_vol[0],
		cg7153am_device->gauge_cell_vol[1]);
	}
	cg7153am_device->update_time = jiffies;

	BATT_MSG("+%s: backup_lock = %d\n", __func__,
		atomic_read(&cg7153am_device->backup_lock));

	if (!atomic_read(&cg7153am_device->backup_lock))	{
		BATT_MSG("-%s: backup_lock = %d\n", __func__,
			atomic_read(&cg7153am_device->backup_lock));
		cg7153am_battery_status_backup(info);
	}

	if (atomic_read(&debug_enable))
		printk(KERN_INFO "batt:\t%d\t%d\t%d\t%d\t%d\t%d\n",
			info->ac_status, info->charge_current,
			info->batt_voltage, info->batt_temperature,
			info->batt_capacity, info->register_status);

	BATT_MSG("-%s:\n", __func__);
}

static int cg7153am_polling_thread(void *data)
{
	struct cg7153am_device_info *batt_info = data;

	BATT_MSG("+%s\n", __func__);
	while (1) {
		wait_event_interruptible_timeout(batt_info->wait_q_update,
			batt_info->state_changed &&
			!atomic_read(&cg7153am_device->is_in_suspend),
			atomic_read(&poll_interval));
		if (!atomic_read(&cg7153am_device->is_in_suspend)) {
			cg7153am_battery_status_update(batt_info);

			if (batt_info->state_changed) {
				batt_info->state_changed = false;
				BATT_MSG("%s:staus change update\n", __func__);
				power_supply_changed(
					&cg7153am_supply[SUPPLY_TYPE_AC]);
			} else {
				BATT_MSG("%s:battery staus update\n", __func__);
				power_supply_changed(
					&cg7153am_supply[SUPPLY_TYPE_BATTERY]);
			}
		} else {
			BATT_MSG("%s:polling-suspend\n", __func__);
		}

		wake_lock_timeout(&cg7153am_device->wake_lock, HZ);
	};

	BATT_MSG("-%s\n", __func__);
	return 0;
}
/*static void work_func(struct work_struct *work)
{
	printk(KERN_INFO "%s(%d)\n", __func__, __LINE__);
	if (cg7153am_device->charger_pdata->touch_change_config != NULL)
		cg7153am_device->charger_pdata->touch_change_config();
}*/

static irqreturn_t state_change_isr(int irq, void *power_supply)
{

	BATT_MSG("%s irq:%d\n", __func__, irq);

	/*schedule_work(&cg7153am_device->work);*/
	cg7153am_device->ac_latest_update_time = jiffies;
	cg7153am_device->state_changed = true;
	wake_up(&cg7153am_device->wait_q_update);

	return IRQ_HANDLED;
}

static int cg7153am_battery_probe(struct platform_device *pdev)
{
	int rc, flags;
	u16 gauge_fw_temp;
	struct i2c_client *client;
	client = to_i2c_client(pdev->dev.parent);
	cg7153am_device = kzalloc(sizeof(*cg7153am_device), GFP_KERNEL);
	if (!cg7153am_device) {
		printk(KERN_ERR "!cg7153am_device\n");
		return -ENOMEM;
	}

	memset(cg7153am_device, 0, sizeof(*cg7153am_device));
	init_waitqueue_head(&cg7153am_device->wait_q_update);
	cg7153am_device->client = client;
	cg7153am_device->update_time = jiffies;
	cg7153am_device->charger_pdata = pdev->dev.platform_data;
	cg7153am_device->ac_latest_update_time = jiffies;

	flags = cg7153am_device->client->flags;
	cg7153am_device->client->flags &= ~I2C_M_IGNORE_NAK;
	cg7153am_device->client->flags = flags;

	/*Initialize the data for battery status update*/
	cg7153am_device->tmp_count = 0;
	cg7153am_device->vol_count = 0;
	cg7153am_device->cur_count = 0;
	cg7153am_device->cap_count = 0;
	cg7153am_device->batt_temperature = TMP_MAX;
	cg7153am_device->batt_voltage	= VOL_MAX;
	cg7153am_device->charge_current = CUR_MAX;
	cg7153am_device->batt_capacity = CAP_MAX;
	cg7153am_device->data_id = 0;
	cg7153am_device->suspended = true;

	cg7153am_device->fw_version =
		cg7153am_version_get(cg7153am_device->client);

	if (cg7153am_device->charger_pdata->is_ac_online == NULL) {
		printk(KERN_ERR "%s is_ac_online = NULL !!!\n", __func__);
		rc =  -EINVAL;
		goto fail_init;
	}
	if (cg7153am_device->fw_version >= FW_VERSIOR_BORDER_GAUGE_FW) {
		gauge_fw_temp = i2c_smbus_read_word_data_retry(
			cg7153am_device->client,
			CG7153AM_REG_BATTERY_MANUFACTURER_ACCESS);
		if (gauge_fw_temp < 0)
			printk(KERN_ERR "%s gauge FW read Fail", __func__);
		else {
			cg7153am_device->gauge_fw_ascii[0] =
				(u8)(gauge_fw_temp & 0x00ff);
			cg7153am_device->gauge_fw_ascii[1] =
				(u8)((gauge_fw_temp & 0xff00) >> 8);
			printk(KERN_INFO "%s gauge FW V%c.%c", __func__,
				cg7153am_device->gauge_fw_ascii[0],
				cg7153am_device->gauge_fw_ascii[1]);
		}
	}
	cg7153am_battery_status_update(cg7153am_device);

	rc = request_irq(
		gpio_to_irq(cg7153am_device->charger_pdata->ac_present_gpio),
		state_change_isr,
		IRQF_SHARED | IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
		"ac_present_irq", &cg7153am_supply[SUPPLY_TYPE_AC]);
	if (rc < 0) {
		dev_err(&cg7153am_device->client->dev,
			"%s: request_irq failed(%d)\n", __func__, rc);
		goto fail_irq_ac_present;
	}

	atomic_set(&cg7153am_device->backup_lock, 0);
	cg7153am_device->sequence_index = 0;
/* Low battery is detected by MCU MCU_INTERRUPT*/
/*	rc = request_irq(
		gpio_to_irq(cg7153am_device->charger_pdata->lowbatt_gpio),
		state_change_isr,
		IRQF_SHARED | IRQF_TRIGGER_FALLING,
		"low_batt", &cg7153am_supply[SUPPLY_TYPE_BATTERY]);

	if (rc < 0) {
		printk(KERN_ERR "Failed to request low batt irq\n");
		goto fail_irq_low_batt;
	}
*/
	rc = power_supply_register(&client->dev,
		&cg7153am_supply[SUPPLY_TYPE_AC]);
	if (rc) {
		dev_err(&cg7153am_device->client->dev,
			"%s: Failed to register ac power supply\n", __func__);
			goto fail_power_register_ac;
	}

	if (!cg7153am_device->batt_is_present) {
		dev_err(&cg7153am_device->client->dev,
			"%s: Failed to detect battery", __func__);
		return 0;
	}

	power_supply_register(&client->dev,
		&cg7153am_supply[SUPPLY_TYPE_BATTERY]);
	if (rc) {
		dev_err(&cg7153am_device->client->dev,
			"%s: Failed to register batt power supply\n", __func__);
			goto fail_power_register_batt;
	}

	rc = cg7153am_battery_create_attrs(
		cg7153am_supply[SUPPLY_TYPE_BATTERY].dev);
	if (rc) {
		dev_err(&cg7153am_device->client->dev,
			"%s: Failed to create battery attribute\n", __func__);
			goto fail_power_register_batt;
	}

	device_init_wakeup(&client->dev, 1);
	cg7153am_device->state_changed = true;
	atomic_set(&poll_interval,
		cg7153am_device->charger_pdata->interval_slow);
	cg7153am_battery_set_polling_interval(
		cg7153am_device->charger_pdata->interval_slow);
	atomic_set(&debug_enable, 0);
	cg7153am_device->polling_thread = kthread_create(
		cg7153am_polling_thread, cg7153am_device,
		"cg7153am_polling_thread");
	wake_up_process(cg7153am_device->polling_thread);
	wake_lock_init(&cg7153am_device->wake_lock, WAKE_LOCK_SUSPEND,
		"batterylock");

	/*INIT_WORK(&cg7153am_device->work, work_func);*/

	device_init_wakeup(&pdev->dev, 1);
	if (device_may_wakeup(&pdev->dev))
		enable_irq_wake(gpio_to_irq(
			cg7153am_device->charger_pdata->ac_present_gpio));

	return 0;

fail_power_register_batt:
	power_supply_unregister(&cg7153am_supply[SUPPLY_TYPE_BATTERY]);

fail_power_register_ac:
	power_supply_unregister(&cg7153am_supply[SUPPLY_TYPE_AC]);

fail_irq_ac_present:
	free_irq(
		gpio_to_irq(cg7153am_device->charger_pdata->ac_present_gpio),
		cg7153am_device);
/*
fail_irq_low_batt:
	free_irq(
		gpio_to_irq(cg7153am_device->charger_pdata->lowbatt_gpio),
		cg7153am_device);
*/

fail_init:
	kfree(cg7153am_device);
	cg7153am_device = NULL;

	return rc;
}

static int cg7153am_battery_remove(struct platform_device *pdev)
{
	int i;

	cg7153am_battery_remove_attrs(cg7153am_supply[SUPPLY_TYPE_BATTERY].dev);

	for (i = 0; i < ARRAY_SIZE(cg7153am_supply); i++)
		power_supply_unregister(&cg7153am_supply[i]);

	free_irq(gpio_to_irq(
		cg7153am_device->charger_pdata->ac_present_gpio),
		cg7153am_device);

	kfree(cg7153am_device);
	cg7153am_device = NULL;

	return 0;
}

#if defined(CONFIG_PM)
static int cg7153am_battery_suspend(struct platform_device *pdev,
	pm_message_t state)
{
	printk(KERN_INFO "%s\n", __func__);
	cg7153am_device->suspended = true;
	atomic_set(&cg7153am_device->is_in_suspend, 1);
	wake_up(&cg7153am_device->wait_q_update);
	return 0;
}

/* any smbus transaction will wake up bq20z75 */
static int cg7153am_battery_resume(struct platform_device *pdev)
{
	printk(KERN_INFO "%s\n", __func__);
	atomic_set(&cg7153am_device->is_in_suspend, 0);
	wake_up(&cg7153am_device->wait_q_update);

	return 0;
}
#endif

static struct platform_driver cg7153am_battery_driver = {
	.driver	= {
		.name	= "cg7153am-battery",
		.owner	= THIS_MODULE,
	},
	.probe	= cg7153am_battery_probe,
	.remove	= __devexit_p(cg7153am_battery_remove),
	.suspend	= cg7153am_battery_suspend,
	.resume		= cg7153am_battery_resume,
};

static int __init cg7153am_battery_init(void)
{
	int rc;
	printk(KERN_INFO "%s\n", __func__);

	rc = platform_driver_register(&cg7153am_battery_driver);
	if (rc < 0)
		printk(KERN_ERR "Failed to register MCU driver\n");
	return rc;
}
module_init(cg7153am_battery_init);

static void __exit cg7153am_battery_exit(void)
{
	platform_driver_unregister(&cg7153am_battery_driver);
}
module_exit(cg7153am_battery_exit);

MODULE_AUTHOR("Pegatron Corporation");
MODULE_DESCRIPTION("BQ20z45 battery monitor driver");
MODULE_LICENSE("GPL");
