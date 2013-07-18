#ifndef __CG7153AM_H__
#define __CG7153AM_H__

#include <linux/i2c.h>

#define CG7153AM_EVT_FW_VERSION 0x10
#define CG7153AM_DVT_FW_VERSION 0x21
#define CG7153AM_PVT_FW_VERSION 0x30

struct device;

enum cg7153am_led_index {
	AMBER,
	WHITE
};

enum cg7153am_led_mode {
	MCU_LED_ON,
	MCU_LED_OFF,
	MCU_LED_BLINKING
};

enum cg7153am_finished_judgement {
	JUDGEMENT_FOR_DEAD_BATTERY,
	JUDGEMENT_FOR_OVER_DISCHARGE_BATTERY,
	JUDGEMENT_FOR_VENDOR_NAME,
};

struct cg7153am_subdev_info {
	int		id;
	const char	*name;
	void		*platform_data;
};

struct cg7153am_platform_data {
	int hw_version;
	int is_fw_update_mode;
	int gpio_sys_sus;
	int gpio_polarity_sys_sus;
	int gpio_wake;
	int gpio_polarity_wake;
	int gpio_mcu_int;
	int gpio_mcu_rst;
	int num_subdevs;
	struct cg7153am_subdev_info *sub_devs;
};

int cg7153am_led_set(struct device *cg7153am_dev, int index,
	enum cg7153am_led_mode mode);
int cg7153am_version_get(struct i2c_client *i2c);
void wakeup_mcu(struct i2c_client *i2c);

int suspend_mcu(struct i2c_client *i2c);

#endif /* __CG7153AM_H__ */
