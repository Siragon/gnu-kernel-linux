
/*
 * pca9532.h - platform data structure for pca9532 led controller
 *
 * Copyright (C) 2008 Riku Voipio <riku.voipio@movial.fi>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * Datasheet: http://www.nxp.com/acrobat/datasheets/PCA9532_3.pdf
 *
 */

#ifndef __LINUX_LEDS_MP_H
#define __LINUX_LEDS_MP_H

#include <linux/leds.h>
#include <linux/i2c/cg7153am.h>

struct cg7153am_led_device {
	enum cg7153am_led_index led_id;
	struct led_classdev led_dev;
};

struct cg7153am_led_platform_data {
	u16 num_led;
	struct cg7153am_led_device *led_devs;
};

#endif /* __LINUX_LEDS_MP_H */
