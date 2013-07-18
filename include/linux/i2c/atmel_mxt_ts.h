/*
 * Atmel maXTouch Touchscreen driver
 *
 * Copyright (C) 2010 Samsung Electronics Co.Ltd
 * Copyright (C) 2011 Atmel Corporation
 * Copyright (C) 2011 NVIDIA Corporation
 * Author: Joonyoung Shim <jy0922.shim@samsung.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#ifndef __LINUX_ATMEL_MXT_TS_H
#define __LINUX_ATMEL_MXT_TS_H

#include <linux/types.h>

/*
 * Atmel I2C addresses
 */
#define	MXT224_I2C_ADDR1	0x4A
#define	MXT224_I2C_ADDR2	0x4B
#define	MXT1386_I2C_ADDR1	0x4C
#define	MXT1386_I2C_ADDR2	0x4D
#define	MXT1386_I2C_ADDR3	0x5A
#define	MXT1386_I2C_ADDR4	0x5B

/* Orient */
#define MXT_NORMAL		0x0
#define MXT_DIAGONAL		0x1
#define MXT_HORIZONTAL_FLIP	0x2
#define MXT_ROTATED_90_COUNTER	0x3
#define MXT_VERTICAL_FLIP	0x4
#define MXT_ROTATED_90		0x5
#define MXT_ROTATED_180		0x6
#define MXT_DIAGONAL_COUNTER	0x7

#define MXT_MESSAGE_BUFFER_SIZE        128

struct mxt_cfg_object {
       u8 type;
       u8 instance;
       u8 length;
       u8 *p_data;
};

struct mxt_cfg_noise {
	u8 type;
	u8 offset;
	u8 noise_value;
	u8 cfg_value;
};

/* The platform data for the Atmel maXTouch touchscreen driver */
struct mxt_platform_data {
	const u8 *config;
	size_t config_length;

	unsigned int x_line;
	unsigned int y_line;
	unsigned int x_size;
	unsigned int y_size;
	unsigned int blen;
	unsigned int threshold;
	unsigned int voltage;
	unsigned char orient;
	unsigned long irqflags;
	u8(*read_chg) (void);
	u32 config_crc;
	unsigned int actv_cycle_time;
	unsigned int idle_cycle_time;
	struct mxt_cfg_object *cfg_objects;
	size_t cfg_objects_length;
	struct mxt_cfg_noise *cfg_noises;
	size_t cfg_noises_length;
	unsigned int gpio_wake;
	unsigned int gpio_ac;
	unsigned int gpio_hdmi;
	unsigned int config_fw_version;
	u8 ap_threshold;
};

enum T25_self_test_result {
	e_self_test_none,
	e_self_test_fail,
	e_self_test_pass,
};

#endif /* __LINUX_ATMEL_MXT_TS_H */
