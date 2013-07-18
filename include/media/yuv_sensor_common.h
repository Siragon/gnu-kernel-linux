#ifndef ___YUV_SENSOR_COMMON_H__
#define ___YUV_SENSOR_COMMON_H__

/*
 * kernel/drivers/media/video/tegra
 *
 * YUV Sensor Common Interface
 *
 * This header contains IOCTL code, relative data structures and enums used in
 * YUV sensor common interface.
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

#include <linux/ioctl.h>  /* For IOCTL macros */

#define YUVSENSOR_IOCTL_SET_MODE \
	_IOW('o', 1, struct yuvsensor_mode)

#define YUVSENSOR_IOCTL_TRIGGER_AF \
	_IOW('o', 2, int)

#define YUVSENSOR_IOCTL_GET_AF_STATUS \
	_IOR('o', 3, int)

#define YUVSENSOR_IOCTL_SET_EFFECT \
	_IOW('o', 4, struct yuvsensor_effect)

#define YUVSENSOR_IOCTL_GET_EXPOSURE_TIME \
	_IOR('o', 5, struct yuvsensor_exposure)

#define YUVSENSOR_IOCTL_GET_FACE_STATUS \
	_IOR('o', 6, int)

#define YUVSENSOR_IOCTL_SET_AF_AREA \
	_IOR('o', 7, struct yuvsensor_region)

#define YUVSENSOR_IOCTL_SET_AE_AREA \
	_IOR('o', 8, struct yuvsensor_region)

enum {
	YUVSENSOR_ITEM_COLOREFFECT,
	YUVSENSOR_ITEM_WB,
	YUVSENSOR_ITEM_BRIGHTNESS,
	YUVSENSOR_ITEM_SCENE,
	YUVSENSOR_ITEM_FLASHMODE,
	YUVSENSOR_ITEM_AFMODE,
	YUVSENSOR_ITEM_MAX_TO_INIT,
/* Below items are for special usage. */
	YUVSENSOR_ITEM_FLASHVALUE,
	YUVSENSOR_ITEM_AELOCK,
};

enum {
	YUVSENSOR_COLOREFFECT_NONE,
	YUVSENSOR_COLOREFFECT_MONO,
	YUVSENSOR_COLOREFFECT_SEPIA,
	YUVSENSOR_COLOREFFECT_NEGATIVE,
	YUVSENSOR_COLOREFFECT_SOLARIZE,
	YUVSENSOR_COLOREFFECT_POSTERIZE,
};

enum {
	YUVSENSOR_WB_AUTO,
	YUVSENSOR_WB_SUNLIGHT,
	YUVSENSOR_WB_CLOUDY,
	YUVSENSOR_WB_FLUORESCENT,
	YUVSENSOR_WB_INCANDESCENT,
};

enum {
	YUVSENSOR_SCENE_AUTO,
	YUVSENSOR_SCENE_ACTION,
	YUVSENSOR_SCENE_NIGHT,
};

enum {
	YUVSENSOR_BRIGHTNESS_0,
	YUVSENSOR_BRIGHTNESS_P1,
	YUVSENSOR_BRIGHTNESS_P2,
	YUVSENSOR_BRIGHTNESS_N1,
	YUVSENSOR_BRIGHTNESS_N2,
};

enum {
	YUVSENSOR_FLASHMODE_OFF,
	YUVSENSOR_FLASHMODE_ON,
	YUVSENSOR_FLASHMODE_AUTO,
	YUVSENSOR_FLASHMODE_TORCH,
};

enum {
	YUVSENSOR_AFMODE_INFINITY,
	YUVSENSOR_AFMODE_FIXED,
	YUVSENSOR_AFMODE_AUTO,
	YUVSENSOR_AFMODE_MACRO,
	YUVSENSOR_AFMODE_CONTINUOUS_VIDEO,
	YUVSENSOR_AFMODE_CONTINUOUS_PICTURE,
};

enum {
	YUVSENSOR_AELOCK_OFF,
	YUVSENSOR_AELOCK_ON,
};

enum {
	YUVSENSOR_FACEDETECT_OFF,
	YUVSENSOR_FACEDETECT_ON,
};

struct yuvsensor_mode {
	int xres;
	int yres;
	int still_count;
};

struct yuvsensor_effect {
	int item;
	int value;
};

struct yuvsensor_exposure {
	int coarse_exposure_line;
	int line_length_pck;
	int gain;
	int pclk_mclk_ratio;
	int flash_is_fired;
};

struct yuvsensor_face {
	int rect[4];
};

struct yuvsensor_faces {
	int number_of_faces;
	struct yuvsensor_face faces[10];
};

struct yuvsensor_region {
	int left;
	int top;
	int right;
	int bottom;
};

struct yuvsensor_regions {
	int number_of_regions;
	struct yuvsensor_region regions[10];
};

#ifdef __KERNEL__

struct yuvsensor_platform_data {
	int (*power_on)(void);
	int (*power_off)(void);
	/* board-specific led function (usually controlled by GPIOs) */
	int (*set_led_level) (int);
	int (*check_interrupt) (int);
};
#endif /* __KERNEL__ */

#endif  /* __YUV_SENSOR_COMMON_H__ */

