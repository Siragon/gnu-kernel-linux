#ifndef ___MT9D115_SENSOR_H__
#define ___MT9D115_SENSOR_H__

#include <linux/ioctl.h>  /* For IOCTL macros */

#define MT9D115_NAME	"mt9d115"
#define MT9D115_PATH     "/dev/mt9d115"

#define MT9D115_WAIT_MS       0
/* special number to indicate this is wait time require */
#define MT9D115_TABLE_END     1
/* special number to indicate this is end of table */
#define MT9D115_MAX_RETRIES   3 /* max counter for retry I2C access */

#define MT9D115_IOCTL_SET_MODE		_IOW('o', 1, struct mt9d115_mode)
#define MT9D115_IOCTL_GET_STATUS		_IOR('o', 2, __u8)
#define MT9D115_IOCTL_SET_EFFECT	_IOW('o', 3, struct mt9d115_effect)
#define MT9D115_IOCTL_SET_AE_RECT	_IOW('o', 4, struct mt9d115_ae_rect)

#define MT9D115_ITEM_EFFECT		0
#define MT9D115_ITEM_WB			1
#define MT9D115_ITEM_BRIGHTNESS		2
#define MT9D115_ITEM_SCENE		3
#define MT9D115_ITEM_MAX		4

#define MT9D115_EFFECT_NONE		0
#define MT9D115_EFFECT_MONO		1
#define MT9D115_EFFECT_SEPIA		2
#define MT9D115_EFFECT_NEGATIVE		3
#define MT9D115_EFFECT_SOLARIZE		4
#define MT9D115_EFFECT_POSTERIZE	5

#define MT9D115_WB_AUTO			0
#define MT9D115_WB_SUNLIGHT		1
#define MT9D115_WB_CLOUDY		2
#define MT9D115_WB_FLUORESCENT		3
#define MT9D115_WB_INCANDESCENT		4

#define MT9D115_SCENE_AUTO		0
#define MT9D115_SCENE_ACTION		1
#define MT9D115_SCENE_NIGHT		2

#define MT9D115_BRIGHTNESS_0		0
#define MT9D115_BRIGHTNESS_P1		1
#define MT9D115_BRIGHTNESS_P2		2
#define MT9D115_BRIGHTNESS_N1		3
#define MT9D115_BRIGHTNESS_N2		4

struct mt9d115_mode {
	int xres;
	int yres;
	int still_count;
};

struct mt9d115_effect {
	int item;
	int value;
};

struct mt9d115_ae_rect {
	int x;
	int y;
	int width;
	int height;
};

#ifdef __KERNEL__
struct mt9d115_platform_data {
	int (*power_on)(void);
	int (*power_off)(void);

};
#endif /* __KERNEL__ */

#endif  /* __MT9D115_SENSOR_H__ */

