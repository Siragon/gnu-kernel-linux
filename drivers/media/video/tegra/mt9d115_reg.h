#ifndef ___MT9D115_REG_H__
#define ___MT9D115_REG_H__

#include <linux/uaccess.h>

struct sensor_reg {
	u16 addr;
	u16 val;
};

enum {
	SENSOR_MODE_1600x1200,
	SENSOR_MODE_1280x720,
	SENSOR_MODE_800x600,
	SENSOR_MODE_800x480,
	SENSOR_MODE_768x432,
	SENSOR_MODE_640x480,
};

extern struct sensor_reg *mt9d115_mode_table[];
extern struct sensor_reg *mt9d115_mode_table_capture[];
extern struct sensor_reg mt9d115_init[];
extern struct sensor_reg mt9d115_scene_auto[];
extern struct sensor_reg mt9d115_scene_action[];
extern struct sensor_reg mt9d115_scene_night[];
extern struct sensor_reg mt9d115_wb_auto[];
extern struct sensor_reg mt9d115_wb_sunlight[];
extern struct sensor_reg mt9d115_wb_fluorescent[];
extern struct sensor_reg mt9d115_wb_incandescent[];
extern struct sensor_reg mt9d115_effect_none[];
extern struct sensor_reg mt9d115_effect_mono[];
extern struct sensor_reg mt9d115_effect_sepia[];
extern struct sensor_reg mt9d115_effect_negative[];
extern struct sensor_reg mt9d115_effect_solarize[];
extern struct sensor_reg mt9d115_effect_posterize[];
extern struct sensor_reg mt9d115_brightness_0[];
extern struct sensor_reg mt9d115_brightness_p1[];
extern struct sensor_reg mt9d115_brightness_p2[];
extern struct sensor_reg mt9d115_brightness_n1[];
extern struct sensor_reg mt9d115_brightness_n2[];
extern struct sensor_reg mt9d115_back_to_preview[];
extern struct sensor_reg mt9d115_ae_rect_default[];
extern struct sensor_reg mt9d115_ae_rectangular[];

#endif  /* __MT9D115_REG_H__ */
