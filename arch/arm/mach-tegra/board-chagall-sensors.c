/*
 * arch/arm/mach-tegra/board-chagall-sensors.c
 *
 * Copyright (c) 2010-2012, NVIDIA CORPORATION, All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 * Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 *
 * Neither the name of NVIDIA CORPORATION nor the names of its contributors
 * may be used to endorse or promote products derived from this software
 * without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
 * IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
 * TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/regulator/consumer.h>
#include <linux/nct1008.h>
#include <mach/fb.h>
#include <mach/gpio.h>
#ifdef CONFIG_VIDEO_MT9D115
#include <media/mt9d115.h>
#endif /* CONFIG_VIDEO_T9D115 */
#include <media/yuv_sensor_common.h>
#ifdef CONFIG_VIDEO_CE1502
#include <media/ce1502.h>
#endif /* CONFIG_VIDEO_CE1502 */
#include <generated/mach-types.h>
#include "gpio-names.h"
#include "board.h"
#include <linux/mpu.h>
#include <mach/gpio.h>
#include <mach/edp.h>
#include <mach/thermal.h>
#include <linux/therm_est.h>

#include "gpio-names.h"
#include "board-chagall.h"
#include "cpu-tegra.h"
#include <linux/i2c/cg7153am_power.h>
#include <linux/leds-cg7153am.h>

#include <linux/platform_device.h>
#include <linux/switch.h>


struct tegra_camera_gpios {
	int gpio;
	bool output_enable;
	int value;
};

static struct tegra_camera_gpios chagall_camera_gpio_keys[] = {
	{CAM1_RST, true, 0},
	{CAM1_PWDN, true, 0},
	{CAM2_RST, true, 0},
	{CAM2_PWDN, true, 0},
	{CAM_MCLK_GPIO, true, 0},
	{GP42_8M_HOST_INT, false, 0},
};

struct tegra_camera_powers {
	struct regulator *reg;
	const char *name;
};

static struct tegra_camera_powers chagall_8M_powers[] = {
	{NULL, "vddio_1v8_5m"},
	{NULL, "avdd_2v8_5m"},
	{NULL, "vdd_2v8_5m"},
	{NULL, "vdd_cameraisp_1v1"},
	{NULL, "vdd_cameraisp_1v3"},
};

static struct tegra_camera_powers chagall_2M_powers[] = {
	{NULL, "vdd_1v8_2m"},
	{NULL, "vddio_1v8_2m"},
	{NULL, "avdd_2v8_2m"},
};

#ifdef CONFIG_BATTERY_CG7153AM

/*
#define HW_LOW_BATTERY_N TEGRA_GPIO_PS7
*/
#define BATTERY_POLL_PERIOD_SLOW 300
#define BATTERY_POLL_PERIOD_FASTER 100

static void chagall_cg7153am_gpio_init(void)
{
	tegra_gpio_enable(MCU_SYS_SUS);
	gpio_request(MCU_SYS_SUS, "MCU_SYS_SUS");
	gpio_direction_output(MCU_SYS_SUS, 1);
	gpio_export(MCU_SYS_SUS, false);

	tegra_gpio_enable(MCU_RESET);
	gpio_request(MCU_RESET, "MCU_RESET");
	gpio_direction_output(MCU_RESET, 0);
	gpio_export(MCU_RESET, true);

	tegra_gpio_enable(MCU_WAKE);
	gpio_request(MCU_WAKE, "MCU_WAKE");
	gpio_direction_output(MCU_WAKE, 1);
	gpio_export(MCU_WAKE, true);

	tegra_gpio_enable(MCU_INTERRUPT);
	gpio_request(MCU_INTERRUPT, "MCU_INTERRUPT");
	gpio_direction_input(MCU_INTERRUPT);

	tegra_gpio_enable(CHARGER_AC_PRESENT_GPIO);
	gpio_request(CHARGER_AC_PRESENT_GPIO, "AC_IN");
	gpio_direction_input(CHARGER_AC_PRESENT_GPIO);
}

static int battery_get_ac_status(void)
{
	return gpio_get_value(CHARGER_AC_PRESENT_GPIO);
}

static struct CG7153AM_power_pdata cg7153am_charger_platform = {
	.is_ac_online = battery_get_ac_status,
	.ac_present_gpio = CHARGER_AC_PRESENT_GPIO,
	.interval_slow = BATTERY_POLL_PERIOD_SLOW,
	.interval_faster = BATTERY_POLL_PERIOD_FASTER,
};

#define DEFINE_LED_ENTRY(index, led_name)			\
	{							\
		.led_id = index,				\
		.led_dev =					\
		{						\
			.name		= led_name,		\
			.brightness	= LED_OFF,		\
			.max_brightness = 1,			\
		},						\
	}

static struct cg7153am_led_device cg7153am_led_devs[] = {
	DEFINE_LED_ENTRY(WHITE, "white"),
	DEFINE_LED_ENTRY(AMBER, "amber"),
};

static struct cg7153am_led_platform_data cg7153am_led_platform_data = {
	.num_led	= ARRAY_SIZE(cg7153am_led_devs),
	.led_devs	= cg7153am_led_devs,
};

static struct cg7153am_subdev_info cg7153am_sub_devs[] = {
	{
		.id	= 0,
		.name	= "cg7153am-leds",
		.platform_data = &cg7153am_led_platform_data,
	},
	{
		.id	= 1,
		.name	= "cg7153am-battery",
		.platform_data = &cg7153am_charger_platform,
	},
};

struct cg7153am_platform_data chagall_cg7153am_platform_data = {
	.hw_version = -1,
	.is_fw_update_mode = 0,
	.gpio_sys_sus		= MCU_SYS_SUS,
	.gpio_polarity_sys_sus = 0,
	.gpio_wake		= MCU_WAKE,
	.gpio_polarity_wake = 1,
	.gpio_mcu_int	= MCU_INTERRUPT,
	.gpio_mcu_rst = MCU_RESET,
	.num_subdevs		= ARRAY_SIZE(cg7153am_sub_devs),
	.sub_devs		= cg7153am_sub_devs,
};

struct cg7153am_platform_data chagall_cg7153am_ota_platform_data = {
	.hw_version = -1,
	.is_fw_update_mode = 1,
	.gpio_sys_sus		= MCU_SYS_SUS,
	.gpio_polarity_sys_sus = 0,
	.gpio_wake		= MCU_WAKE,
	.gpio_polarity_wake = 1,
	.gpio_mcu_int	= MCU_INTERRUPT,
	.gpio_mcu_rst = MCU_RESET,
	.num_subdevs		= ARRAY_SIZE(cg7153am_sub_devs),
	.sub_devs		= cg7153am_sub_devs,
};

const struct i2c_board_info chagall_cg7153am_i2c[] = {
	{
		I2C_BOARD_INFO("cg7153am", 0x10),
		.platform_data	= &chagall_cg7153am_platform_data,
	},
};

static const struct i2c_board_info chagall_cg7153am_i2c_ota[] = {
	{
		I2C_BOARD_INFO("cg7153am", 0x77),
		.platform_data = &chagall_cg7153am_ota_platform_data,
	},
};
#endif /* CONFIG_BATTERY_CG7153AM */

static int chagall_camera_init(void)
{
	int ret, i;

	for (i = 0; i < ARRAY_SIZE(chagall_camera_gpio_keys); i++) {
		tegra_gpio_enable(chagall_camera_gpio_keys[i].gpio);
		ret = gpio_request(chagall_camera_gpio_keys[i].gpio,
			__func__);
		if (ret < 0) {
			pr_err("%s: gpio_request failed for gpio #%d\n",
				__func__, i);
			goto fail;
		}
		if (chagall_camera_gpio_keys[i].output_enable)
			gpio_direction_output(chagall_camera_gpio_keys[i].gpio,
				chagall_camera_gpio_keys[i].value);
		else
			gpio_direction_input(chagall_camera_gpio_keys[i].gpio);
		gpio_export(chagall_camera_gpio_keys[i].gpio, false);
	}

	return 0;
fail:
	while (i--)
		gpio_free(chagall_camera_gpio_keys[i].gpio);

	return ret;
}

static int chagall_sensors_power_enable(void)
{
	static struct regulator *chagall_sen_3v3_en = NULL;
	static struct regulator *chagall_sen_1v8_en = NULL;

       int ret = 0;
       if (NULL == chagall_sen_3v3_en) {
               chagall_sen_3v3_en = regulator_get(NULL, "vdd_3v3_sensor");
               if (WARN_ON(IS_ERR_OR_NULL(chagall_sen_3v3_en))) {
                       pr_err("%s(): couldn't get regulator vdd_3v3_sensor: %ld\n",
                                  __func__, PTR_ERR(chagall_sen_3v3_en));
                       ret = PTR_ERR(chagall_sen_3v3_en);
                       goto fail_3v3;
               }
       }
       ret = regulator_enable(chagall_sen_3v3_en);
       if (ret < 0) {
               pr_err("%s(): couldn't enable regulator vdd_3v3_sensor\n", __func__);
               goto fail_3v3;
       }

       if (NULL == chagall_sen_1v8_en) {
               chagall_sen_1v8_en = regulator_get(NULL, "vdd_1v8_sensor");
               if (WARN_ON(IS_ERR_OR_NULL(chagall_sen_1v8_en))) {
                       pr_err("%s(): couldn't get regulator vdd_1v8_sensor: %ld\n",
                                  __func__, PTR_ERR(chagall_sen_1v8_en));
                       ret = PTR_ERR(chagall_sen_1v8_en);
                       goto fail_1v8;
               }
       }
       ret = regulator_enable(chagall_sen_1v8_en);
       if (ret < 0) {
               pr_err("%s(): couldn't enable regulator vdd_1v8_sensor\n", __func__);

               goto fail_1v8;
       }
       return ret;

fail_1v8:
       if (chagall_sen_1v8_en) {
               regulator_put(chagall_sen_1v8_en);
               chagall_sen_1v8_en = NULL;
       }
fail_3v3:
       if (chagall_sen_3v3_en) {
               regulator_put(chagall_sen_3v3_en);
               chagall_sen_3v3_en = NULL;
       }
       return ret;
}
fs_initcall(chagall_sensors_power_enable);

#ifdef CONFIG_VIDEO_MT9D115
static int mt9d115_power_on(void)
{
	int i;
	pr_info("%s ++\n", __func__);

	for (i = 0; i < ARRAY_SIZE(chagall_2M_powers); i++) {
		if (chagall_2M_powers[i].reg == NULL) {
			chagall_2M_powers[i].reg = regulator_get(NULL,
				chagall_2M_powers[i].name);
			if (WARN_ON(IS_ERR(chagall_2M_powers[i].reg))) {
				pr_err("%s: couldn't get %s: %ld\n", __func__,
					chagall_2M_powers[i].name,
					PTR_ERR(chagall_2M_powers[i].reg));
				goto reg_alloc_fail;
			}
		}
	}

	gpio_direction_output(CAM2_PWDN, 0);
	gpio_direction_output(CAM2_RST, 1);
	regulator_enable(chagall_2M_powers[1].reg);
	mdelay(5);
	regulator_enable(chagall_2M_powers[0].reg);
	mdelay(5);
	tegra_gpio_disable(CAM_MCLK_GPIO);
	mdelay(10);
	gpio_direction_output(CAM2_RST, 0);
	mdelay(10);
	gpio_direction_output(CAM2_RST, 1);
	mdelay(10);
	regulator_enable(chagall_2M_powers[2].reg);
	mdelay(10);
	pr_info("%s --\n", __func__);
	return 0;
reg_alloc_fail:
	return -ENODEV;
}

static int mt9d115_power_off(void)
{
	pr_info("%s ++\n", __func__);
	gpio_direction_output(CAM2_RST, 0);
	mdelay(20);
	if (chagall_2M_powers[2].reg)
		regulator_disable(chagall_2M_powers[2].reg);
	if (chagall_2M_powers[0].reg)
		regulator_disable(chagall_2M_powers[0].reg);
	if (chagall_2M_powers[1].reg)
		regulator_disable(chagall_2M_powers[1].reg);
	tegra_gpio_enable(CAM_MCLK_GPIO);
	pr_info("%s --\n", __func__);
	return 0;
}

struct mt9d115_platform_data mt9d115_data = {
	.power_on = mt9d115_power_on,
	.power_off = mt9d115_power_off,
};
#endif /* CONFIG_VIDEO_MT9D115 */

#ifdef CONFIG_VIDEO_CE1502
static int chagall_ce1502_power_on(void)
{
	int i;
	pr_info("%s ++\n", __func__);

	for (i = 0; i < ARRAY_SIZE(chagall_8M_powers); i++) {
		if (chagall_8M_powers[i].reg == NULL) {
			chagall_8M_powers[i].reg = regulator_get(NULL,
				chagall_8M_powers[i].name);
			if (WARN_ON(IS_ERR(chagall_8M_powers[i].reg))) {
				pr_err("%s: couldn't get %s: %ld\n", __func__,
					chagall_8M_powers[i].name,
					PTR_ERR(chagall_8M_powers[i].reg));
				goto reg_alloc_fail;
			}
		}
	}
	regulator_set_voltage(chagall_8M_powers[4].reg, 1300*1000,
		1300*1000);

	regulator_enable(chagall_8M_powers[3].reg);
	regulator_enable(chagall_8M_powers[0].reg);
	regulator_enable(chagall_8M_powers[1].reg);
	regulator_enable(chagall_8M_powers[4].reg);
	regulator_enable(chagall_8M_powers[2].reg);

	mdelay(1);
	tegra_gpio_disable(CAM_MCLK_GPIO);
	mdelay(1);
	gpio_direction_output(CAM1_PWDN, 1);
	udelay(5);
	gpio_direction_output(CAM1_RST, 1);
	mdelay(1);
	pr_info("%s --\n", __func__);
	return 0;

reg_alloc_fail:
	return -ENODEV;
}

static int chagall_ce1502_power_off(void)
{
	pr_info("%s ++\n", __func__);
	gpio_direction_output(CAM1_RST, 0);
	gpio_direction_output(CAM1_PWDN, 0);
	mdelay(1);
	tegra_gpio_enable(CAM_MCLK_GPIO);
	mdelay(1);
	if (chagall_8M_powers[2].reg)
		regulator_disable(chagall_8M_powers[2].reg);
	udelay(1);
	if (chagall_8M_powers[4].reg)
		regulator_disable(chagall_8M_powers[4].reg);
	if (chagall_8M_powers[1].reg)
		regulator_disable(chagall_8M_powers[1].reg);
	if (chagall_8M_powers[0].reg)
		regulator_disable(chagall_8M_powers[0].reg);
	if (chagall_8M_powers[3].reg)
		regulator_disable(chagall_8M_powers[3].reg);
	pr_info("%s --\n", __func__);
	return 0;
}

static int chagall_ce1502_check_interrupt(int pin)
{
	if (1 == gpio_get_value(GP42_8M_HOST_INT))
		return 1;
		return 0;
}

struct yuvsensor_platform_data chagall_ce1502_data = {
	.power_on = chagall_ce1502_power_on,
	.power_off = chagall_ce1502_power_off,
	.check_interrupt = chagall_ce1502_check_interrupt,
};
#endif /* CONFIG_VIDEO_CE1502 */

static const struct i2c_board_info chagall_i2c3_board_info[] = {
#ifdef CONFIG_VIDEO_CE1502
	{
		I2C_BOARD_INFO(CE1502_NAME, 0x3c),
		.platform_data = &chagall_ce1502_data,
	},
#endif /* CONFIG_VIDEO_CE1502 */
#ifdef CONFIG_VIDEO_MT9D115
	{
		I2C_BOARD_INFO(MT9D115_NAME, 0x3d),
		.platform_data = &mt9d115_data,
	},
#endif /* CONFIG_VIDEO_MT9D115 */
};



static int nct_get_temp(void *_data, long *temp)
{
	struct nct1008_data *data = _data;
	return nct1008_thermal_get_temp(data, temp);
}

static int nct_get_temp_low(void *_data, long *temp)
{
	struct nct1008_data *data = _data;
	return nct1008_thermal_get_temp_low(data, temp);
}

static int nct_set_limits(void *_data,
			long lo_limit_milli,
			long hi_limit_milli)
{
	struct nct1008_data *data = _data;
	return nct1008_thermal_set_limits(data,
					lo_limit_milli,
					hi_limit_milli);
}

static int nct_set_alert(void *_data,
				void (*alert_func)(void *),
				void *alert_data)
{
	struct nct1008_data *data = _data;
	return nct1008_thermal_set_alert(data, alert_func, alert_data);
}

static int nct_set_shutdown_temp(void *_data, long shutdown_temp)
{
	struct nct1008_data *data = _data;
	return nct1008_thermal_set_shutdown_temp(data, shutdown_temp);
}

#ifdef CONFIG_TEGRA_SKIN_THROTTLE
static int nct_get_itemp(void *dev_data, long *temp)
{
	struct nct1008_data *data = dev_data;
	return nct1008_thermal_get_temps(data, NULL, temp);
}
#endif

static void nct1008_probe_callback(struct nct1008_data *data)
{
	struct tegra_thermal_device *ext_nct;

	ext_nct = kzalloc(sizeof(struct tegra_thermal_device),
					GFP_KERNEL);
	if (!ext_nct) {
		pr_err("unable to allocate thermal device\n");
		return;
	}

	ext_nct->name = "nct_ext";
	ext_nct->id = THERMAL_DEVICE_ID_NCT_EXT;
	ext_nct->data = data;
	ext_nct->offset = TDIODE_OFFSET;
	ext_nct->get_temp = nct_get_temp;
	ext_nct->get_temp_low = nct_get_temp_low;
	ext_nct->set_limits = nct_set_limits;
	ext_nct->set_alert = nct_set_alert;
	ext_nct->set_shutdown_temp = nct_set_shutdown_temp;

	tegra_thermal_device_register(ext_nct);

#ifdef CONFIG_TEGRA_SKIN_THROTTLE
	{
		struct tegra_thermal_device *int_nct;
		int_nct = kzalloc(sizeof(struct tegra_thermal_device),
						GFP_KERNEL);
		if (!int_nct) {
			kfree(int_nct);
			pr_err("unable to allocate thermal device\n");
			return;
		}

		int_nct->name = "nct_int";
		int_nct->id = THERMAL_DEVICE_ID_NCT_INT;
		int_nct->data = data;
		int_nct->get_temp = nct_get_itemp;

		tegra_thermal_device_register(int_nct);
	}
#endif
}

static struct nct1008_platform_data chagall_nct1008_pdata = {
	.supported_hwrev = true,
	.ext_range = true,
	.conv_rate = 0x04,
	.offset = 8,
	.probe_callback = nct1008_probe_callback,
};

static struct i2c_board_info chagall_i2c4_nct1008_board_info[] = {
	{
		I2C_BOARD_INFO("nct1008", 0x4C),
		.platform_data = &chagall_nct1008_pdata,
		.irq = -1,
	}
};

static int chagall_nct1008_init(void)
{
	int nct1008_port = -1;
	int ret = 0;

	nct1008_port = TEGRA_GPIO_PU5;

	if (nct1008_port >= 0) {
		chagall_i2c4_nct1008_board_info[0].irq = TEGRA_GPIO_TO_IRQ(nct1008_port);

		ret = gpio_request(nct1008_port, "temp_alert");
		if (ret < 0)
			return ret;

		ret = gpio_direction_input(nct1008_port);
		if (ret < 0)
			gpio_free(nct1008_port);
	}

	return ret;
}

/* MPU board file definition	*/
#if (MPU_GYRO_TYPE == MPU_TYPE_MPU3050)
#define MPU_GYRO_NAME		"mpu3050"
#endif
#if (MPU_GYRO_TYPE == MPU_TYPE_MPU6050)
#define MPU_GYRO_NAME		"mpu6050"
#endif
static struct mpu_platform_data mpu_gyro_data = {
	.int_config	= 0x10,
	.level_shifter	= 0,
	.orientation	= MPU_GYRO_ORIENTATION,	/* Located in board_[platformname].h	*/
};

#if (MPU_GYRO_TYPE == MPU_TYPE_MPU3050)
static struct ext_slave_platform_data mpu_accel_data = {
	.address	= MPU_ACCEL_ADDR,
	.irq		= 0,
	.adapt_num	= MPU_ACCEL_BUS_NUM,
	.bus		= EXT_SLAVE_BUS_SECONDARY,
	.orientation	= MPU_ACCEL_ORIENTATION,	/* Located in board_[platformname].h	*/
};
#endif

static struct ext_slave_platform_data mpu_compass_data = {
	.address	= MPU_COMPASS_ADDR,
	.irq		= 0,
	.adapt_num	= MPU_COMPASS_BUS_NUM,
	.bus		= EXT_SLAVE_BUS_PRIMARY,
	.orientation	= MPU_COMPASS_ORIENTATION,	/* Located in board_[platformname].h	*/
};

static struct i2c_board_info __initdata inv_mpu_i2c2_board_info[] = {
	{
		I2C_BOARD_INFO(MPU_GYRO_NAME, MPU_GYRO_ADDR),
		.irq = TEGRA_GPIO_TO_IRQ(MPU_GYRO_IRQ_GPIO),
		.platform_data = &mpu_gyro_data,
	},
	{
		I2C_BOARD_INFO(MPU_ACCEL_NAME, MPU_ACCEL_ADDR),
		.irq = TEGRA_GPIO_TO_IRQ(MPU_ACCEL_IRQ_GPIO),
		.platform_data = &mpu_accel_data,
	},
	{
		I2C_BOARD_INFO(MPU_COMPASS_NAME, MPU_COMPASS_ADDR),
		.irq = TEGRA_GPIO_TO_IRQ(MPU_COMPASS_IRQ_GPIO),
		.platform_data = &mpu_compass_data,
	},
};

static void mpuirq_init(void)
{
	int ret = 0;

	pr_info("*** MPU START *** mpuirq_init...\n");

#if (MPU_GYRO_TYPE == MPU_TYPE_MPU3050)
#if	MPU_ACCEL_IRQ_GPIO
	ret = gpio_request(MPU_ACCEL_IRQ_GPIO, MPU_ACCEL_NAME);
	if (ret < 0) {
		pr_err("%s: gpio_request failed %d\n", __func__, ret);
		return;
	}

	ret = gpio_direction_input(MPU_ACCEL_IRQ_GPIO);
	if (ret < 0) {
		pr_err("%s: gpio_direction_input failed %d\n", __func__, ret);
		gpio_free(MPU_ACCEL_IRQ_GPIO);
		return;
	}
#endif
#endif

	/* MPU-IRQ assignment */
	ret = gpio_request(MPU_GYRO_IRQ_GPIO, MPU_GYRO_NAME);
	if (ret < 0) {
		pr_err("%s: gpio_request failed %d\n", __func__, ret);
		return;
	}

	ret = gpio_direction_input(MPU_GYRO_IRQ_GPIO);
	if (ret < 0) {
		pr_err("%s: gpio_direction_input failed %d\n", __func__, ret);
		gpio_free(MPU_GYRO_IRQ_GPIO);
		return;
	}
	pr_info("*** MPU END *** mpuirq_init...\n");

	i2c_register_board_info(MPU_GYRO_BUS_NUM, inv_mpu_i2c2_board_info,
		ARRAY_SIZE(inv_mpu_i2c2_board_info));
}

#define IQS128_IRQ       TEGRA_GPIO_PQ3

/*Switch GPIO*/
static struct gpio_switch_platform_data iqs128_switch_data = {
       .name = "iqs128",
       .gpio = IQS128_IRQ,
       .wakeup = 0,
       .state_off = "CAP_SENSOR_PRESS",
       .state_on = "CAP_SENSOR_RELEASE",
       .irqflags = IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
};

static struct platform_device iqs128_switch_device = {
       .name = "switch-gpio",
       .id = 0,
       .dev = {
                  .platform_data = &iqs128_switch_data,
       }
};

static int chagall_iqs128_init(void)
{
       printk(KERN_INFO "chagall_iqs128_init\n");

	tegra_gpio_enable(iqs128_switch_data.gpio);

       platform_device_register(&iqs128_switch_device);

       return 0;
}


static struct i2c_board_info chagall_i2c2_isl_board_info[] = {
	{
		I2C_BOARD_INFO("isl29023", 0x44),
	}
};

int __init chagall_sensors_init(bool is_wifi_sku)
{
	int err;

	if (is_wifi_sku == false) {
		chagall_iqs128_init();
	}

	chagall_camera_init();

	i2c_register_board_info(2, chagall_i2c3_board_info,
		ARRAY_SIZE(chagall_i2c3_board_info));

	err = chagall_nct1008_init();
	if (err)
		return err;
	i2c_register_board_info(4, chagall_i2c4_nct1008_board_info,
		ARRAY_SIZE(chagall_i2c4_nct1008_board_info));

#ifdef CONFIG_BATTERY_CG7153AM
	chagall_cg7153am_gpio_init();

	i2c_register_board_info(0, chagall_cg7153am_i2c_ota,
			ARRAY_SIZE(chagall_cg7153am_i2c_ota));

	i2c_register_board_info(0, chagall_cg7153am_i2c,
			ARRAY_SIZE(chagall_cg7153am_i2c));
#endif /* CONFIG_BATTERY_CG7153AM */

#ifdef CONFIG_SENSORS_ISL29023
	i2c_register_board_info(2, chagall_i2c2_isl_board_info,
			ARRAY_SIZE(chagall_i2c2_isl_board_info));
#endif /* CONFIG_SENSORS_ISL29023 */

	mpuirq_init();
	return 0;
}

