/*
 * arch/arm/mach-tegra/board-chagall-power.c
 *
 * Copyright (C) 2011-2012, NVIDIA Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
 * 02111-1307, USA
 */
#include <linux/i2c.h>
#include <linux/pda_power.h>
#include <linux/platform_device.h>
#include <linux/resource.h>
#include <linux/regulator/machine.h>
#include <linux/mfd/tps6591x.h>
#include <linux/mfd/max77663-core.h>
#include <linux/gpio.h>
#include <linux/io.h>
#include <linux/regulator/fixed.h>
#include <linux/regulator/tps6591x-regulator.h>
#include <linux/regulator/tps62360.h>
#include <linux/power/gpio-charger.h>

#include <asm/mach-types.h>

#include <mach/iomap.h>
#include <mach/irqs.h>
#include <mach/pinmux.h>
#include <mach/edp.h>

#include "gpio-names.h"
#include "board.h"
#include "board-chagall.h"
#include "pm.h"
#include "tegra3_tsensor.h"

#define PMC_CTRL		0x0
#define PMC_CTRL_INTR_LOW	(1 << 17)

static struct regulator_consumer_supply tps6591x_vdd1_supply_chagall[] = {
	REGULATOR_SUPPLY("vmmc", NULL),
};

static struct regulator_consumer_supply tps6591x_vdd1_supply_chagall_dvt[] = {
	REGULATOR_SUPPLY("unused_vdd1", NULL),
};

static struct regulator_consumer_supply tps6591x_vdd2_supply_chagall[] = {
	REGULATOR_SUPPLY("vddio_ddr_1v2", NULL),
};

static struct regulator_consumer_supply tps6591x_vddctrl_supply_chagall[] = {
	REGULATOR_SUPPLY("vdd_cpu_pmu", NULL),
	REGULATOR_SUPPLY("vdd_cpu", NULL),
};

static struct regulator_consumer_supply tps6591x_vio_supply_chagall[] = {
	REGULATOR_SUPPLY("vdd_gen1v8", NULL),
	REGULATOR_SUPPLY("avdd_hdmi_pll", NULL),
	REGULATOR_SUPPLY("vddio_uart", NULL),
	REGULATOR_SUPPLY("vddio_audio", NULL),
	REGULATOR_SUPPLY("vddio_bb", NULL),
	REGULATOR_SUPPLY("avdd_usb_pll", NULL),
	REGULATOR_SUPPLY("vddio_cam", NULL),
	REGULATOR_SUPPLY("vddio_lcd", NULL),
	REGULATOR_SUPPLY("vddio_sdmmc", "sdhci-tegra.3"),
	REGULATOR_SUPPLY("vddio_sdmmc", "sdhci-tegra.2"),
	REGULATOR_SUPPLY("avdd_osc", NULL),
	REGULATOR_SUPPLY("vddio_sys", NULL),
	REGULATOR_SUPPLY("dsp_vddio_1v8", NULL),
	REGULATOR_SUPPLY("cdc_dcvdd", NULL),
	REGULATOR_SUPPLY("cdc_dbvdd", NULL),
	REGULATOR_SUPPLY("cdc_cpvdd", NULL),
	REGULATOR_SUPPLY("cdc_avdd", NULL),
	REGULATOR_SUPPLY("lvds_iovcc_1v8", NULL),
	REGULATOR_SUPPLY("vdd_1v2_1v8_emmc", NULL),
	REGULATOR_SUPPLY("pwrdet_cam", NULL),
	REGULATOR_SUPPLY("pwrdet_sdmmc4", NULL),
	REGULATOR_SUPPLY("pwrdet_uart", NULL),
	REGULATOR_SUPPLY("pwrdet_audio", NULL),
	REGULATOR_SUPPLY("pwrdet_bb", NULL),
	REGULATOR_SUPPLY("pwrdet_lcd", NULL),
	REGULATOR_SUPPLY("pwrdet_vi", NULL),
	REGULATOR_SUPPLY("pwrdet_sdmmc3", NULL),
	REGULATOR_SUPPLY("ldo5", NULL),
	REGULATOR_SUPPLY("ldo6", NULL),
	REGULATOR_SUPPLY("ldo7", NULL),
	REGULATOR_SUPPLY("ldo8", NULL),
};
static struct regulator_consumer_supply tps6591x_vio_supply_chagall_mp[] = {
	REGULATOR_SUPPLY("vdd_gen1v8", NULL),
	REGULATOR_SUPPLY("avdd_hdmi_pll", NULL),
	REGULATOR_SUPPLY("vddio_uart", NULL),
	REGULATOR_SUPPLY("vddio_audio", NULL),
	REGULATOR_SUPPLY("vddio_bb", NULL),
	REGULATOR_SUPPLY("avdd_usb_pll", NULL),
	REGULATOR_SUPPLY("vddio_cam", NULL),
	REGULATOR_SUPPLY("vddio_lcd", NULL),
	REGULATOR_SUPPLY("vddio_sdmmc", "sdhci-tegra.3"),
	REGULATOR_SUPPLY("vddio_sdmmc", "sdhci-tegra.2"),
	REGULATOR_SUPPLY("avdd_osc", NULL),
	REGULATOR_SUPPLY("vddio_sys", NULL),
	REGULATOR_SUPPLY("dsp_vddio_1v8", NULL),
	REGULATOR_SUPPLY("cdc_dcvdd", NULL),
	REGULATOR_SUPPLY("cdc_dbvdd", NULL),
	REGULATOR_SUPPLY("cdc_cpvdd", NULL),
	REGULATOR_SUPPLY("cdc_avdd", NULL),
	REGULATOR_SUPPLY("lvds_iovcc_1v8", NULL),
	REGULATOR_SUPPLY("vdd_1v2_1v8_emmc", NULL),
	REGULATOR_SUPPLY("pwrdet_cam", NULL),
	REGULATOR_SUPPLY("pwrdet_sdmmc4", NULL),
	REGULATOR_SUPPLY("pwrdet_uart", NULL),
	REGULATOR_SUPPLY("pwrdet_audio", NULL),
	REGULATOR_SUPPLY("pwrdet_bb", NULL),
	REGULATOR_SUPPLY("pwrdet_lcd", NULL),
	REGULATOR_SUPPLY("pwrdet_vi", NULL),
	REGULATOR_SUPPLY("pwrdet_sdmmc3", NULL),
	REGULATOR_SUPPLY("ldo5", NULL),
	REGULATOR_SUPPLY("ldo6", NULL),
	REGULATOR_SUPPLY("ldo7", NULL),
	REGULATOR_SUPPLY("ldo8", NULL),
	REGULATOR_SUPPLY("vdd_gps_1v8", NULL),
	REGULATOR_SUPPLY("vdd_wlan_1v8", NULL),
};

/*VDD_2V85_EMMC*/
static struct regulator_consumer_supply tps6591x_ldo1_supply_chagall[] = {
	REGULATOR_SUPPLY("vdd_2v85_emmc", NULL),
};

static struct regulator_consumer_supply tps6591x_ldo2_supply_chagall[] = {
	REGULATOR_SUPPLY("vddio_sd_slot", NULL),
};

static struct regulator_consumer_supply tps6591x_ldo3_supply_chagall[] = {
	REGULATOR_SUPPLY("vddio_sdmmc", "sdhci-tegra.0"),
	REGULATOR_SUPPLY("pwrdet_sdmmc1", NULL),
	REGULATOR_SUPPLY("en_gps_1v8_3v3", NULL),
};

static struct regulator_consumer_supply tps6591x_ldo4_supply_chagall[] = {
	REGULATOR_SUPPLY("vdd_rtc", NULL),
};

static struct regulator_consumer_supply tps6591x_ldo5_supply_chagall[] = {
	REGULATOR_SUPPLY("vdd_cameraisp_1v3", NULL),
};

static struct regulator_consumer_supply tps6591x_ldo6_supply_chagall[] = {
	REGULATOR_SUPPLY("avdd_dsi_csi", NULL),
	REGULATOR_SUPPLY("vddio_hsic", NULL),
	REGULATOR_SUPPLY("pwrdet_mipi", NULL),
};
static struct regulator_consumer_supply tps6591x_ldo7_supply_chagall[] = {
	REGULATOR_SUPPLY("avdd_plla_p_c_s", NULL),
	REGULATOR_SUPPLY("avdd_pllm", NULL),
	REGULATOR_SUPPLY("avdd_pllu_d", NULL),
	REGULATOR_SUPPLY("avdd_pllu_d2", NULL),
	REGULATOR_SUPPLY("avdd_pllx", NULL),
};

static struct regulator_consumer_supply tps6591x_ldo8_supply_chagall[] = {
	REGULATOR_SUPPLY("vdd_ddr_hs", NULL),
};

#define TPS_PDATA_INIT(_name, _sname, _minmv, _maxmv, _supply_reg, _always_on, \
		_boot_on, _apply_uv, _init_uV, _init_enable, _init_apply, _ectrl, _flags) \
static struct tps6591x_regulator_platform_data pdata_##_name##_##_sname = \
{								\
	.regulator = {						\
		.constraints = {				\
			.min_uV = (_minmv)*1000,		\
			.max_uV = (_maxmv)*1000,		\
			.valid_modes_mask = (REGULATOR_MODE_NORMAL |  \
					REGULATOR_MODE_STANDBY), \
			.valid_ops_mask = (REGULATOR_CHANGE_MODE |    \
					REGULATOR_CHANGE_STATUS |  \
					REGULATOR_CHANGE_VOLTAGE), \
			.always_on = _always_on,		\
			.boot_on = _boot_on,			\
			.apply_uV = _apply_uv,			\
		},						\
		.num_consumer_supplies =			\
		ARRAY_SIZE(tps6591x_##_name##_supply_##_sname),	\
		.consumer_supplies = tps6591x_##_name##_supply_##_sname,	\
		.supply_regulator = _supply_reg,		\
	},							\
	.init_uV =  _init_uV * 1000,				\
	.init_enable = _init_enable,				\
	.init_apply = _init_apply,				\
	.ectrl = _ectrl,					\
	.flags = _flags,					\
}

TPS_PDATA_INIT(vdd1,    chagall,  600, 1500, 0, 1, 1, 0, -1, 1, 1, EXT_CTRL_SLEEP_OFF, 0);
TPS_PDATA_INIT(vdd2,    chagall,  600, 1500, 0, 1, 1, 0, -1, 0, 0, 0, 0);
TPS_PDATA_INIT(vddctrl, chagall, 600,  1400, 0, 1, 1, 0, -1, 0, 0, EXT_CTRL_EN1, 0);
TPS_PDATA_INIT(vio,     chagall, 1500, 3300, 0, 1, 1, 0, -1, 0, 0, 0, 0);
TPS_PDATA_INIT(ldo1,    chagall, 1000, 3300, tps6591x_rails(VDD_2), 1, 0, 0, 3200, 1, 1, 0, 0);
TPS_PDATA_INIT(ldo2,    chagall, 1000, 3300, tps6591x_rails(VDD_2), 0, 0, 1, 3200, 1, 1, 0, 0);
TPS_PDATA_INIT(ldo3,    chagall, 1000, 3300, 0, 0, 0, 0, 1800, 0, 0, 0, 0);
TPS_PDATA_INIT(ldo4,    chagall, 1000, 3300, 0, 1, 0, 0, -1, 0, 0, 0, LDO_LOW_POWER_ON_SUSPEND);
TPS_PDATA_INIT(ldo5,    chagall, 1000, 3300, tps6591x_rails(VIO), 0, 0, 0, -1, 0, 0, 0, 0);
TPS_PDATA_INIT(ldo6,    chagall, 1200, 1200, tps6591x_rails(VIO), 0, 0, 0, 1200, 0, 1, 0, 0);
TPS_PDATA_INIT(ldo7,    chagall, 1200, 1200, tps6591x_rails(VIO), 1, 1, 1, -1, 0, 0, EXT_CTRL_SLEEP_OFF, LDO_LOW_POWER_ON_SUSPEND);
TPS_PDATA_INIT(ldo8,    chagall, 1000, 3300, tps6591x_rails(VIO), 1, 0, 0, -1, 0, 0, EXT_CTRL_SLEEP_OFF, LDO_LOW_POWER_ON_SUSPEND);

#if defined(CONFIG_RTC_DRV_TPS6591x)
static struct tps6591x_rtc_platform_data rtc_data = {
	.irq = TEGRA_NR_IRQS + TPS6591X_INT_RTC_ALARM,
	.time = {
		.tm_year = 2000,
		.tm_mon = 0,
		.tm_mday = 1,
		.tm_hour = 0,
		.tm_min = 0,
		.tm_sec = 0,
	},
};

#define TPS_RTC_REG()					\
{						\
	.id	= 0,				\
	.name	= "rtc_tps6591x",		\
	.platform_data = &rtc_data,		\
}
#endif

#define TPS_REG(_id, _name, _sname)				\
{							\
	.id	= TPS6591X_ID_##_id,			\
	.name	= "tps6591x-regulator",			\
	.platform_data	= &pdata_##_name##_##_sname,	\
}

	static struct tps6591x_subdev_info tps_devs_chagall[] = {
		TPS_REG(VIO, vio, chagall),
		TPS_REG(VDD_1, vdd1, chagall),
		TPS_REG(VDD_2, vdd2, chagall),
		TPS_REG(VDDCTRL, vddctrl, chagall),
		TPS_REG(LDO_1, ldo1, chagall),
		TPS_REG(LDO_2, ldo2, chagall),
		TPS_REG(LDO_3, ldo3, chagall),
		TPS_REG(LDO_4, ldo4, chagall),
		TPS_REG(LDO_5, ldo5, chagall),
		TPS_REG(LDO_6, ldo6, chagall),
		TPS_REG(LDO_7, ldo7, chagall),
		TPS_REG(LDO_8, ldo8, chagall),
#if defined(CONFIG_RTC_DRV_TPS6591x)
		TPS_RTC_REG(),
#endif
	};

#define TPS_GPIO_INIT_PDATA(gpio_nr, _init_apply, _sleep_en, _pulldn_en, _output_en, _output_val)	\
	[gpio_nr] = {					\
		.sleep_en	= _sleep_en,	\
		.pulldn_en	= _pulldn_en,	\
		.output_mode_en	= _output_en,	\
		.output_val	= _output_val,	\
		.init_apply	= _init_apply,	\
	}
static struct tps6591x_gpio_init_data tps_gpio_pdata_chagall[] =  {
	TPS_GPIO_INIT_PDATA(0, 1, 0, 0, 1, 1),
	TPS_GPIO_INIT_PDATA(1, 0, 0, 0, 0, 0),
	TPS_GPIO_INIT_PDATA(2, 1, 1, 0, 1, 1),
	TPS_GPIO_INIT_PDATA(3, 0, 0, 0, 0, 0),
	TPS_GPIO_INIT_PDATA(4, 0, 0, 0, 0, 0),
	TPS_GPIO_INIT_PDATA(5, 0, 0, 0, 0, 0),
	TPS_GPIO_INIT_PDATA(6, 1, 0, 0, 1, 1),
	TPS_GPIO_INIT_PDATA(7, 0, 0, 0, 0, 0),
	TPS_GPIO_INIT_PDATA(8, 1, 0, 0, 1, 1),
};

static struct tps6591x_sleep_keepon_data tps_slp_keepon = {
	.clkout32k_keepon = 1,
};

static struct tps6591x_platform_data tps_platform = {
	.irq_base	= TPS6591X_IRQ_BASE,
	.gpio_base	= TPS6591X_GPIO_BASE,
	.dev_slp_en	= true,
	.slp_keepon	= &tps_slp_keepon,
	.use_power_off	= true,
};

static struct i2c_board_info __initdata chagall_regulators[] = {
	{
		I2C_BOARD_INFO("tps6591x", 0x2D),
		.irq		= INT_EXTERNAL_PMU,
		.platform_data	= &tps_platform,
	},
};

/* TPS62361B DC-DC converter */
static struct regulator_consumer_supply tps62361_dcdc_supply[] = {
	REGULATOR_SUPPLY("vdd_core", NULL),
};

static struct tps62360_regulator_platform_data tps62361_pdata = {
	.reg_init_data = {					\
		.constraints = {				\
			.min_uV = 500000,			\
				.max_uV = 1770000,			\
				.valid_modes_mask = (REGULATOR_MODE_NORMAL |  \
						REGULATOR_MODE_STANDBY), \
				.valid_ops_mask = (REGULATOR_CHANGE_MODE |    \
						REGULATOR_CHANGE_STATUS |  \
						REGULATOR_CHANGE_VOLTAGE), \
				.always_on = 1,				\
				.boot_on =  1,				\
				.apply_uV = 0,				\
		},						\
		.num_consumer_supplies = ARRAY_SIZE(tps62361_dcdc_supply), \
			.consumer_supplies = tps62361_dcdc_supply,	\
	},						\
	.en_discharge = true,					\
		.vsel0_gpio = -1,					\
		.vsel1_gpio = -1,					\
		.vsel0_def_state = 1,					\
		.vsel1_def_state = 1,					\
};

static struct i2c_board_info __initdata tps62361_boardinfo[] = {
	{
		I2C_BOARD_INFO("tps62361", 0x60),
		.platform_data	= &tps62361_pdata,
	},
};

int __init chagall_regulator_init(void)
{
	void __iomem *pmc = IO_ADDRESS(TEGRA_PMC_BASE);
	u32 pmc_ctrl;

	/* configure the power management controller to trigger PMU
	 * interrupts when low */

	pmc_ctrl = readl(pmc + PMC_CTRL);
	writel(pmc_ctrl | PMC_CTRL_INTR_LOW, pmc + PMC_CTRL);

	/* The regulator details have complete constraints */
	regulator_has_full_constraints();

	tps_platform.num_subdevs = ARRAY_SIZE(tps_devs_chagall);
	tps_platform.subdevs = tps_devs_chagall;

	/* Enable DEV_SLP and enable sleep on GPIO2 */
	tps_platform.dev_slp_en = true;
	tps_platform.gpio_init_data = tps_gpio_pdata_chagall;
	tps_platform.num_gpioinit_data =
		ARRAY_SIZE(tps_gpio_pdata_chagall);

	pdata_vdd1_chagall.regulator.num_consumer_supplies
		= ARRAY_SIZE(tps6591x_vdd1_supply_chagall_dvt);
	pdata_vdd1_chagall.regulator.consumer_supplies
		= tps6591x_vdd1_supply_chagall_dvt;
	pdata_vdd1_chagall.regulator.constraints.always_on = 0;
	pdata_vdd1_chagall.regulator.constraints.boot_on = 0;
	pdata_vdd1_chagall.init_enable = 0;
	pdata_vdd1_chagall.init_apply = 0;

	pdata_vio_chagall.regulator.num_consumer_supplies
		= ARRAY_SIZE(tps6591x_vio_supply_chagall_mp);
	pdata_vio_chagall.regulator.consumer_supplies
		= tps6591x_vio_supply_chagall_mp;

	i2c_register_board_info(4, chagall_regulators, 1);

	/* Register the external core regulator */
	i2c_register_board_info(4, tps62361_boardinfo, 1);

	return 0;
}


/* EN_REG from TPS6591X_GPIO_0*/
static struct regulator_consumer_supply fixed_reg_en_reg_supply[] = {
	REGULATOR_SUPPLY("vdd_5v0_sby", NULL),
};

/* EN_VDD_SOC from TPS6591X_GPIO_2*/
static struct regulator_consumer_supply fixed_reg_en_vdd_soc_supply[] = {
	REGULATOR_SUPPLY("vdd_1v2_core_tegra", NULL),
};

/* EN_5V0_SYS from TPS6591X_GPIO_8*/
static struct regulator_consumer_supply fixed_reg_en_5v0_sys_supply[] = {
	REGULATOR_SUPPLY("vdd_5v0_sys", NULL),
	REGULATOR_SUPPLY("vdd_spk_amp", NULL),
	REGULATOR_SUPPLY("vdd_flash_sw", NULL),
};


/* EN_3V3_SYS from TPS6591X_GPIO_6*/
static struct regulator_consumer_supply fixed_reg_en_3v3_sys_mp_supply[] = {
	REGULATOR_SUPPLY("vmmc", NULL),
	REGULATOR_SUPPLY("vdd_3v3_sys", NULL),
	REGULATOR_SUPPLY("vdd_lvds", NULL),
	REGULATOR_SUPPLY("pwrdet_pex_ctl", NULL),
	REGULATOR_SUPPLY("avdd_hdmi", NULL),
	REGULATOR_SUPPLY("avdd_usb", NULL),
	REGULATOR_SUPPLY("hvdd_sata", NULL),
	REGULATOR_SUPPLY("pwrdet_nand", NULL),
	REGULATOR_SUPPLY("avddio_audio", NULL),
	REGULATOR_SUPPLY("vdd_3v3_devices", NULL),
	REGULATOR_SUPPLY("vdd_nct1008", NULL),
	REGULATOR_SUPPLY("ls_vdd", NULL),
	REGULATOR_SUPPLY("vdd_3v3_cap_vdd", NULL),
	REGULATOR_SUPPLY("vdd_gps_3v3", NULL),
	REGULATOR_SUPPLY("vdd_wlan_3v3", NULL),
};

/* EN_VDD_PNL1 from AP GPIO LCD_M1 W01*/
static struct regulator_consumer_supply fixed_reg_en_vdd_pnl1_supply[] = {
	REGULATOR_SUPPLY("vdd_lcd_panel", NULL),
};

/* VBAT_BL_EN from AP GPIO LCD_PWR2 C06*/
static struct regulator_consumer_supply fixed_reg_vbat_bl_en_supply[] = {
	REGULATOR_SUPPLY("vdd_backlight", NULL),
};

/* TS_EN* from AP GPIO ULPI_DATA7 O00*/
static struct regulator_consumer_supply fixed_reg_ts_en_supply[] = {
	REGULATOR_SUPPLY("vdd_5v0_ts", NULL),
};

/* CAM1_LDO4_EN from AP GPIO GMI_AD9 H01*/
static struct regulator_consumer_supply fixed_reg_cam1_ldo4_en_supply[] = {
	REGULATOR_SUPPLY("vdd_cameraisp_1v1", NULL),
};

/* CAM1_LDO3_EN from AP GPIO DAP3_SCLK P03*/
static struct regulator_consumer_supply fixed_reg_cam1_ldo3_en_supply[] = {
	REGULATOR_SUPPLY("vdd_2v8_5m", NULL),
};

/* CAM1_LDO2_EN from AP GPIO DAP3_DIN P01*/
static struct regulator_consumer_supply fixed_reg_cam1_ldo2_en_supply[] = {
	REGULATOR_SUPPLY("avdd_2v8_5m", NULL),
};

/* CAM1_LDO1_EN from AP GPIO KB_ROW6 R06*/
static struct regulator_consumer_supply fixed_reg_cam1_ldo1_en_supply[] = {
	REGULATOR_SUPPLY("vddio_1v8_5m", NULL),
};

/* CAM2_LDO3_EN from AP GPIO GPIO_PBB7 BB04*/
static struct regulator_consumer_supply fixed_reg_cam2_ldo3_en_supply[] = {
	REGULATOR_SUPPLY("avdd_2v8_2m", NULL),
};

/* CAM2_LDO2_EN from AP GPIO GPIO_PBB4 BB04*/
static struct regulator_consumer_supply fixed_reg_cam2_ldo2_en_supply[] = {
	REGULATOR_SUPPLY("vddio_1v8_2m", NULL),
};

/* CAM2_LDO1_EN from AP GPIO KB_ROW7 R07*/
static struct regulator_consumer_supply fixed_reg_cam2_ldo1_en_supply[] = {
	REGULATOR_SUPPLY("vdd_1v8_2m", NULL),
};

/* SEN_1V8_EN from AP GPIO SDMMC3_DAT4 D01*/
static struct regulator_consumer_supply fixed_reg_sen_1v8_en_supply[] = {
	REGULATOR_SUPPLY("vdd_1v8_sensor", NULL),
};

/* SEN_3V3_EN from AP GPIO SPDIF_OUT K05*/
static struct regulator_consumer_supply fixed_reg_sen_3v3_en_supply[] = {
	REGULATOR_SUPPLY("vdd_3v3_sensor", NULL),
};

/*EN_USB1_VBUS_OC*/
static struct regulator_consumer_supply fixed_reg_en_usb1_vbus_oc_supply[] = {
	REGULATOR_SUPPLY("vdd_vbus_micro_usb", NULL),
	REGULATOR_SUPPLY("vdd_5v0_usb1", NULL),
};

/*EN_USB3_VBUS_OC*/
static struct regulator_consumer_supply fixed_reg_en_usb3_vbus_oc_supply[] = {
	REGULATOR_SUPPLY("vdd_vbus_typea_usb", NULL),
	REGULATOR_SUPPLY("vdd_5v0_usb3", NULL),
};

/* EN_3V3_FUSE from AP GPIO LCD_PWR1 C01*/
static struct regulator_consumer_supply fixed_reg_en_3v3_fuse_supply[] = {
	REGULATOR_SUPPLY("vdd_fuse", NULL),
	REGULATOR_SUPPLY("vdd_3v3_fuse_tegra", NULL),
};

/* EN_VDDIO_VID_OC from AP GPIO DAP3_DOUT P02*/
static struct regulator_consumer_supply fixed_reg_en_vddio_vid_oc_supply[] = {
	REGULATOR_SUPPLY("vdd_hdmi_con", NULL),
	REGULATOR_SUPPLY("vdd_5v0_vid", NULL),
};


/* EN_1V8_DMIC */
static struct regulator_consumer_supply fixed_reg_en_1v8_dmic_supply[] = {
	REGULATOR_SUPPLY("vdd_dmic", NULL),
};

/* EN_5V0_AMIC */
static struct regulator_consumer_supply fixed_reg_en_5v0_amic_supply[] = {
	REGULATOR_SUPPLY("vdd_amic", NULL),
};

/* EN_3v8_3g_BUCK */
static struct regulator_consumer_supply fixed_reg_en_3v8_3g_buck_supply[] = {
	REGULATOR_SUPPLY("vbat_3g", NULL),
	REGULATOR_SUPPLY("vbat_pa_dcdc", NULL),
	REGULATOR_SUPPLY("vbat_pmu", NULL),
};

/* Macro for defining fixed regulator sub device data */
#define FIXED_SUPPLY(_name) "fixed_reg_"#_name
#define FIXED_REG_OD(_id, _var, _name, _in_supply, _always_on,		\
		_boot_on, _gpio_nr, _active_high, _boot_state,		\
		_millivolts, _od_state)					\
	static struct regulator_init_data ri_data_##_var =		\
	{								\
		.supply_regulator = _in_supply,				\
		.num_consumer_supplies =				\
			ARRAY_SIZE(fixed_reg_##_name##_supply),		\
		.consumer_supplies = fixed_reg_##_name##_supply,	\
		.constraints = {					\
			.valid_modes_mask = (REGULATOR_MODE_NORMAL |	\
					REGULATOR_MODE_STANDBY),	\
			.valid_ops_mask = (REGULATOR_CHANGE_MODE |	\
					REGULATOR_CHANGE_STATUS |	\
					REGULATOR_CHANGE_VOLTAGE),	\
			.always_on = _always_on,			\
			.boot_on = _boot_on,				\
		},							\
	};								\
	static struct fixed_voltage_config fixed_reg_##_var##_pdata =	\
	{								\
		.supply_name = FIXED_SUPPLY(_name),			\
		.microvolts = _millivolts * 1000,			\
		.gpio = _gpio_nr,					\
		.enable_high = _active_high,				\
		.enabled_at_boot = _boot_state,				\
		.init_data = &ri_data_##_var,				\
		.gpio_is_open_drain = _od_state,			\
	};								\
	static struct platform_device fixed_reg_##_var##_dev = {	\
		.name   = "reg-fixed-voltage",				\
		.id     = _id,						\
		.dev    = {						\
			.platform_data = &fixed_reg_##_var##_pdata,	\
		},							\
	}

#define FIXED_REG(_id, _var, _name, _in_supply, _always_on, _boot_on,	\
		 _gpio_nr, _active_high, _boot_state, _millivolts)	\
	FIXED_REG_OD(_id, _var, _name, _in_supply, _always_on, _boot_on,  \
		_gpio_nr, _active_high, _boot_state, _millivolts, false)

FIXED_REG(0, en_reg,           en_reg,          NULL,          1, 1, TPS6591X_GPIO_0, true,  1, 5000);
FIXED_REG(1, en_vdd_soc,       en_vdd_soc,      NULL,          1, 1, TPS6591X_GPIO_2, true,  1, 1200);
FIXED_REG(2, en_5v0_sys,       en_5v0_sys,      NULL,          0, 0, TPS6591X_GPIO_8, true,  0, 5000);
FIXED_REG(3, en_3v3_sys_mp,    en_3v3_sys_mp,   NULL,          1, 0, TPS6591X_GPIO_6, true,  0, 3300);
FIXED_REG(4, en_vdd_pnl1,      en_vdd_pnl1,     FIXED_SUPPLY(en_3v3_sys), 0, 0, TEGRA_GPIO_PW1,  true,  1, 3300);
FIXED_REG(5, vbat_bl_en,       vbat_bl_en,      NULL,          0, 0, TEGRA_GPIO_PC6,  true,  1, 5000);
FIXED_REG(6, ts_en,            ts_en,           FIXED_SUPPLY(en_5v0_sys), 0, 0, TEGRA_GPIO_PO0,  false, 0, 5000);
FIXED_REG(7, cam1_ldo4_en,     cam1_ldo4_en,    FIXED_SUPPLY(en_5v0_sys), 0, 0, TEGRA_GPIO_PH1,  true,  0, 1100);
FIXED_REG(8, cam1_ldo3_en,     cam1_ldo3_en,    FIXED_SUPPLY(en_3v3_sys), 0, 0, TEGRA_GPIO_PP3,  true,  0, 2800);
FIXED_REG(9, cam1_ldo2_en,     cam1_ldo2_en,    FIXED_SUPPLY(en_3v3_sys), 0, 0, TEGRA_GPIO_PP1,  true,  0, 2800);
FIXED_REG(10, cam1_ldo1_en,    cam1_ldo1_en,    tps6591x_rails(VIO),  0, 0, TEGRA_GPIO_PR6,  true,  0, 1800);
FIXED_REG(11, cam2_ldo3_en,    cam2_ldo3_en,    FIXED_SUPPLY(en_3v3_sys), 0, 0, TEGRA_GPIO_PBB7, true,  0, 2800);
FIXED_REG(12, cam2_ldo2_en,    cam2_ldo2_en,    tps6591x_rails(VIO),  0, 0, TEGRA_GPIO_PBB4, true,  0, 1800);
FIXED_REG(13, cam2_ldo1_en,    cam2_ldo1_en,    tps6591x_rails(VIO),  0, 0, TEGRA_GPIO_PR7,  true,  0, 1800);
FIXED_REG(14, sen_1v8_en,      sen_1v8_en,      tps6591x_rails(VIO),  0, 0, TEGRA_GPIO_PD1,  true,  0, 1800);
FIXED_REG(15, sen_3v3_en,      sen_3v3_en,      FIXED_SUPPLY(en_3v3_sys), 0, 0, TEGRA_GPIO_PK5,  true,  0, 3300);
FIXED_REG(16, en_3v3_fuse,     en_3v3_fuse,     FIXED_SUPPLY(en_3v3_sys), 0, 0, TEGRA_GPIO_PC1,  true,  0, 3300);
FIXED_REG(17, en_vddio_vid_oc, en_vddio_vid_oc, FIXED_SUPPLY(en_5v0_sys), 0, 0, TEGRA_GPIO_PP2,  true,  0, 5000);
FIXED_REG(18, en_usb1_vbus_oc, en_usb1_vbus_oc, FIXED_SUPPLY(en_5v0_sys), 0, 0, TEGRA_GPIO_PDD3, true,  0, 5000);
FIXED_REG(19, en_usb3_vbus_oc, en_usb3_vbus_oc, FIXED_SUPPLY(en_5v0_sys), 0, 0, TEGRA_GPIO_PCC6, true,  0, 5000);
FIXED_REG(20, en_1v8_dmic,     en_1v8_dmic,     tps6591x_rails(VIO),      0, 0, TEGRA_GPIO_PX0,  true,  0, 1800);
FIXED_REG(21, en_3v8_3g_buck,  en_3v8_3g_buck,  NULL,                     0, 0, TEGRA_GPIO_PK7,  true,  0, 3800);
FIXED_REG(25, en_5v0_amic,     en_5v0_amic,     tps6591x_rails(VIO),  0, 0, TEGRA_GPIO_PO1,  false,  0, 1800);

/*
 * Creating the fixed/gpio-switch regulator device tables for different boards
 */
#define ADD_FIXED_REG(_name)	(&fixed_reg_##_name##_dev)


static struct platform_device *fixed_reg_devs_chagall_mp[] = {
	ADD_FIXED_REG(en_reg),
	ADD_FIXED_REG(en_vdd_soc),
	ADD_FIXED_REG(en_5v0_sys),
	ADD_FIXED_REG(en_3v3_sys_mp),
	ADD_FIXED_REG(en_vdd_pnl1),
	ADD_FIXED_REG(vbat_bl_en),
	ADD_FIXED_REG(ts_en),
	ADD_FIXED_REG(cam1_ldo4_en),
	ADD_FIXED_REG(cam1_ldo3_en),
	ADD_FIXED_REG(cam1_ldo2_en),
	ADD_FIXED_REG(cam1_ldo1_en),
	ADD_FIXED_REG(cam2_ldo3_en),
	ADD_FIXED_REG(cam2_ldo2_en),
	ADD_FIXED_REG(cam2_ldo1_en),
	ADD_FIXED_REG(sen_1v8_en),
	ADD_FIXED_REG(sen_3v3_en),
	ADD_FIXED_REG(en_3v3_fuse),
	ADD_FIXED_REG(en_vddio_vid_oc),
	ADD_FIXED_REG(en_usb1_vbus_oc),
	ADD_FIXED_REG(en_usb3_vbus_oc),
	ADD_FIXED_REG(en_1v8_dmic),
	ADD_FIXED_REG(en_3v8_3g_buck),
	ADD_FIXED_REG(en_5v0_amic),
};

int __init chagall_fixed_regulator_init(void)
{
	int i;
	struct platform_device **fixed_reg_devs;
	int    nfixreg_devs;


	nfixreg_devs = ARRAY_SIZE(fixed_reg_devs_chagall_mp);
	fixed_reg_devs = fixed_reg_devs_chagall_mp;

	ri_data_en_vdd_pnl1.supply_regulator
		= FIXED_SUPPLY(en_3v3_sys_mp);
	ri_data_cam1_ldo3_en.supply_regulator
		= FIXED_SUPPLY(en_3v3_sys_mp);
	ri_data_cam1_ldo2_en.supply_regulator
		= FIXED_SUPPLY(en_3v3_sys_mp);
	ri_data_cam2_ldo3_en.supply_regulator
		= FIXED_SUPPLY(en_3v3_sys_mp);
	ri_data_sen_3v3_en.supply_regulator
		= FIXED_SUPPLY(en_3v3_sys_mp);
	ri_data_en_3v3_fuse.supply_regulator
		= FIXED_SUPPLY(en_3v3_sys_mp);

	for (i = 0; i < nfixreg_devs; ++i) {
		struct fixed_voltage_config *fixed_reg_pdata =
				fixed_reg_devs[i]->dev.platform_data;
		int gpio_nr = fixed_reg_pdata->gpio;
		if (gpio_nr < TEGRA_NR_GPIOS)
			tegra_gpio_enable(gpio_nr);
	}
	return platform_add_devices(fixed_reg_devs, nfixreg_devs);
}
subsys_initcall_sync(chagall_fixed_regulator_init);

static void chagall_board_suspend(int lp_state, enum suspend_stage stg)
{
	if ((lp_state == TEGRA_SUSPEND_LP1) && (stg == TEGRA_SUSPEND_BEFORE_CPU))
		tegra_console_uart_suspend();
}

static void chagall_board_resume(int lp_state, enum resume_stage stg)
{
	if ((lp_state == TEGRA_SUSPEND_LP1) && (stg == TEGRA_RESUME_AFTER_CPU))
		tegra_console_uart_resume();
}

static struct tegra_suspend_platform_data chagall_suspend_data = {
	.cpu_timer	= 2000,
	.cpu_off_timer	= 200,
	.suspend_mode	= TEGRA_SUSPEND_LP0,
	.core_timer	= 0x7e7e,
	.core_off_timer = 0,
	.corereq_high	= true,
	.sysclkreq_high	= true,
	.cpu_lp2_min_residency = 2000,
	.board_suspend = chagall_board_suspend,
	.board_resume = chagall_board_resume,
};

int __init chagall_suspend_init(void)
{
	chagall_suspend_data.corereq_high = true;

	tegra_init_suspend(&chagall_suspend_data);
	return 0;
}

#ifdef CONFIG_TEGRA_EDP_LIMITS

int __init chagall_edp_init(void)
{
	unsigned int regulator_mA;

	regulator_mA = get_maximum_cpu_current_supported();
	if (!regulator_mA) {
		regulator_mA = 6000; /* regular T30/s */
	}
	pr_info("%s: CPU regulator %d mA\n", __func__, regulator_mA);

	tegra_init_cpu_edp_limits(regulator_mA);
	return 0;
}
#endif

