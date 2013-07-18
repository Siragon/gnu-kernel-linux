/*
 * arch/arm/mach-tegra/board-chagall.h
 *
 * Copyright (c) 2011, NVIDIA Corporation.
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

#ifndef _MACH_TEGRA_BOARD_CHAGALL_H
#define _MACH_TEGRA_BOARD_CHAGALL_H

#include <mach/gpio.h>
#include <mach/irqs.h>
#include <linux/mfd/tps6591x.h>
#include <linux/mfd/ricoh583.h>
/* Processor Board  ID */
#define BOARD_E1291   0x0C5B

/* SKU Information */
#define BOARD_SKU_B11	0xb11

#define SKU_DCDC_TPS62361_SUPPORT	0x1
#define SKU_SLT_ULPI_SUPPORT		0x2
#define SKU_T30S_SUPPORT		0x4

#define SKU_MEMORY_TYPE_BIT            0x3
#define SKU_MEMORY_TYPE_MASK          0x7
/* If BOARD_E1291 */
#define SKU_MEMORY_SAMSUNG_K4P8G304EB_FGC1             0x0
#define SKU_MEMORY_ELPIDA_EDB8132B2MA_8D_F             0x1
#define SKU_MEMORY_SAMSUNG_K4P8G304EB_FGC2             0x2
#define SKU_MEMORY_HYNIX_H9TCNNN8JDMMPR_NGM            0x3

/* If other BOARD_ variants */
#define SKU_MEMORY_CHAGALL_1GB_1R	0x0
#define SKU_MEMORY_CHAGALL_2GB_2R	0x2
#define SKU_MEMORY_CHAGALL_2GB_1R_HYK0	0x4
#define SKU_MEMORY_CHAGALL_2GB_1R_HYH9	0x6
#define SKU_MEMORY_CHAGALL_2GB_1R_HYNIX	0x1
#define MEMORY_TYPE(sku) (((sku) >> SKU_MEMORY_TYPE_BIT) & SKU_MEMORY_TYPE_MASK)

/* External peripheral act as gpio */
/* TPS6591x GPIOs */
#define TPS6591X_GPIO_BASE	TEGRA_NR_GPIOS
#define TPS6591X_GPIO_0		(TPS6591X_GPIO_BASE + TPS6591X_GPIO_GP0)
#define TPS6591X_GPIO_1		(TPS6591X_GPIO_BASE + TPS6591X_GPIO_GP1)
#define TPS6591X_GPIO_2		(TPS6591X_GPIO_BASE + TPS6591X_GPIO_GP2)
#define TPS6591X_GPIO_3		(TPS6591X_GPIO_BASE + TPS6591X_GPIO_GP3)
#define TPS6591X_GPIO_4		(TPS6591X_GPIO_BASE + TPS6591X_GPIO_GP4)
#define TPS6591X_GPIO_5		(TPS6591X_GPIO_BASE + TPS6591X_GPIO_GP5)
#define TPS6591X_GPIO_6		(TPS6591X_GPIO_BASE + TPS6591X_GPIO_GP6)
#define TPS6591X_GPIO_7		(TPS6591X_GPIO_BASE + TPS6591X_GPIO_GP7)
#define TPS6591X_GPIO_8		(TPS6591X_GPIO_BASE + TPS6591X_GPIO_GP8)
#define TPS6591X_GPIO_END	(TPS6591X_GPIO_BASE + TPS6591X_GPIO_NR)


/* WM8903 GPIOs */
#define CHAGALL_GPIO_WM8903(_x_)	(TPS6591X_GPIO_END + (_x_))
#define CHAGALL_GPIO_WM8903_END		CHAGALL_GPIO_WM8903(4)

/* Audio-related GPIOs */
#define TEGRA_GPIO_CDC_IRQ		TEGRA_GPIO_PW3
#define TEGRA_GPIO_SPKR_EN		CHAGALL_GPIO_WM8903(2)
#define TEGRA_GPIO_HP_DET		TEGRA_GPIO_PW2
#define TEGRA_GPIO_LINEOUT_DET         TEGRA_GPIO_PS3

/* CAMERA RELATED GPIOs on CHAGALL */
#define CAM_MCLK_GPIO		TEGRA_GPIO_PCC0
#define CAM1_RST			TEGRA_GPIO_PBB3
#define CAM1_PWDN			TEGRA_GPIO_PBB5
#define CAM2_RST			TEGRA_GPIO_PBB0
#define CAM2_PWDN			TEGRA_GPIO_PBB6
#define GP42_8M_HOST_INT	TEGRA_GPIO_PCC1

/* MCU gpios*/
#define MCU_INTERRUPT    TEGRA_GPIO_PC7
#define MCU_SYS_SUS      TEGRA_GPIO_PB1
#define MCU_RESET        TEGRA_GPIO_PD2
#define MCU_WAKE         TEGRA_GPIO_PB0

/* common pins( backlight ) for all display boards */
#define chagall_bl_enb			TEGRA_GPIO_PH2
#define chagall_bl_pwm			TEGRA_GPIO_PH0
#define chagall_hdmi_hpd			TEGRA_GPIO_PN7

/* Battery GPIO */
#define CHARGER_AC_PRESENT_GPIO	TEGRA_GPIO_PV1

/* Display GPIO */
#define chagall_lvds_shutdown		TEGRA_GPIO_PN6
#define chagall_lcd_cabc		TEGRA_GPIO_PO6
#define chagall_lcd_color_engine	TEGRA_GPIO_PO7
/* common pins( backlight ) for all display boards */
#define chagall_bl_enb			TEGRA_GPIO_PH2
#define chagall_bl_pwm			TEGRA_GPIO_PH0
#define chagall_hdmi_hpd		TEGRA_GPIO_PN7

/*****************Interrupt tables ******************/
/* External peripheral act as interrupt controller */
/* TPS6591x IRQs */
#define TPS6591X_IRQ_BASE	TEGRA_NR_IRQS
#define TPS6591X_IRQ_END	(TPS6591X_IRQ_BASE + 18)
#define DOCK_DETECT_GPIO TEGRA_GPIO_PU4

int chagall_regulator_init(void);
int chagall_suspend_init(void);
int chagall_sdhci_init(void);
int chagall_pinmux_init(void);
int chagall_panel_init(void);
int chagall_sensors_init(bool is_wifi_sku);
int chagall_keys_init(void);
int chagall_pins_state_init(void);
int chagall_emc_init(void);
int chagall_edp_init(void);
int chagall_mu739_modem_init(void);
int chagall_mu739_modem_deinit(void);

#define MPU_TYPE_MPU3050	1
#define MPU_TYPE_MPU6050	2
#define MPU_GYRO_TYPE		MPU_TYPE_MPU3050
#define MPU_GYRO_IRQ_GPIO	TEGRA_GPIO_PX1
#define MPU_GYRO_ADDR		0x68
#define MPU_GYRO_BUS_NUM	2
#define MPU_GYRO_ORIENTATION	{ 0, 1, 0, 1, 0, 0, 0, 0, -1 }
#define MPU_ACCEL_NAME		"kxtf9"
#define MPU_ACCEL_IRQ_GPIO	0 /* DISABLE ACCELIRQ:  TEGRA_GPIO_PL1 */
#define MPU_ACCEL_ADDR		0x0F
#define MPU_ACCEL_BUS_NUM	2
#define MPU_ACCEL_ORIENTATION	{ 1, 0, 0, 0, -1, 0, 0, 0, -1 }
#define MPU_COMPASS_NAME	"ak8975"
#define MPU_COMPASS_IRQ_GPIO	0
#define MPU_COMPASS_ADDR	0x0C
#define MPU_COMPASS_BUS_NUM	2
#define MPU_COMPASS_ORIENTATION	{ 0, 1, 0, 1, 0, 0, 0, 0, -1 }

#define TDIODE_OFFSET	(10000)	/* in millicelsius */


enum power_type{
 WIFI_POWER_ENABLE=0,
 BT_POWER_ENABLE=1
};

#endif
