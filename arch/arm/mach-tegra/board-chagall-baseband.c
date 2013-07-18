/*
 * arch/arm/mach-tegra/board-chagall-baseband.c
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

#include <linux/resource.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/err.h>
#include <linux/wakelock.h>
#include <linux/platform_data/tegra_usb.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <mach/pinmux.h>
#include <mach/usb_phy.h>
#include <mach/tegra_usb_modem_power.h>
#include "board.h"
#include "devices.h"
#include "gpio-names.h"
#include "board-chagall.h"
#include <linux/regulator/consumer.h>

extern void usb2_autosuspend(int en);

/*Chagall 3G*/
#define CHAGALL_3G_RESIN_N TEGRA_GPIO_PP0
#define CHAGALL_3G_PWRDWN_N TEGRA_GPIO_PV7
#define CHAGALL_3G_DISABLE_N TEGRA_GPIO_PV6
#define CHAGALL_3G_R_ON TEGRA_GPIO_PO2
#define CHAGALL_3G_RESOUT TEGRA_GPIO_PX5
#define CHAGALL_3G_WW_WAKE TEGRA_GPIO_PN2
#define CHAGALL_SIM_DET TEGRA_GPIO_PQ2

static struct wake_lock mdm_wake_lock;

static struct gpio modem_gpios[] = {
	{CHAGALL_3G_RESIN_N, GPIOF_OUT_INIT_LOW, "CHAGALL_3G_RESIN_N"},
	{CHAGALL_3G_PWRDWN_N, GPIOF_OUT_INIT_LOW, "CHAGALL_3G_PWRDWN_N"},
	{CHAGALL_3G_DISABLE_N, GPIOF_OUT_INIT_LOW, "CHAGALL_3G_DISABLE_N"},
	{CHAGALL_3G_R_ON, GPIOF_OUT_INIT_LOW, "CHAGALL_3G_R_ON"},
	{CHAGALL_3G_RESOUT, GPIOF_IN, "CHAGALL_3G_RESOUT"},
    {CHAGALL_SIM_DET, GPIOF_IN, "CHAGALL_SIM_DET"},
};

static struct regulator *chagall_3g_buck = NULL;

int chagall_mu739_modem_deinit(void)
{

	pr_info("%s: for WiFi SKU\n", __func__);

	tegra_gpio_enable(CHAGALL_3G_PWRDWN_N);
	if ( gpio_request(CHAGALL_3G_PWRDWN_N, "3G_pwrdwn_n") < 0 ) {
		pr_err("%s: gpio_request failed for gpio: CHAGALL_3G_PWRDWN_N\n", __func__);
		return -1;
	}
	gpio_direction_output(CHAGALL_3G_PWRDWN_N, 0);
	gpio_export(CHAGALL_3G_PWRDWN_N, true);

	tegra_gpio_enable(CHAGALL_3G_RESIN_N);
	if ( gpio_request(CHAGALL_3G_RESIN_N, "3G_resin_n") < 0 ) {
		pr_err("%s: gpio_request failed for gpio: CHAGALL_3G_RESIN_N\n", __func__);
		return -1;
	}
	gpio_direction_output(CHAGALL_3G_RESIN_N, 0);
	gpio_export(CHAGALL_3G_RESIN_N, true);

	tegra_gpio_enable(CHAGALL_3G_DISABLE_N);
	if ( gpio_request(CHAGALL_3G_DISABLE_N, "3G_disable_n") < 0 ) {
		pr_err("%s: gpio_request failed for gpio: CHAGALL_3G_DISABLE_N\n", __func__);
		return -1;
	}
	gpio_direction_output(CHAGALL_3G_DISABLE_N, 0);
	gpio_export(CHAGALL_3G_DISABLE_N, true);

	tegra_gpio_enable(CHAGALL_3G_R_ON);
	if ( gpio_request(CHAGALL_3G_R_ON, "3G_r_on") < 0 ) {
		pr_err("%s: gpio_request failed for gpio: CHAGALL_3G_R_ON\n", __func__);
		return -1;
	}
	gpio_direction_output(CHAGALL_3G_R_ON,0);
	gpio_export(CHAGALL_3G_R_ON, true);

	tegra_gpio_enable(CHAGALL_3G_RESOUT);
	if ( gpio_request(CHAGALL_3G_RESOUT, "3G_resout") < 0 ) {
		pr_err("%s: gpio_request failed for gpio: CHAGALL_3G_RESOUT\n", __func__);
		return -1;
	}
	gpio_direction_output(CHAGALL_3G_RESOUT, 0);
	gpio_export(CHAGALL_3G_RESOUT, true);

	tegra_gpio_enable(CHAGALL_3G_WW_WAKE);
	if ( gpio_request(CHAGALL_3G_WW_WAKE, "3G_wake") < 0 ) {
		pr_err("%s: gpio_request failed for gpio: CHAGALL_3G_WW_WAKE\n", __func__);
		return -1;
	}
	gpio_direction_output(CHAGALL_3G_WW_WAKE, 0);
	gpio_export(CHAGALL_3G_WW_WAKE, true);

	return 0;
}

static int baseband_get_status(void)
{
	return gpio_get_value(CHAGALL_SIM_DET);
}

static void baseband_start(void)
{
	/*
	 *  Leave baseband powered OFF.
	 *  User-space daemons will take care of powering it up.
	 */

	regulator_enable(chagall_3g_buck);
	gpio_set_value(CHAGALL_3G_PWRDWN_N, 1);
	gpio_set_value(CHAGALL_3G_RESIN_N, 1);
	msleep(5);
	gpio_set_value(CHAGALL_3G_R_ON, 1);
	udelay(100);
	gpio_set_value(CHAGALL_3G_R_ON, 0);
	msleep(200);
	gpio_set_value(CHAGALL_3G_DISABLE_N, 1);
}

static void baseband_reset(void)
{
	/* Initiate power cycle on baseband sub system */

	if (!baseband_get_status()) {
		usb2_autosuspend(1);

		printk(KERN_INFO "%s: reset module\n", __func__);
		disable_irq(gpio_to_irq(CHAGALL_3G_WW_WAKE));
		gpio_direction_output(CHAGALL_3G_RESOUT, 0);
		gpio_direction_output(CHAGALL_3G_WW_WAKE, 0);
		mdelay(50);
		gpio_set_value(CHAGALL_3G_R_ON, 0);
		gpio_set_value(CHAGALL_3G_DISABLE_N, 0);
		gpio_set_value(CHAGALL_3G_PWRDWN_N, 0);
		gpio_set_value(CHAGALL_3G_RESIN_N, 0);
		regulator_disable(chagall_3g_buck);

		mdelay(50);

		gpio_direction_input(CHAGALL_3G_RESOUT);
		gpio_direction_input(CHAGALL_3G_WW_WAKE);

		regulator_enable(chagall_3g_buck);
		gpio_set_value(CHAGALL_3G_PWRDWN_N, 1);
		gpio_set_value(CHAGALL_3G_RESIN_N, 1);
		mdelay(5);
		gpio_set_value(CHAGALL_3G_R_ON, 1);
		udelay(100);
		gpio_set_value(CHAGALL_3G_R_ON, 0);
		mdelay(200);
		gpio_set_value(CHAGALL_3G_DISABLE_N, 1);

		mdelay(50);
		usb2_autosuspend(0);
		usb2_autosuspend(1);

		enable_irq(gpio_to_irq(CHAGALL_3G_WW_WAKE));

		wake_lock_timeout(&mdm_wake_lock, HZ * 10);

	}
}

static int baseband_init(void)
{
	int ret;

	ret = gpio_request_array(modem_gpios, ARRAY_SIZE(modem_gpios));
	if (ret)
		return ret;

	chagall_3g_buck = regulator_get(NULL, "vbat_3g");
	if (IS_ERR(chagall_3g_buck)) {
		pr_err("fail to get 3g buck\r\n");
		return -1;
	}

	tegra_gpio_enable(CHAGALL_3G_RESIN_N);
	tegra_gpio_enable(CHAGALL_3G_PWRDWN_N);
	tegra_gpio_enable(CHAGALL_3G_DISABLE_N);
	tegra_gpio_enable(CHAGALL_3G_R_ON);
	tegra_gpio_enable(CHAGALL_3G_RESOUT);
	tegra_gpio_enable(CHAGALL_SIM_DET);

	/* export GPIO for user space access through sysfs */
	gpio_export(CHAGALL_3G_RESIN_N, true);
	gpio_export(CHAGALL_3G_PWRDWN_N, true);
	gpio_export(CHAGALL_3G_DISABLE_N, true);
	gpio_export(CHAGALL_3G_R_ON, true);
	gpio_export(CHAGALL_3G_RESOUT, true);
	gpio_export(CHAGALL_SIM_DET, true);

	wake_lock_init(&mdm_wake_lock, WAKE_LOCK_SUSPEND, "mdm_lock");

	return 0;
}

static const struct tegra_modem_operations baseband_operations = {
	.init = baseband_init,
	.start = baseband_start,
	.reset = baseband_reset,
	.get_status = baseband_get_status,
};

static struct tegra_usb_modem_power_platform_data baseband_pdata = {
	.ops = &baseband_operations,
	.wake_gpio = CHAGALL_3G_WW_WAKE,
	.wake_irq_flags = IRQF_TRIGGER_RISING,
	.boot_gpio = -1,
	.boot_irq_flags = IRQF_TRIGGER_NONE,
	.autosuspend_delay = 6000,
	.short_autosuspend_delay = 0,
};

static struct platform_device chagall_baseband_device = {
	.name = "tegra_usb_modem_power",
	.id = -1,
	.dev = {
		.platform_data = &baseband_pdata,
	},
};

int __init chagall_mu739_modem_init(void)
{
	printk(KERN_INFO "%s\n", __func__);
	platform_device_register(&chagall_baseband_device);
	return 0;
}
