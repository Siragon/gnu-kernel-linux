/*
 * arch/arm/mach-tegra/board-chagall.c
 *
 * Copyright (c) 2011-2012, NVIDIA Corporation. All rights reserved.
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

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/ctype.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/serial_8250.h>
#include <linux/i2c.h>
#include <linux/i2c/panjit_ts.h>
#include <linux/dma-mapping.h>
#include <linux/delay.h>
#include <linux/i2c-tegra.h>
#include <linux/gpio.h>
#include <linux/input.h>
#include <linux/platform_data/tegra_usb.h>
#include <linux/spi/spi.h>
#include <linux/tegra_uart.h>
#include <linux/memblock.h>
#include <linux/spi-tegra.h>
#include <linux/rfkill-gpio.h>

#include <sound/wm8903.h>
#include <media/tegra_dtv.h>

#include <mach/clk.h>
#include <mach/iomap.h>
#include <mach/io_dpd.h>
#include <mach/irqs.h>
#include <mach/pinmux.h>
#include <mach/iomap.h>
#include <mach/io.h>
#include <mach/i2s.h>
#include <mach/tegra_asoc_pdata.h>
#include <mach/tegra_wm8903_pdata.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <mach/usb_phy.h>
#include <mach/thermal.h>
#include <mach/pci.h>
#include <mach/tegra_fiq_debugger.h>
#ifdef CONFIG_SND_SOC_FM34
#include <sound/fm34.h>
#endif

#include "board.h"
#include "clock.h"
#include "board-chagall.h"
#include "board-touch.h"
#include "devices.h"
#include "gpio-names.h"
#include "fuse.h"
#include "pm.h"
#include "baseband-xmm-power.h"
#include "wdt-recovery.h"
#include <linux/i2c/goodix_touch.h>
#include <linux/timed_gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/switch.h>

#define GPS_1V8_REG "vdd_gps_1v8"
#define GPS_3V3_REG "vdd_gps_3v3"
#define GPS_LDO_REG "en_gps_1v8_3v3"

int enable_wireless_regulator(int enable, int power_type);

#define DOCK_DET_IRQ TEGRA_GPIO_PS4
#define DOCK_CONNECTED "1"
#define DOCK_DISCONNECTED "0"

static struct gpio_switch_platform_data dock_switch_data = {
   .name = "dock",
   .gpio = DOCK_DET_IRQ,
   .wakeup = 1,
   .state_off = DOCK_CONNECTED,
   .state_on = DOCK_DISCONNECTED,
   .irqflags = IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
};

static struct platform_device dock_switch_device = {
   .name = "switch-gpio",
   .id = 1,
   .dev = {
       .platform_data = &dock_switch_data,
   }
};

static int chagall_dock_init(void)
{
   printk(KERN_INFO "chagall_dock_init\n");

   tegra_gpio_enable(dock_switch_data.gpio);

   platform_device_register(&dock_switch_device);

   return 0;
}

static void chagall_i2c_bus_reset_devices_gen1(struct tegra_i2c_platform_data *pd, u16 client_addr);

static bool is_wifi_sku = false;
static char *rck = 0;
static bool is_goodix = true;


static struct balanced_throttle throttle_list[] = {
#ifdef CONFIG_TEGRA_THERMAL_THROTTLE
	{
		.id = BALANCED_THROTTLE_ID_TJ,
		.throt_tab_size = 10,
		.throt_tab = {
			{      0, 1000 },
			{ 640000, 1000 },
			{ 640000, 1000 },
			{ 640000, 1000 },
			{ 640000, 1000 },
			{ 640000, 1000 },
			{ 760000, 1000 },
			{ 760000, 1050 },
			{1000000, 1050 },
			{1000000, 1100 },
		},
	},
#endif
#ifdef CONFIG_TEGRA_SKIN_THROTTLE
	{
		.id = BALANCED_THROTTLE_ID_SKIN,
		.throt_tab_size = 6,
		.throt_tab = {
			{ 640000, 1200 },
			{ 640000, 1200 },
			{ 760000, 1200 },
			{ 760000, 1200 },
			{1000000, 1200 },
			{1000000, 1200 },
		},
	},
#endif
};

/* All units are in millicelsius */
static struct tegra_thermal_data thermal_data = {
	.shutdown_device_id = THERMAL_DEVICE_ID_NCT_EXT,
	.temp_shutdown = 90000,

#if defined(CONFIG_TEGRA_EDP_LIMITS) || defined(CONFIG_TEGRA_THERMAL_THROTTLE)
	.throttle_edp_device_id = THERMAL_DEVICE_ID_NCT_EXT,
#endif
#ifdef CONFIG_TEGRA_EDP_LIMITS
	.edp_offset = TDIODE_OFFSET,  /* edp based on tdiode */
	.hysteresis_edp = 3000,
#endif
#ifdef CONFIG_TEGRA_THERMAL_THROTTLE
	.temp_throttle = 85000,
	.tc1 = 0,
	.tc2 = 1,
	.passive_delay = 2000,
#endif
#ifdef CONFIG_TEGRA_SKIN_THROTTLE
	.skin_device_id = THERMAL_DEVICE_ID_SKIN,
	.temp_throttle_skin = 43000,
	.tc1_skin = 0,
	.tc2_skin = 1,
	.passive_delay_skin = 5000,

	.skin_temp_offset = 9793,
	.skin_period = 1100,
	.skin_devs_size = 2,
	.skin_devs = {
		{
			THERMAL_DEVICE_ID_NCT_EXT,
			{
				2, 1, 1, 1,
				1, 1, 1, 1,
				1, 1, 1, 0,
				1, 1, 0, 0,
				0, 0, -1, -7
			}
		},
		{
			THERMAL_DEVICE_ID_NCT_INT,
			{
				-11, -7, -5, -3,
				-3, -2, -1, 0,
				0, 0, 1, 1,
				1, 2, 2, 3,
				4, 6, 11, 18
			}
		},
	},
#endif
};

static struct rfkill_gpio_platform_data chagall_bt_rfkill_pdata[] = {
	{
		.name           = "bt_rfkill",
                .shutdown_gpio  = TEGRA_GPIO_PB2,
                .reset_gpio     = TEGRA_GPIO_PU0,
		.type           = RFKILL_TYPE_BLUETOOTH,
	},
};

static struct platform_device chagall_bt_rfkill_device = {
	.name = "rfkill_gpio",
	.id             = -1,
	.dev = {
		.platform_data = &chagall_bt_rfkill_pdata,
	},
};

static struct resource chagall_bluesleep_resources[] = {
	[0] = {
		.name = "gpio_host_wake",
			.start  = TEGRA_GPIO_PU6,
			.end    = TEGRA_GPIO_PU6,
			.flags  = IORESOURCE_IO,
	},
	[1] = {
		.name = "gpio_ext_wake",
			.start  = TEGRA_GPIO_PU1,
			.end    = TEGRA_GPIO_PU1,
			.flags  = IORESOURCE_IO,
	},
	[2] = {
		.name = "host_wake",
			.start  = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PU6),
			.end    = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PU6),
			.flags  = IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHEDGE,
	},
};

static struct platform_device chagall_bluesleep_device = {
	.name           = "bluesleep",
	.id             = -1,
	.num_resources  = ARRAY_SIZE(chagall_bluesleep_resources),
	.resource       = chagall_bluesleep_resources,
};

static noinline void __init chagall_setup_bluesleep(void)
{
	platform_device_register(&chagall_bluesleep_device);
	return;
}

static __initdata struct tegra_clk_init_table chagall_clk_init_table[] = {
	/* name		parent		rate		enabled */
	{ "pll_m",	NULL,		0,		false},
	{ "hda",	"pll_p",	108000000,	false},
	{ "hda2codec_2x","pll_p",	48000000,	false},
	{ "pwm",	"pll_p",	6375000,	true},
	{ "blink",	"clk_32k",	32768,		true},
	{ "i2s0",	"pll_a_out0",	0,		false},
	{ "i2s1",	"pll_a_out0",	0,		false},
	{ "i2s3",	"pll_a_out0",	0,		false},
	{ "spdif_out",	"pll_a_out0",	0,		false},
	{ "d_audio",	"clk_m",	12000000,	false},
	{ "dam0",	"clk_m",	12000000,	false},
	{ "dam1",	"clk_m",	12000000,	false},
	{ "dam2",	"clk_m",	12000000,	false},
	{ "audio1",	"i2s1_sync",	0,		false},
	{ "audio3",	"i2s3_sync",	0,		false},
	{ "vi_sensor",	"pll_p",	150000000,	false},
	{ "i2c1",	"pll_p",	3200000,	false},
	{ "i2c2",	"pll_p",	3200000,	false},
	{ "i2c3",	"pll_p",	3200000,	false},
	{ "i2c4",	"pll_p",	3200000,	false},
	{ "i2c5",	"pll_p",	3200000,	false},
        { "vi",         "pll_p",        0,              false},
	{ NULL,		NULL,		0,		0},
};

#ifdef CONFIG_SND_SOC_FM34
const static u16 fm34_property[][2] = {
	{0x3FA0, 0x91CB},
	{0x3FB0, 0x501A},
	{0x3FA1, 0x82F3},
	{0x3FB1, 0x02F5},
	{0x3FA2, 0x82B5},
	{0x3FB2, 0x501F},
	{0x3FA3, 0x83F4},
	{0x3FB3, 0x502A},
	{0x3FA4, 0x9666},
	{0x3FB4, 0x503B},
	{0x3FA5, 0x82CC},
	{0x3FB5, 0x5046},
	{0x3FA6, 0xC2CC},
	{0x3FB6, 0x5046},
	{0x3FA7, 0xC210},
	{0x3FB7, 0x504A},
	{0x3FA8, 0x8210},
	{0x3FB8, 0x504A},
	{0x3FA9, 0x8231},
	{0x3FB9, 0x504B},
	{0x3FAA, 0xC231},
	{0x3FBA, 0x504B},
	{0x3FAB, 0x823F},
	{0x3FBB, 0x5056},
	{0x3FAC, 0xC23F},
	{0x3FBC, 0x5056},
	{0x3FAD, 0x83E8},
	{0x3FBD, 0x506B},
	{0x3FAE, 0x9758},
	{0x3FBE, 0x5074},
	{0x22F8, 0x8005},
	{0x22C8, 0x0026},
	{0x22EE, 0x0000},
	{0x22F9, 0x085F},
	{0x22FA, 0x2481},
	{0x2305, 0x0000},
	{0x2301, 0x0002},
	{0x2307, 0xF8F8},
	{0x2309, 0x0800},
	{0x230D, 0x0400},
	{0x230C, 0x0600},
	{0x22F2, 0x0044},
	{0x22F6, 0x0000},
	{0x22D2, 0x8A94},
	{0x2303, 0x0001},
	{0x2300, 0x0000},
	{0x2304, 0x2310},
	{0x232F, 0x0110},
	{0x2339, 0x0006},
	{0x23D0, 0x0620},
	{0x236E, 0x2000},
	{0x2370, 0x4000},
	{0x3FD2, 0x0032},
	{0x2390, 0x7842},
	{0x2391, 0x4000},
	{0x2392, 0x4000},
	{0x2393, 0x4000},
	{0x2394, 0x4000},
	{0x2395, 0x4000},
	{0x2333, 0x0008},
	{0x23B4, 0x0006},
	{0x23B3, 0x0018},
	{0x23CF, 0x0200},
	{0x23D5, 0x6000},
	{0x22FB, 0x0000}
};

static struct fm34_conf fm34_conf = {
	.pwdn = TEGRA_GPIO_PN1,
	.rst = TEGRA_GPIO_PN0,
	.bp = TEGRA_GPIO_PN3,
	.cprop = sizeof(fm34_property)/sizeof(u16)/2,
	.pprop = (u16 *)fm34_property,
};

static const struct i2c_board_info chagall_fm34_board_info[] = {
	{
		I2C_BOARD_INFO("fm34_i2c", 0x60),
		.platform_data = &fm34_conf,
	},
};

static int __init chagall_fm34_init(void)
{
	tegra_gpio_enable(fm34_conf.pwdn);
	if(fm34_conf.bp != -1)
		tegra_gpio_enable(fm34_conf.bp);
	if(fm34_conf.rst != -1)
		tegra_gpio_enable(fm34_conf.rst);

	i2c_register_board_info(0, chagall_fm34_board_info, 1);

	return 0;
}
#endif

static struct tegra_i2c_platform_data chagall_i2c1_platform_data = {
	.adapter_nr	= 0,
	.bus_count	= 1,
	.bus_clk_rate	= { 400000, 0 },
	.scl_gpio		= {TEGRA_GPIO_PC4, 0},
	.sda_gpio		= {TEGRA_GPIO_PC5, 0},
	.arb_recovery = arb_lost_recovery,
	.i2c_bus_reset_devices = chagall_i2c_bus_reset_devices_gen1,
};

static struct tegra_i2c_platform_data chagall_i2c2_platform_data = {
	.adapter_nr	= 1,
	.bus_count	= 1,
	.bus_clk_rate	= { 400000, 0 },
	.is_clkon_always = true,
	.scl_gpio		= {TEGRA_GPIO_PT5, 0},
	.sda_gpio		= {TEGRA_GPIO_PT6, 0},
	.arb_recovery = arb_lost_recovery,
	.i2c_bus_reset_devices = NULL,
};

static struct tegra_i2c_platform_data chagall_i2c3_platform_data = {
	.adapter_nr	= 2,
	.bus_count	= 1,
	.bus_clk_rate	= { 400000, 0 },
	.scl_gpio		= {TEGRA_GPIO_PBB1, 0},
	.sda_gpio		= {TEGRA_GPIO_PBB2, 0},
	.arb_recovery = arb_lost_recovery,
	.i2c_bus_reset_devices = NULL,
};

static struct tegra_i2c_platform_data chagall_i2c4_platform_data = {
	.adapter_nr	= 3,
	.bus_count	= 1,
	.bus_clk_rate	= { 90000, 0 },
	.scl_gpio		= {TEGRA_GPIO_PV4, 0},
	.sda_gpio		= {TEGRA_GPIO_PV5, 0},
	.arb_recovery = arb_lost_recovery,
	.i2c_bus_reset_devices = NULL,
};

static struct tegra_i2c_platform_data chagall_i2c5_platform_data = {
	.adapter_nr	= 4,
	.bus_count	= 1,
	.bus_clk_rate	= { 400000, 0 },
	.scl_gpio		= {TEGRA_GPIO_PZ6, 0},
	.sda_gpio		= {TEGRA_GPIO_PZ7, 0},
	.arb_recovery = arb_lost_recovery,
	.i2c_bus_reset_devices = NULL,
};

static void chagall_i2c_bus_reset_devices_gen1(struct tegra_i2c_platform_data *pd, u16 client_addr) {

	printk(KERN_ERR "i2c_bus_reset_devices:0x%x!!\n", client_addr);
	/*reset uP*/

	printk(KERN_ERR "Reset mcu by gpio:%d!!\n",  MCU_RESET);
	gpio_direction_output(MCU_RESET, 1); /* high active */
	msleep(50);
	gpio_direction_output(MCU_RESET, 0);
}

static struct wm8903_platform_data chagall_wm8903_pdata = {
	.irq_active_low = 1,
	.micdet_cfg = 0,
	.micdet_delay = 100,
	.gpio_base = CHAGALL_GPIO_WM8903(0),
	.gpio_cfg = {
		(WM8903_GPn_FN_DMIC_LR_CLK_OUTPUT << WM8903_GP1_FN_SHIFT),
		(WM8903_GPn_FN_DMIC_LR_CLK_OUTPUT << WM8903_GP2_FN_SHIFT) |
			WM8903_GP2_DIR,
		0,
		WM8903_GPIO_NO_CONFIG,
		WM8903_GPIO_NO_CONFIG,
	},

	.adc_digital_volume = 0xEF,
	.adc_analogue_volume = 0x1F,
	.dac_digital_volume = 0xBE,
	.dac_headphone_volume = 0x2E,
	.dac_lineout_volume = 0x38,
	.dac_speaker_volume = 0x37,
};

static struct i2c_board_info __initdata chagall_codec_wm8903_info = {
	I2C_BOARD_INFO("wm8903", 0x1a),
	.irq = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_CDC_IRQ),
	.platform_data = &chagall_wm8903_pdata,
};

static void chagall_i2c_init(void)
{
	tegra_i2c_device1.dev.platform_data = &chagall_i2c1_platform_data;
	tegra_i2c_device2.dev.platform_data = &chagall_i2c2_platform_data;
	tegra_i2c_device3.dev.platform_data = &chagall_i2c3_platform_data;
	tegra_i2c_device4.dev.platform_data = &chagall_i2c4_platform_data;
	tegra_i2c_device5.dev.platform_data = &chagall_i2c5_platform_data;

	platform_device_register(&tegra_i2c_device5);
	platform_device_register(&tegra_i2c_device4);
	platform_device_register(&tegra_i2c_device3);
	platform_device_register(&tegra_i2c_device2);
	platform_device_register(&tegra_i2c_device1);

	i2c_register_board_info(0, &chagall_codec_wm8903_info, 1);
}

static struct platform_device *chagall_uart_devices[] __initdata = {
	/* &tegra_uarta_device, */
	&tegra_uartb_device,
	&tegra_uartc_device,
	&tegra_uartd_device,
	/* &tegra_uarte_device, */
};
static struct uart_clk_parent uart_parent_clk[] = {
	[0] = {.name = "clk_m"},
	[1] = {.name = "pll_p"},
#ifndef CONFIG_TEGRA_PLLM_RESTRICTED
	[2] = {.name = "pll_m"},
#endif
};

static struct tegra_uart_platform_data chagall_uart_pdata;
/* static struct tegra_uart_platform_data chagall_loopback_uart_pdata; */

static void __init uart_debug_init(void)
{
	/* UARTD is the debug port. */
	pr_info("Selecting UARTD as the debug console\n");
	chagall_uart_devices[2] = &debug_uartd_device;
	debug_uart_clk =  clk_get_sys("serial8250.0", "uartd");
	debug_uart_port_base = ((struct plat_serial8250_port *)(
		debug_uartd_device.dev.platform_data))->mapbase;
	return;
}

static void __init chagall_uart_init(void)
{
	struct clk *c;
	int i;

	for (i = 0; i < ARRAY_SIZE(uart_parent_clk); ++i) {
		c = tegra_get_clock_by_name(uart_parent_clk[i].name);
		if (IS_ERR_OR_NULL(c)) {
			pr_err("Not able to get the clock for %s\n",
						uart_parent_clk[i].name);
			continue;
		}
		uart_parent_clk[i].parent_clk = c;
		uart_parent_clk[i].fixed_clk_rate = clk_get_rate(c);
	}
	chagall_uart_pdata.parent_clk_list = uart_parent_clk;
	chagall_uart_pdata.parent_clk_count = ARRAY_SIZE(uart_parent_clk);
	tegra_uartb_device.dev.platform_data = &chagall_uart_pdata;
	tegra_uartc_device.dev.platform_data = &chagall_uart_pdata;
	/* tegra_uartd_device.dev.platform_data = &chagall_uart_pdata; */
	if (!is_tegra_debug_uartport_hs()) {
		uart_debug_init();
		/* Clock enable for the debug channel */
		if (!IS_ERR_OR_NULL(debug_uart_clk)) {
			pr_info("The debug console clock name is %s\n",
						debug_uart_clk->name);
			c = tegra_get_clock_by_name("pll_p");
			if (IS_ERR_OR_NULL(c))
				pr_err("Not getting the parent clock pll_p\n");
			else
				clk_set_parent(debug_uart_clk, c);

			clk_enable(debug_uart_clk);
			clk_set_rate(debug_uart_clk, clk_get_rate(c));
		} else {
			pr_err("Not getting the clock %s for debug console\n",
					debug_uart_clk->name);
		}
	}

	platform_add_devices(chagall_uart_devices,
				ARRAY_SIZE(chagall_uart_devices));
}

static struct platform_device tegra_camera = {
	.name = "tegra_camera",
	.id = -1,
};

#ifdef CONFIG_SPI
static struct platform_device *chagall_spi_devices[] __initdata = {
	&tegra_spi_device4,
};

struct spi_clk_parent spi_parent_clk[] = {
	[0] = {.name = "pll_p"},
#ifndef CONFIG_TEGRA_PLLM_RESTRICTED
	[1] = {.name = "pll_m"},
	[2] = {.name = "clk_m"},
#else
	[1] = {.name = "clk_m"},
#endif
};

static struct tegra_spi_platform_data chagall_spi_pdata = {
	.is_dma_based		= true,
	.max_dma_buffer		= (16 * 1024),
	.is_clkon_always	= false,
	.max_rate		= 100000000,
};

static void __init chagall_spi_init(void)
{
	int i;
	struct clk *c;

	for (i = 0; i < ARRAY_SIZE(spi_parent_clk); ++i) {
		c = tegra_get_clock_by_name(spi_parent_clk[i].name);
		if (IS_ERR_OR_NULL(c)) {
			pr_err("Not able to get the clock for %s\n",
						spi_parent_clk[i].name);
			continue;
		}
		spi_parent_clk[i].parent_clk = c;
		spi_parent_clk[i].fixed_clk_rate = clk_get_rate(c);
	}
	chagall_spi_pdata.parent_clk_list = spi_parent_clk;
	chagall_spi_pdata.parent_clk_count = ARRAY_SIZE(spi_parent_clk);
	tegra_spi_device4.dev.platform_data = &chagall_spi_pdata;
	platform_add_devices(chagall_spi_devices,
				ARRAY_SIZE(chagall_spi_devices));

}
#endif

static struct resource tegra_rtc_resources[] = {
	[0] = {
		.start = TEGRA_RTC_BASE,
		.end = TEGRA_RTC_BASE + TEGRA_RTC_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = INT_RTC,
		.end = INT_RTC,
		.flags = IORESOURCE_IRQ,
	},
};

static struct platform_device tegra_rtc_device = {
	.name = "tegra_rtc",
	.id   = -1,
	.resource = tegra_rtc_resources,
	.num_resources = ARRAY_SIZE(tegra_rtc_resources),
};

static struct tegra_asoc_platform_data chagall_audio_wm8903_pdata = {
	.gpio_spkr_en		= TEGRA_GPIO_SPKR_EN,
	.gpio_hp_det		= TEGRA_GPIO_HP_DET,
	.gpio_hp_mute		= -1,
	.gpio_int_mic_en	= -1,
	.gpio_ext_mic_en	= -1,
	.gpio_lineout_det       = TEGRA_GPIO_LINEOUT_DET,
       .i2s_param[HIFI_CODEC]  = {
               .audio_port_id  = 0,
               .is_i2s_master  = 1,
               .i2s_mode       = TEGRA_DAIFMT_I2S,
       },
       .i2s_param[BASEBAND]    = {
               .audio_port_id  = -1,
       },
       .i2s_param[BT_SCO]      = {
               .audio_port_id  = 3,
               .is_i2s_master  = 1,
               .i2s_mode       = TEGRA_DAIFMT_DSP_A,
       },
};

static struct platform_device chagall_audio_wm8903_device = {
	.name	= "tegra-snd-wm8903",
	.id	= 0,
	.dev	= {
		.platform_data = &chagall_audio_wm8903_pdata,
	},
};

static struct timed_gpio timed_gpios[] = {
       {
			.name = "vibrator",
			.gpio = TEGRA_GPIO_PU4,
			.max_timeout = 15000,
       },
};

static struct timed_gpio_platform_data timed_gpio_data = {
       .num_gpios      = ARRAY_SIZE(timed_gpios),
       .gpios          = timed_gpios,
};

static struct platform_device vibrator_timed_gpios = {
       .name = "timed-gpio",
       .id = -1,
       .dev = {
		.platform_data = &timed_gpio_data,
       },
};

static void chagall_vibrator_init(void)
{
       tegra_gpio_enable(TEGRA_GPIO_PU4);
}

static struct platform_device *chagall_devices[] __initdata = {
	&tegra_pmu_device,
	&tegra_rtc_device,
	&tegra_udc_device,
#if defined(CONFIG_TEGRA_IOVMM_SMMU) ||  defined(CONFIG_TEGRA_IOMMU_SMMU)
	&tegra_smmu_device,
#endif
	&tegra_wdt0_device,
	&tegra_wdt1_device,
	&tegra_wdt2_device,
#if defined(CONFIG_TEGRA_AVP)
	&tegra_avp_device,
#endif
	&tegra_camera,
#if defined(CONFIG_CRYPTO_DEV_TEGRA_SE)
	&tegra_se_device,
#endif
	&tegra_ahub_device,
	&tegra_dam_device0,
	&tegra_dam_device1,
	&tegra_dam_device2,
	&tegra_i2s_device1,
	&tegra_i2s_device3,
	&tegra_spdif_device,
	&spdif_dit_device,
	&bluetooth_dit_device,
	&chagall_bt_rfkill_device,
	&tegra_pcm_device,
	&chagall_audio_wm8903_device,
	&tegra_hda_device,
#if defined(CONFIG_CRYPTO_DEV_TEGRA_AES)
	&tegra_aes_device,
#endif
	&vibrator_timed_gpios,
};


#if defined(CONFIG_TOUCHSCREEN_ATMEL_MXT) || defined(CONFIG_TOUCHSCREEN_GOODIX_BIG)
static int chagall_touch_power_enable(void)
{
	struct regulator *chagall_ts_vdd = NULL;

	/* use regulator_enable to power on touch */
	chagall_ts_vdd = regulator_get(NULL, "vdd_5v0_ts");
	if(WARN_ON(IS_ERR(chagall_ts_vdd)))
		pr_err("%s: couldn't get regulator vdd_5v0_ts: %ld\n",
			__func__, PTR_ERR(chagall_ts_vdd));
	else {
		printk("enable regulator touch\n");
		regulator_enable(chagall_ts_vdd);
	}

	return 0;
}
fs_initcall(chagall_touch_power_enable);
#endif

#ifdef CONFIG_TOUCHSCREEN_GOODIX_BIG
static struct goodix_i2c_platform_data goodix_data = {
	.gpio_reset = TEGRA_GPIO_PH6,
};

static struct i2c_board_info __initdata goodix_i2c_info[] = {
	{
		I2C_BOARD_INFO("Goodix-TS", 0x55),
		.irq = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PH4),
		.platform_data = &goodix_data,
	},
};

static int __init chagall_goodix_touch_init(void)
{
       tegra_gpio_enable(TEGRA_GPIO_PH4);
       tegra_gpio_enable(TEGRA_GPIO_PH6);
       i2c_register_board_info(1, goodix_i2c_info, 1);

       return 0;
}
#endif

int enable_wireless_regulator(int enable, int power_type)
{
	struct regulator *chagall_wlan_1v8_vdd = NULL;
	struct regulator *chagall_wlan_3v3_vdd = NULL;

	printk(KERN_INFO "%s(): enable = %d\n", __func__, enable);

	chagall_wlan_1v8_vdd = regulator_get(NULL, "vdd_wlan_1v8");
	chagall_wlan_3v3_vdd = regulator_get(NULL, "vdd_wlan_3v3");

	if (IS_ERR(chagall_wlan_1v8_vdd) || IS_ERR(chagall_wlan_3v3_vdd)) {
		pr_err("fail to get wlan regulator\r\n");
		return -1;
	}

	if (enable) {
		regulator_enable(chagall_wlan_1v8_vdd);
		regulator_enable(chagall_wlan_3v3_vdd);
	} else {
		regulator_disable(chagall_wlan_1v8_vdd);
		regulator_put(chagall_wlan_1v8_vdd);

		regulator_disable(chagall_wlan_3v3_vdd);
		regulator_put(chagall_wlan_3v3_vdd);
	}

	msleep(200);
	return 0;
}

#if defined(CONFIG_USB_SUPPORT)

static struct tegra_usb_platform_data tegra_udc_pdata = {
	.port_otg = true,
	.has_hostpc = true,
	.phy_intf = TEGRA_USB_PHY_INTF_UTMI,
	.op_mode = TEGRA_USB_OPMODE_DEVICE,
	.u_data.dev = {
		.vbus_pmu_irq = 0,
		.vbus_gpio = -1,
		.charging_supported = false,
		.remote_wakeup_supported = false,
	},
	.u_cfg.utmi = {
		.hssync_start_delay = 0,
		.elastic_limit = 16,
		.idle_wait_delay = 17,
		.term_range_adj = 6,
		.xcvr_setup = 8,
		.xcvr_lsfslew = 2,
		.xcvr_lsrslew = 2,
		.xcvr_setup_offset = 0,
		.xcvr_use_fuses = 1,
	},
};

static struct tegra_usb_platform_data tegra_ehci2_utmi_pdata = {
	.port_otg = false,
	.has_hostpc = true,
	.phy_intf = TEGRA_USB_PHY_INTF_UTMI,
	.op_mode        = TEGRA_USB_OPMODE_HOST,
	.u_data.host = {
		.vbus_gpio = -1,
		.hot_plug = false,
		.remote_wakeup_supported = true,
		.power_off_on_suspend = true,
	},
	.u_cfg.utmi = {
		.hssync_start_delay = 0,
		.elastic_limit = 16,
		.idle_wait_delay = 17,
		.term_range_adj = 6,
		.xcvr_setup = 15,
		.xcvr_lsfslew = 2,
		.xcvr_lsrslew = 2,
		.xcvr_setup_offset = 0,
		.xcvr_use_fuses = 1,
	},
};

static struct tegra_usb_platform_data tegra_ehci3_utmi_pdata = {
	.port_otg = false,
	.has_hostpc = true,
	.phy_intf = TEGRA_USB_PHY_INTF_UTMI,
	.op_mode = TEGRA_USB_OPMODE_HOST,
	.u_data.host = {
		.vbus_gpio = -1,
		.vbus_reg = "vdd_vbus_typea_usb",
		.hot_plug = true,
		.remote_wakeup_supported = true,
		.power_off_on_suspend = true,
	},
	.u_cfg.utmi = {
		.hssync_start_delay = 0,
		.elastic_limit = 16,
		.idle_wait_delay = 17,
		.term_range_adj = 6,
		.xcvr_setup = 8,
		.xcvr_lsfslew = 2,
		.xcvr_lsrslew = 2,
		.xcvr_setup_offset = 0,
		.xcvr_use_fuses = 1,
	},
};

static struct tegra_usb_platform_data tegra_ehci1_utmi_pdata = {
	.port_otg = true,
	.has_hostpc = true,
	.phy_intf = TEGRA_USB_PHY_INTF_UTMI,
	.op_mode = TEGRA_USB_OPMODE_HOST,
	.u_data.host = {
		.vbus_gpio = -1,
		.vbus_reg = "vdd_vbus_micro_usb",
		.hot_plug = true,
		.remote_wakeup_supported = true,
		.power_off_on_suspend = true,
	},
	.u_cfg.utmi = {
		.hssync_start_delay = 0,
		.elastic_limit = 16,
		.idle_wait_delay = 17,
		.term_range_adj = 6,
		.xcvr_setup = 15,
		.xcvr_lsfslew = 2,
		.xcvr_lsrslew = 2,
		.xcvr_setup_offset = 0,
		.xcvr_use_fuses = 1,
	},
};

static struct tegra_usb_otg_data tegra_otg_pdata = {
	.ehci_device = &tegra_ehci1_device,
	.ehci_pdata = &tegra_ehci1_utmi_pdata,
};
#endif

#if defined(CONFIG_USB_SUPPORT)
static void chagall_usb_init(void)
{
	/* OTG should be the first to be registered */
	tegra_otg_device.dev.platform_data = &tegra_otg_pdata;
	platform_device_register(&tegra_otg_device);

	/* setup the udc platform data */
	tegra_udc_device.dev.platform_data = &tegra_udc_pdata;

	tegra_ehci2_device.dev.platform_data = &tegra_ehci2_utmi_pdata;
	platform_device_register(&tegra_ehci2_device);

	tegra_ehci3_device.dev.platform_data = &tegra_ehci3_utmi_pdata;
	platform_device_register(&tegra_ehci3_device);

}
#else
static void chagall_usb_init(void) { }
#endif

static int chagall_modem_power_init(void)
{
	if(is_wifi_sku)
		return chagall_mu739_modem_deinit();
	else
		return chagall_mu739_modem_init();
}

fs_initcall(chagall_modem_power_init);


static int __init sku_check(char *options)
{
	if(0 == strcmp(options, "wifi-only"))
		is_wifi_sku = true;
	return 1;
}
__setup("androidboot.carrier=", sku_check);

static int chagall_gps_power_enable(void)
{
	int ret = 0;
	struct regulator *gps_1v8_reg = NULL;
	struct regulator *gps_3v3_reg = NULL;
	gps_3v3_reg = regulator_get(NULL, GPS_3V3_REG);
	if (IS_ERR_OR_NULL(gps_3v3_reg)) {
		printk(KERN_ERR "%s: couldn't get regulator vdd_3v3_gps\n", __func__);
		goto gps_3v3_regulator_put;
	} else {
		regulator_enable(gps_3v3_reg);
		printk(KERN_INFO "%s: regulator vdd_3v3_gps enabled\n", __func__);
	}

	gps_1v8_reg = regulator_get(NULL, GPS_1V8_REG);
	if (IS_ERR_OR_NULL(gps_1v8_reg)) {
		printk(KERN_ERR "%s: couldn't get regulator vdd_1v8_gps\n", __func__);
		goto gps_1v8_regulator_put;
	} else {
		regulator_enable(gps_1v8_reg);
		printk(KERN_INFO "%s: regulator vdd_1v8_gps enabled\n", __func__);
	}
	return ret;

gps_1v8_regulator_put:
	if (gps_1v8_reg) {
		regulator_disable(gps_1v8_reg);
		regulator_put(gps_1v8_reg);
	}
gps_3v3_regulator_put:
	if (gps_3v3_reg) {
		regulator_disable(gps_3v3_reg);
		regulator_put(gps_3v3_reg);
	}
	return ret;
}

module_init(chagall_gps_power_enable);

static int __init rck_check(char *options)
{
	rck = options;
	return 1;
}
__setup("RCK=", rck_check);

static int __init touch_check(char *options)
{
	if (0 == strcmp(options, "goodix"))
		is_goodix = true;
	return 1;
}
__setup("touch=", touch_check);

static void __init tegra_chagall_init(void)
{
	tegra_thermal_init(&thermal_data,
				throttle_list,
				ARRAY_SIZE(throttle_list));
	tegra_clk_init_from_table(chagall_clk_init_table);
	chagall_pinmux_init();
	chagall_i2c_init();
#ifdef CONFIG_SPI
	chagall_spi_init();
#endif
	chagall_usb_init();
#ifdef CONFIG_TEGRA_EDP_LIMITS
	chagall_edp_init();
#endif
	chagall_uart_init();
	platform_add_devices(chagall_devices, ARRAY_SIZE(chagall_devices));
	tegra_ram_console_debug_init();
	tegra_io_dpd_init();
#ifdef CONFIG_SND_SOC_FM34
	chagall_fm34_init();
#endif
	chagall_sdhci_init();
	chagall_regulator_init();
	chagall_suspend_init();
	chagall_goodix_touch_init();
	chagall_keys_init();
	chagall_panel_init();
	chagall_sensors_init(is_wifi_sku);
	chagall_setup_bluesleep();
	chagall_pins_state_init();
	chagall_emc_init();
	tegra_release_bootloader_fb();
	chagall_vibrator_init();
#ifdef CONFIG_TEGRA_WDT_RECOVERY
	tegra_wdt_recovery_init();
#endif
	tegra_serial_debug_init(TEGRA_UARTD_BASE, INT_WDT_CPU, NULL, -1, -1);
	chagall_dock_init();
}

static void __init tegra_chagall_reserve(void)
{
#if defined(CONFIG_NVMAP_CONVERT_CARVEOUT_TO_IOVMM)
	/* support 1920X1200 with 24bpp */
	tegra_reserve(0, SZ_8M + SZ_1M, SZ_8M + SZ_1M);
#else
	tegra_reserve(SZ_128M, SZ_8M, SZ_8M);
#endif
	tegra_ram_console_debug_reserve(SZ_1M);
}

MACHINE_START(CHAGALL, "chagall")
	.boot_params    = 0x80000100,
	.map_io         = tegra_map_common_io,
	.reserve        = tegra_chagall_reserve,
	.init_early	= tegra_init_early,
	.init_irq       = tegra_init_irq,
	.timer          = &tegra_timer,
	.init_machine   = tegra_chagall_init,
MACHINE_END
