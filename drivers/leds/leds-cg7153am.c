/*
 * leds-mp.c - MCU Led Driver
 *
 * Copyright (c) 2011, PEGATRON Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 *
 */

#include <linux/module.h>
#include <linux/i2c/cg7153am.h>
#include <linux/leds-cg7153am.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#if 0
#define DEBUG
#endif
#ifdef DEBUG
#define LOG_FUNC() printk(KERN_INFO "<%s>\n", __func__)
#define LOG_MSG(x...) pr_err(x)
#else
#define LOG_FUNC() do {} while (0)
#define LOG_MSG(x...) do {} while (0);
#endif

static inline struct device *to_cg7153am_dev(struct device *dev)
{
	return dev->parent;
}

static void set_brightness(struct led_classdev *led_cdev,
				  enum led_brightness brightness)
{
	struct cg7153am_led_device *led =
		container_of(led_cdev, struct cg7153am_led_device, led_dev);

	LOG_MSG("%s-id:%d, brightness:%d\n", __func__, led->led_id, brightness);

	cg7153am_led_set(to_cg7153am_dev(led_cdev->dev),
		led->led_id,
		(brightness == LED_OFF) ? MCU_LED_OFF : MCU_LED_ON);
}

static int set_blink(struct led_classdev *led_cdev,
				     unsigned long *delay_on,
				     unsigned long *delay_off)
{
	struct cg7153am_led_device *led =
		container_of(led_cdev, struct cg7153am_led_device, led_dev);

	LOG_MSG("%s-id:%d, delay_on:%ld, delay_off:%ld\n", __func__,
		led->led_id, *delay_on, *delay_off);

	/*note: we don't care the duration of blinking now*/
	if (*delay_on > 0 && *delay_off > 0)
		return cg7153am_led_set(to_cg7153am_dev(led_cdev->dev),
			led->led_id, MCU_LED_BLINKING);
	else
		return cg7153am_led_set(to_cg7153am_dev(led_cdev->dev),
			led->led_id, MCU_LED_OFF);
}
static int __devinit cg7153am_led_probe(struct platform_device *pdev)
{
	struct cg7153am_led_platform_data *pd = pdev->dev.platform_data;
	struct cg7153am_led_device *each_led;

	int ret = -ENODEV;
	int i;

	LOG_FUNC();

	for (i = 0; i < pd->num_led; i++) {
		each_led = &pd->led_devs[i];

		LOG_MSG("%s:name:%s, id:%d, num_led:%d\n", __func__,
			each_led->led_dev.name, each_led->led_id, pd->num_led);

		each_led->led_dev.brightness_set = set_brightness;
		each_led->led_dev.blink_set = set_blink;
		ret = led_classdev_register(to_cg7153am_dev(&pdev->dev),
			&each_led->led_dev);
		if (ret < 0) {
			WARN_ON(1);
			break;
		}
	}

	return ret;
}

static int __devexit cg7153am_led_remove(struct platform_device *pdev)
{
	int i;
	struct cg7153am_led_platform_data *pd = pdev->dev.platform_data;
	struct cg7153am_led_device *each_led;

	LOG_FUNC();

	for (i = 0; i < pd->num_led; i++) {
		each_led = &pd->led_devs[i];
		led_classdev_unregister(&each_led->led_dev);
	}

	return 0;
}

static struct platform_driver led_platform_driver = {
	.driver	= {
		.name	= "cg7153am-leds",
		.owner	= THIS_MODULE,
	},
	.probe	= cg7153am_led_probe,
	.remove	= __devexit_p(cg7153am_led_remove),
};

static int __init cg7153am_led_init(void)
{
	LOG_FUNC();

	return platform_driver_register(&led_platform_driver);

}
device_initcall_sync(cg7153am_led_init);

static void __exit cg7153am_led_exit(void)
{
	LOG_FUNC();
	platform_driver_unregister(&led_platform_driver);
}
module_exit(cg7153am_led_exit);

MODULE_AUTHOR("Andrew Hsiao <Andrew_Hsiao@pegatroncorp.com>");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("MicroP LEDS");
