/*
 * include/linux/i2c/goodix_touch.h
 *
 * Copyright (C) 2010 - 2011 Goodix, Inc.
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
 */

#ifndef _LINUX_GOODIX_TOUCH_H
#define _LINUX_GOODIX_TOUCH_H

/*Touchkey Surpport Part*/
/*#define HAVE_TOUCH_KEY*/
#ifdef HAVE_TOUCH_KEY
       #define READ_COOR_ADDR          0x00
       const uint16_t touch_key_array[] = {
       KEY_MENU,                               /*MENU*/
       KEY_HOME,                               /*HOME*/
       KEY_SEND                                /*CALL*/
};
       #define MAX_KEY_NUM (sizeof(touch_key_array)/sizeof(touch_key_array[0]))
#else
       #define READ_COOR_ADDR                          0x01
#endif

struct goodix_i2c_platform_data {
       uint32_t version;       /* Use this entry for panels with */
       int gpio_reset;
};

/*Test Tool*/
#define _READ_CFG              0x0a
#define _READ_RAWDATA  0x02

/*raw data status*/
#define DATA_READY                     1
#define DATA_NON_ACTIVE        0xffffffff
#define DATA_ACTIVE                    0

#define TIMEOUT (-100)

enum CHIP_TYPE {
       GT800,
       GT801,
       GT800PLUS,
       GT816,
       GT818,
       GT8105,
       GT8110,
       GT819,
};

#endif /* _LINUX_GOODIX_TOUCH_H */

