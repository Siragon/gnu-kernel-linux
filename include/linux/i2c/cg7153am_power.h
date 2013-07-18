

#ifndef __CG7153AM_POWER_H__
#define __CG7153AM_POWER_H__

#define PDA_POWER_AC_REMOVE  (0)
#define PDA_POWER_CHARGE_AC  (1 << 0)
#define PDA_POWER_CHARGE_USB (1 << 1)

enum ChargerStatus {
	CHARGING_NOT_PRESENT,
	CHARGING_IN_PROGRESS,
	CHARGING_COMPLETE,
	CHARGING_SUSPEND,
};

struct CG7153AM_power_pdata {
	int (*is_ac_online)(void);
	int ac_present_gpio;
/*	int lowbatt_gpio; */
	unsigned int interval_faster;
	unsigned int interval_slow;
	/*void(*touch_change_config) (void);*/
};

#endif /* __CG7153AM_POWER_H__ */
