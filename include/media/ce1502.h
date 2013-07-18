#ifndef ___CE1502_SENSOR_H__
#define ___CE1502_SENSOR_H__

/*-------------------------------------------Important--------------------------
 * For changing the SENSOR_NAME, you must need to change the owner of the
 * device. For example, please add /dev/mt9d115 0600 media camera in below file
 * ./device/nvidia/ventana/ueventd.ventana.rc
 * Otherwise, ioctl will get permission deny
 * -------------------------------------------Important-------------------------
 */

#define CE1502_NAME	"ce1502"
#define CE1502_DEV(x)	"/dev/"x
#define CE1502_PATH    CE1502_DEV(CE1502_NAME)

#endif  /* __CE1502_SENSOR_H__ */

