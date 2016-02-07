/*
 * An interface to the NXP SE95 temperature sensor
 * Ultra high accuracy digital temperature sensor and thermal watchdog
 * -----------------------------------------------------------------
 *
 * Author  : Kiril Petrov (ice@geomi.org)
 */

#ifndef __SE95_SENSOR_H__
#define __SE95_SENSOR_H__

#include "i2c.h"
#include "lib/sensors.h"

#define SE95_ADDR	0x48	/* ADDR2:ADDR1:ADDR0 = 000 */

extern const struct sensors_sensor se95_sensor;

#endif

