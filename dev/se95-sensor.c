/*
 * An interface to the NXP SE95 temperature sensor
 * Ultra high accuracy digital temperature sensor and thermal watchdog
 * -----------------------------------------------------------------
 *
 * Author  : Kiril Petrov (ice@geomi.org)
 */

#include "contiki.h"
#include "lib/sensors.h"
#include "se95-sensor.h"
#include <stdio.h>

#define DEBUG 0
#if DEBUG
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

/*---------------------------------------------------------------------------*/
static void set_configuration() {
	i2c_init(I2C_CONF_SDA_PORT,
		I2C_CONF_SDA_PIN,
		I2C_CONF_SCL_PORT,
		I2C_CONF_SCL_PIN,
		I2C_SCL_NORMAL_BUS_SPEED);
}
/*---------------------------------------------------------------------------*/
static int value(int type) {
	uint8_t temp[2] = { 0, 0 };

	/* set pointer to temperature register */
	i2c_single_send(SE95_ADDR, 0);
	/* receive the data */
	i2c_burst_receive(SE95_ADDR, temp, 2);

	return ((( temp[0] << 8 ) + temp[1]) >> 3);
}
/*---------------------------------------------------------------------------*/
static int status(int type) {
	switch (type) {
		case SENSORS_ACTIVE:
		case SENSORS_READY:
			return 1; // fix?
			break;
	}
	return 0;
}
/*---------------------------------------------------------------------------*/
static int configure(int type, int c) {
	switch (type) {
		case SENSORS_ACTIVE:
			if (c) {
				// set active
				set_configuration();
			} else {
				// set inactive
			}
			return 1;
	}
	return 0;
}


/*---------------------------------------------------------------------------*/
SENSORS_SENSOR(se95_sensor, "Temperature", value, configure, status); // register the functions

