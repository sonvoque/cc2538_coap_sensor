
/*
 *
 */
#ifndef __DS1820_H__
#define __DS1820_H__

#include <stdbool.h>
#include "ds2482.h"

#define DS18S20_FAMMILY_CODE	0x10
#define DS18B20_FAMMILY_CODE	0x28

struct ow_sensor_s {
	struct ow_sensor_s *next;
	int channel;
	int idx;
	int temperature;
	unsigned char rom_addr[ONEWIRE_ROM_LENGTH];
	int available;
};
typedef struct ow_sensor_s ow_sensor_t;

int ds1820_sample_temperature(ow_sensor_t *sensor, bool all);
int ow_read_temperature(ow_sensor_t *sensor);
int ow_convert_temperature_18B20();
int ow_convert_temperature_18S20();

#endif /* __DS18X20_H__ */
