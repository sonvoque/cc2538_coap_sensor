
#include "contiki.h"
#include "lib/sensors.h"
#include "ds1820.h"

#define DEBUG 1
#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif


//--------------------------------------------------------------------------
// Tell sensor to sample temperature.
// all = 1 tell all sensors to sample.
//
int ds1820_sample_temperature(ow_sensor_t *sensor, bool all){

        if(!OWReset())
                return 0;

        if(all) {
                OWWriteByte(ONEWIRE_CMD_SKIPROM);
        } else {
                OWWriteByte(ONEWIRE_CMD_MATCHROM);
		OWBlock(sensor->rom_addr, ONEWIRE_ROM_LENGTH);
        }

	// Provide power for temperature conversion
	DS2482_set_pullup();
	// Start temp conv.
	OWWriteByte(0x44);

	return 1;
}

//--------------------------------------------------------------------------
// Read temperature from sensor.
//
int ow_read_temperature(ow_sensor_t *s){
	int i, ret = 0;
	uint8_t family;

        if(!OWReset())
                return ret;

        OWWriteByte(ONEWIRE_CMD_MATCHROM);
	OWBlock(s->rom_addr, ONEWIRE_ROM_LENGTH);

        // Send Read Scratchpad command
        OWWriteByte(0xBE);

        for(i=0;i<ONEWIRE_SCRATCHPAD_LENGTH;i++)
		read_result[i] = 0xFF;
        // Read 9 bytes of scratchpad
	OWBlock(read_result, ONEWIRE_SCRATCHPAD_LENGTH);

	// handle different sensor variants (18S20, 18B20)
	family = s->rom_addr[0];
	ret = 1;
	switch(family) {
		case DS18S20_FAMMILY_CODE:
			s->temperature = ow_convert_temperature_18S20();
			break;
		case DS18B20_FAMMILY_CODE:
			s->temperature = ow_convert_temperature_18B20();
			break;
		default:
			ret = 0;
	}

        return ret;
}

//--------------------------------------------------------------------------
// DS18B20 - Family code 0x28
// Time to sample 750ms
//
int ow_convert_temperature_18B20() {
        int temperature;

        temperature = ( read_result[1] << 8 ) + read_result[0];
        //temperature *= 0.0625; //*10 to increase precision of int
        //temperature *= 0.625;
        temperature *= 6.25;
        return temperature;
}

//--------------------------------------------------------------------------
// DS18S20 - Family code 0x10
// Time to sample 750ms
//
int ow_convert_temperature_18S20() {
        int temp;
        int temp_full;
        int count_per_c,count_remain;

        temp=read_result[1];
        temp<<=8;
        temp|=read_result[0];

        temp>>=1;
        temp*=10;

        if((read_result[1]>>7)==1){
        //      temp|=0x8000;
        }
        if(read_result[0]&0x01){
                temp+=5;
        }

        count_per_c=read_result[7];
        count_remain=read_result[6];

        count_per_c*=100;
        count_remain*=100;

        temp_full=temp-25+(count_per_c-count_remain)/count_per_c;

        return temp_full;
}

