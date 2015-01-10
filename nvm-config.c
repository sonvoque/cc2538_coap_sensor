/*
 *
 */

#include "contiki.h"
#include "contiki-lib.h"
#include "rom-util.h"
#include "nvm-config.h"

SENSORConfig_t sensor_cfg;

/*---------------------------------------------------------------------------*/
void sensor_config_print(void)
{
	PRINTF("Sensor config:\n");
	PRINTF("  Magic:     0x%04X\n", sensor_cfg.magic);
	PRINTF("  Version:   %s\n", sensor_cfg.version);
	PRINTF("  Sink path: %s\n", sensor_cfg.sink_path);
	PRINTF("  Interval:  %d\n", sensor_cfg.post_interval);
	PRINTF("  Channel:   %d\n", sensor_cfg.channel);
	PRINTF("  IP to post: ");
	PRINT6ADDR(&sensor_cfg.sink_addr);
	PRINTF("\n");
}

/*---------------------------------------------------------------------------*/
void sensor_cfg_read(void)
{
  PRINTF("Reading 6LBR NVM\n");
  rom_util_memcpy((void *)&sensor_cfg,
   (void *)NVM_CONFIG_ADDRESS, sizeof(SENSORConfig_t));
}

/*---------------------------------------------------------------------------*/
void sensor_cfg_write(void)
{
  long err;
  int retry = 4;
  while (retry > 0 ) {
    PRINTF("Flashing NVM\n");
    err = rom_util_page_erase(NVM_CONFIG_ADDRESS, NVM_CONFIG_SIZE);
    if ( err != 0 ) {
      PRINTF("erase error : %ld\n", err);
    }
    rom_util_program_flash( (uint32_t*)&sensor_cfg,
     NVM_CONFIG_ADDRESS, (sizeof(SENSORConfig_t)/4+1)*4);
    if ( err != 0 ) {
      PRINTF("write error : %ld\n", err);
    }
    if(rom_util_memcmp((void *)&sensor_cfg, (void *)NVM_CONFIG_ADDRESS,
			    sizeof(SENSORConfig_t)) == 0) {
      break;
    }
    PRINTF("verify NVM failed, retry\n");
    retry--;
  }
  if(retry == 0) {
    PRINTF("Could not program 6LBR NVM !\n");
  }
}

/*---------------------------------------------------------------------------*/
void check_nvm(int reset)
{
  uip_ipaddr_t loc_fipaddr;
  int flash = 0;

  if(reset || sensor_cfg.magic != SENSOR_CONFIG_MAGIC) {
    if (reset)
      PRINTF("Reseting NVM...\n");
    else
      PRINTF("Invalid NVM magic number, reseting it...\n");
    sensor_cfg.magic = SENSOR_CONFIG_MAGIC;
    strncpy((char *)&sensor_cfg.version, SENSOR_CONFIG_VERSION, VERSION_MAXLEN);
    strncpy((char *)&sensor_cfg.sink_path, DEFAULT_SINK_PATH, SINK_MAXLEN);
    sensor_cfg.post_interval = DEFAULT_POST_INTERVAL;
    DEFAULT_POST_IP_ADDR(&loc_fipaddr);
    memcpy(&sensor_cfg.sink_addr, &loc_fipaddr.u8, 16);
    sensor_cfg.channel = NVM_DEFAULT_CHANNEL;
    flash = 1;
  }

  if(strncmp(sensor_cfg.version, SENSOR_CONFIG_VERSION, VERSION_MAXLEN)) {
    PRINTF("New version string, write it to the NVM...\n");
    flash = 1;
  }

  if(flash)
    sensor_cfg_write();
}

/*---------------------------------------------------------------------------*/
void load_nvm_config(void)
{
  sensor_cfg_read();
  PRINTF("NVM Magic:    0x%04X\n", sensor_cfg.magic);
  PRINTF("NVM Version:  %s\n",   sensor_cfg.version);

  check_nvm(NVM_NO_RESET);
  sensor_config_print();
}

