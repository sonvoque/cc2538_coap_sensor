/*
 *
 */
#ifndef __NVM_CONFIG_H__
#define __NVM_CONFIG_H__

#include "net/ip/uip.h"

#define DEBUG 1
#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#define PRINT6ADDR(addr) PRINTF("[%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x]", ((uint8_t *)addr)[0], ((uint8_t *)addr)[1], ((uint8_t *)addr)[2], ((uint8_t *)addr)[3], ((uint8_t *)addr)[4], ((uint8_t *)addr)[5], ((uint8_t *)addr)[6], ((uint8_t *)addr)[7], ((uint8_t *)addr)[8], ((uint8_t *)addr)[9], ((uint8_t *)addr)[10], ((uint8_t *)addr)[11], ((uint8_t *)addr)[12], ((uint8_t *)addr)[13], ((uint8_t *)addr)[14], ((uint8_t *)addr)[15])
#define PRINTLLADDR(lladdr) PRINTF("[%02x:%02x:%02x:%02x:%02x:%02x]",(lladdr)->addr[0], (lladdr)->addr[1], (lladdr)->addr[2], (lladdr)->addr[3],(lladdr)->addr[4], (lladdr)->addr[5])
#else
#define PRINTF(...)
#define PRINT6ADDR(addr)
#define PRINTLLADDR(addr)
#endif

#define SENSOR_CONFIG_VERSION VERSION_STRING
#define SENSOR_CONFIG_MAGIC 0x01CE

/* flash config */
/* MAX len for paths and hostnames */
#define SINK_MAXLEN 31
#define VERSION_MAXLEN 31
#define LABEL_MAXLEN 31

#ifndef VERSION_STRING
#define VERSION_STRING ""
#endif

#ifndef LABEL_STRING
#define LABEL_STRING "Default"
#endif

/* default POST location path to post to */
#define DEFAULT_SINK_PATH "/sink"
#define DEFAULT_POST_IP_ADDR(a)		uip_ip6addr(a, 0xbbbb, 0, 0, 0, 0, 0, 0, 1)

/* how long to wait between posts */
#define DEFAULT_POST_INTERVAL 10

#ifdef CC2538_RF_CONF_CHANNEL
#define NVM_DEFAULT_CHANNEL CC2538_RF_CONF_CHANNEL
#else
#define NVM_DEFAULT_CHANNEL 26
#endif

#define NVM_CONFIG_SIZE 2048
#define NVM_CONFIG_ADDRESS (0x00280000 - (2*NVM_CONFIG_SIZE))

#define NVM_NO_RESET	0
#define NVM_RESET	1

/* sensor config */
typedef struct {
  uint16_t magic;			/* sensor magic number 0x5448 */
  char version[VERSION_MAXLEN + 1];	/* sensor config version number */
  char label[LABEL_MAXLEN + 1]; 	/* sensor config label name */
  char sink_path[SINK_MAXLEN + 1];	/* path to post to */
  uint16_t post_interval;		/* how long to wait between posts */
  uint8_t channel;
  uip_ipaddr_t sink_addr;		/* sink's ip address */
} SENSORConfig_t;

extern SENSORConfig_t sensor_cfg;

void load_nvm_config(void);
void sensor_cfg_write(void);
void sensor_config_print(void);

#endif //__NVM_CONFIG_H__
