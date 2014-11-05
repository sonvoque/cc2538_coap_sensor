/*
 * Copyright (c) 2014, Kiril Petrov (ice@geomi.org)
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <string.h>
#include <stdlib.h>

/* contiki */
#include "contiki.h"
#include "contiki-net.h"
#include "net/rpl/rpl.h"
#include "dev/radio.h"
#include "dev/adc.h"
#include "dev/leds.h"
#include "dev/cc2538-sensors.h"
#include "net/rpl/rpl.h"
#include "lpm.h"

#if PLATFORM_HAS_BUTTON
#include "dev/button-sensor.h"
#endif
#if WITH_TMP102_SENSOR
#include "dev/tmp102-sensor.h"
#endif
#if WITH_SE95_SENSOR
#include "dev/se95-sensor.h"
#endif
#include "dev/ds2482.h"

#include "rest-engine.h"
#include "er-coap-engine.h"

#define LEDS_PERIODIC	LEDS_YELLOW

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

/* default POST location path to post to */
#define DEFAULT_SINK_PATH "/sink"

/* how long to wait between posts */
#define DEFAULT_POST_INTERVAL 60

#define REMOTE_PORT     UIP_HTONS(COAP_DEFAULT_PORT)

#define MAX_SENSORS 10
MEMB(sensor_memb, struct ow_sensor_s, MAX_SENSORS);
LIST(sensor_list);
ow_sensor_t *s;
int search_lock;

extern void rplinfo_activate_resources(void);

PROCESS(cc2538_sensor, "CC2538 based sensor");
PROCESS(ow_i2c, "1wire i2c test");
PROCESS(read_temp, "Read temp from DS18x20");
//AUTOSTART_PROCESSES(&cc2538_sensor);
AUTOSTART_PROCESSES(&ow_i2c, &read_temp);

/* flag to test if con has failed or not */
static uint8_t con_ok;
static rpl_dag_t *dag;
static int post_count;
static int uptime_count;

char buf[256];

static struct etimer et_read_sensors;
static struct etimer et_1wire,
		et_read_temp,
		et_sample_temp;
static radio_value_t radio_value;
static process_event_t ev_new_interval;

/* flash config */
/* MAX len for paths and hostnames */
#define SINK_MAXLEN 31

#define SENSOR_CONFIG_PAGE 0x1D000 /* nvm page where conf will be stored */
#define SENSOR_CONFIG_VERSION 1
#define SENSOR_CONFIG_MAGIC 0x5448

/* sensor config */
typedef struct {
  uint16_t magic;			/* sensor magic number 0x5448 */
  uint16_t version;			/* sensor config version number */
  char sink_path[SINK_MAXLEN + 1];	/* path to post to */
  uint16_t post_interval;		/* how long to wait between posts */
  uip_ipaddr_t sink_addr;		/* sink's ip address */
} SENSORConfig;

static SENSORConfig sensor_cfg;

void
sensor_config_set_default(SENSORConfig *c)
{
  int i;
  c->magic = SENSOR_CONFIG_MAGIC;
  c->version = SENSOR_CONFIG_VERSION;
  strncpy(c->sink_path, DEFAULT_SINK_PATH, SINK_MAXLEN);
  c->post_interval = DEFAULT_POST_INTERVAL;
  /* sink's default ip address */
  c->sink_addr.u16[0] = UIP_HTONS(0xbbbb);
  for(i=1;i<7;i++)
	  c->sink_addr.u16[i] = 0;
  c->sink_addr.u16[7] = UIP_HTONS(0x0001);
}

void sensor_config_print(void) {
	PRINTF("sensor config:\n");
	PRINTF("  magic:    %04x\n", sensor_cfg.magic);
	PRINTF("  version:  %d\n",   sensor_cfg.version);
	PRINTF("  sink path: %s\n",  sensor_cfg.sink_path);
	PRINTF("  interval: %d\n",   sensor_cfg.post_interval);
	PRINTF("  ip addr: ");
	PRINT6ADDR(&sensor_cfg.sink_addr);
	PRINTF("\n");
}

int
ipaddr_sprint(char *s, const uip_ipaddr_t *addr)
{
  uint16_t a;
  unsigned int i;
  int f;
  int n;
  n = 0;
  for(i = 0, f = 0; i < sizeof(uip_ipaddr_t); i += 2) {
    a = (addr->u8[i] << 8) + addr->u8[i + 1];
    if(a == 0 && f >= 0) {
      if(f++ == 0) {
	n += sprintf(&s[n], "::");
      }
    } else {
      if(f > 0) {
        f = -1;
      } else if(i > 0) {
	n += sprintf(&s[n], ":");
      }
      n += sprintf(&s[n], "%x", a);
    }
  }
  return n;
}

#if WITH_SE95_SENSOR || WITH_TMP102_SENSOR
static void res_get_handler(void *request, void *response, uint8_t *buffer, uint16_t preferred_size, int32_t *offset);

RESOURCE(res_temp,
         "title=\"Temperature\";rt=\"Temperature\"",
         res_get_handler,
         NULL,
         NULL,
         NULL);

static void
res_get_handler(void *request, void *response, uint8_t *buffer, uint16_t preferred_size, int32_t *offset)
{
  unsigned int accept = -1;
  int temperature;

#if WITH_SE95_SENSOR
  temperature = se95_sensor.value(0);
  if(temperature & (1<<12))
    temperature = (~temperature + 1) * 0.03125 * 1000 * -1;
  else
    temperature *= 0.03125 * 1000;
#elif WITH_TMP102_SENSOR
  temperature = tmp102_sensor.value(TMP102_VALUE_X1000);
#else
  temperature = 0;
#endif

  REST.get_header_accept(request, &accept);

  if(accept == -1 || accept == REST.type.TEXT_PLAIN) {
    REST.set_header_content_type(response, REST.type.TEXT_PLAIN);
    snprintf((char *)buffer, REST_MAX_CHUNK_SIZE, "%d", (uint16_t)temperature);

    REST.set_response_payload(response, (uint8_t *)buffer, strlen((char *)buffer));
  } else if(accept == REST.type.APPLICATION_JSON) {
    REST.set_header_content_type(response, REST.type.APPLICATION_JSON);
    snprintf((char *)buffer, REST_MAX_CHUNK_SIZE, "{'temp':%d}", (uint16_t)temperature);

    REST.set_response_payload(response, buffer, strlen((char *)buffer));
  } else {
    REST.set_response_status(response, REST.status.NOT_ACCEPTABLE);
    const char *msg = "Supporting content-types text/plain and application/json";
    REST.set_response_payload(response, msg, strlen(msg));
  }
}
#endif

static void config_get_handler(void *request, void *response, uint8_t *buffer, uint16_t preferred_size, int32_t *offset);
static void config_post_handler(void *request, void *response, uint8_t *buffer, uint16_t preferred_size, int32_t *offset);

RESOURCE(config,
	"title=\"Config parameters\";rt=\"Data\"",
	config_get_handler,
	config_post_handler,
	NULL,
	NULL);

PROCESS_NAME(read_sensors);

static void
config_get_handler(void *request, void *response, uint8_t *buffer, uint16_t preferred_size, int32_t *offset)
{
  const char *pstr;
  size_t len = 0;
  uint8_t n = 0;

  if ((len = REST.get_query_variable(request, "param", &pstr))) {
    if (strncmp(pstr, "interval", len) == 0) {
      n = sprintf((char *)buffer, "%d", sensor_cfg.post_interval);
    } else if(strncmp(pstr, "path", len) == 0) {
      strncpy((char *)buffer, sensor_cfg.sink_path, SINK_MAXLEN);
      n = strlen(sensor_cfg.sink_path);
    } else if(strncmp(pstr, "ip", len) == 0) {
      n = ipaddr_sprint((char *)buffer, &sensor_cfg.sink_addr);
    } else {
      goto bad;
    }
    REST.set_header_content_type(response, REST.type.TEXT_PLAIN);
    REST.set_response_payload(response, (uint8_t *)buffer, n);
  } else {
    goto bad;
  }
  return;

bad:
  REST.set_response_status(response, REST.status.BAD_REQUEST);
}

static void
config_post_handler(void *request, void *response, uint8_t *buffer, uint16_t preferred_size, int32_t *offset)
{
  void *param = NULL;
  uip_ipaddr_t *new_addr;
  const char *pstr;
  size_t len = 0;
  const uint8_t *new;

  if ((len = REST.get_query_variable(request, "param", &pstr))) {
    if (strncmp(pstr, "interval", len) == 0) {
      param = &sensor_cfg.post_interval;
    } else if(strncmp(pstr, "path", len) == 0) {
      param = sensor_cfg.sink_path;
    } else if(strncmp(pstr, "ip", len) == 0) {
      new_addr = &sensor_cfg.sink_addr;
    } else {
      goto bad_post;
    }
  } else {
    goto bad_post;
  }

    REST.get_request_payload(request, &new);
    if ( (strncmp(pstr, "path", len) == 0) ) {
      strncpy(param, (const char *)new, SINK_MAXLEN);
    } else if(strncmp(pstr, "ip", len) == 0) {
      uiplib_ipaddrconv((const char *)new, new_addr);
      PRINT6ADDR(new_addr);
    }  else {
      *(uint16_t *)param = (uint16_t)atoi((const char *)new);
    }

    /* TODO: */
    /* flash_config_save(&flash_config); */
    sensor_config_print();

    /* do clean-up actions */
    if (strncmp(pstr, "interval", len) == 0) {
      /* send a new_interval event to schedule a post with the new interval */
      process_post(&cc2538_sensor, ev_new_interval, NULL);
    } else if ( (strncmp(pstr, "netloc", len) == 0) ||
		(strncmp(pstr, "path", len) == 0)  ||
		(strncmp(pstr, "ip", len) == 0) ) {
      process_start(&read_sensors, NULL);
    } else if(strncmp(pstr, "channel", len) == 0) {
      /* TODO: save new channel to structure, then make restart of device,
       * and don't forget to use value from this structure to set channel on start
       */
      /*
      set_channel(flash_config.channel);
      flash_config_save(&flash_config);
      */
      /* stuck here and wait for WDT restarts us. Any better options for this ? */
      while (1) { continue; }
      ;
    }

  return;

bad_post:
  REST.set_response_status(response, REST.status.BAD_REQUEST);
}

/* This function is will be passed to COAP_BLOCKING_REQUEST() to handle responses. */
void
client_chunk_handler(void *response)
{
  const uint8_t *chunk;
  coap_packet_t *const coap_pkt = (coap_packet_t *) response;

  int len = coap_get_payload(response, &chunk);
  printf("|%.*s", len, (char *)chunk);

  if (coap_pkt->type == COAP_TYPE_ACK) {
    post_count = 0;
    PRINTF("got ACK\n");
    con_ok = 1;
  } else
    con_ok = 0;
}

PROCESS(do_post, "post results");
PROCESS_THREAD(do_post, ev, data)
{
	PROCESS_BEGIN();
	static coap_packet_t request[1]; /* This way the packet can be treated as pointer as usual. */

	coap_init_message(request, COAP_TYPE_CON, COAP_POST, 0 );
	coap_set_header_uri_path(request, sensor_cfg.sink_path);
	coap_set_payload(request, buf, strlen(buf));

	post_count++;
	COAP_BLOCKING_REQUEST(&sensor_cfg.sink_addr, REMOTE_PORT, request,
				client_chunk_handler);
	if(erbium_status_code)
		PRINTF("status %u: %s\n", erbium_status_code, coap_error_message);
	if (con_ok == 0)
		PRINTF("CON failed\n");
	else
		PRINTF("coap done\n");

	PROCESS_END();
}

#if WITH_BUTTON_SENSOR
PROCESS(button_post, "button post");
PROCESS_THREAD(button_post, ev, data)
{
	uint8_t n = 0;
	linkaddr_t *addr;

	PROCESS_BEGIN();

	addr = &linkaddr_node_addr;
	n += sprintf(&(buf[n]),"{\"eui\":\"%02x%02x%02x%02x%02x%02x%02x%02x\",\"text\":\"%s\"}",
		     addr->u8[0],
		     addr->u8[1],
		     addr->u8[2],
		     addr->u8[3],
		     addr->u8[4],
		     addr->u8[5],
		     addr->u8[6],
		     addr->u8[7],
		     "Button pressed"
		);
	buf[n] = 0;
	PRINTF("buf: %s\n", buf);

	if (dag != NULL) {
		process_start(&do_post, NULL);
	}

	PROCESS_END();
}
#endif

PROCESS(read_sensors, "Read sensors");
PROCESS_THREAD(read_sensors, ev, data)
{
	uint8_t n = 0;
#if WITH_SE95_SENSOR || WITH_TMP102_SENSOR
	uint8_t m = 0;
	char temp_buf[30];
#endif
	int value;
	linkaddr_t *addr;

	PROCESS_BEGIN();

	addr = &linkaddr_node_addr;
	n += sprintf(&(buf[n]),"{\"eui\":\"%02x%02x%02x%02x%02x%02x%02x%02x\"",
		     addr->u8[0],
		     addr->u8[1],
		     addr->u8[2],
		     addr->u8[3],
		     addr->u8[4],
		     addr->u8[5],
		     addr->u8[6],
		     addr->u8[7]);

	value = vdd3_sensor.value(CC2538_SENSORS_VALUE_TYPE_CONVERTED);
	n += sprintf(&(buf[n]),",\"vdd\":%04d", value);
	value = cc2538_temp_sensor.value(CC2538_SENSORS_VALUE_TYPE_CONVERTED);
	n += sprintf(&(buf[n]),",\"temp\":%05d", value);
	n += sprintf(&(buf[n]),",\"c\":\"%d\"", uptime_count);
#if WITH_SE95_SENSOR
	value = se95_sensor.value(0);
	if(value & (1<<12))
		value = (~value + 1) * 0.03125 * 1000 * -1;
	else
		value *= 0.03125 * 1000;
	m = sprintf(temp_buf, ",\"se95\":\"%d mC\"", value);
	strncat(&(buf[n]), temp_buf, m);
	n += m;
#endif
#if WITH_TMP102_SENSOR
	value = tmp102_sensor.value(TMP102_VALUE_X1000);
	m = sprintf(temp_buf, ",\"tmp102\":%05d", value);
	strncat(&(buf[n]), temp_buf, m);
	n += m;
#endif
	if(NETSTACK_RADIO.get_value(RADIO_PARAM_RSSI, &radio_value) == RADIO_RESULT_OK)
		n += sprintf(&(buf[n]),",\"rssi\":\"%d dBm\"", radio_value);

	n += sprintf(&(buf[n]),"}");
	buf[n] = 0;
	PRINTF("buf: %s\n", buf);

	if (dag != NULL) {
		process_start(&do_post, NULL);
	}

	PROCESS_END();
}

PROCESS_THREAD(cc2538_sensor, ev, data)
{
	PROCESS_BEGIN();

	ev_new_interval = process_alloc_event();

	/* Initialize the REST engine. */
	rest_init_engine();
	rest_activate_resource(&config, "config");

#if WITH_SE95_SENSOR || WITH_TMP102_SENSOR
	rest_activate_resource(&res_temp, "temp");
#endif
#if WITH_TMP102_SENSOR
	PRINTF("TMP102 Sensor\n");
	SENSORS_ACTIVATE(tmp102_sensor);
	tmp102_sensor.configure(TMP102_CONF_MODE, TMP102_EXTENDED_MODE);
	tmp102_sensor.configure(TMP102_CONF_CONV_RATE, TMP102_CONV_RATE_4HZ);
#endif
#if WITH_SE95_SENSOR
	PRINTF("SE95 Sensor\n");
	SENSORS_ACTIVATE(se95_sensor);
#endif
#if WITH_BUTTON_SENSOR
	SENSORS_ACTIVATE(button_sensor);
	PRINTF("Button Sensor\n");
#endif

#if WITH_MMS_BOARD
	/* off dual led on PD4 & PD5 */
	GPIO_SET_OUTPUT(GPIO_D_BASE, 0x30);
	GPIO_WRITE_PIN(GPIO_D_BASE, 0x30, 0);
#endif
	sensor_config_set_default(&sensor_cfg);
	sensor_config_print();

	etimer_set(&et_read_sensors, 5 * CLOCK_SECOND);

	dag = NULL;
	post_count = 0;
	uptime_count = 0;

	while(1) {
		PROCESS_WAIT_EVENT();

#if WITH_BUTTON_SENSOR
		if (ev == sensors_event && data == &button_sensor) {
			printf("===Ice: Button pressed\n");
			process_start(&button_post, NULL);
		}
#endif
		if( ev == ev_new_interval ) {
			etimer_set(&et_read_sensors, sensor_cfg.post_interval * CLOCK_SECOND);
			etimer_restart(&et_read_sensors);
			PRINTF("New interval for read_sensor is set\n");
		}

		if(etimer_expired(&et_read_sensors)) {

			PRINTF("---Ice: post_count = %d\n", post_count);

			if(dag == NULL) {
				dag = rpl_get_any_dag();
				if (dag != NULL) {
					PRINTF("joined DAG. Posting to ");
					PRINT6ADDR(&sensor_cfg.sink_addr);
					PRINTF("\n");
				}
				post_count++;
			}

			etimer_set(&et_read_sensors, sensor_cfg.post_interval * CLOCK_SECOND);
			process_start(&read_sensors, NULL);
			uptime_count++;
		}

	}
	PROCESS_END();
}

int scan_channel(int ch) {
	int i, count = 0;
	int status;
	int match;

	DS2482_channel_select(ch);
	status = OWFirst();

	while(status) {
		count++;
		PRINTF("%s: ch[%d:%d] Device found ", __FUNCTION__, ch, count);
		for(i=ONEWIRE_ROM_LENGTH-1;i>=0;i--)
			PRINTF("%02X", ROM_NO[i]);
		//PRINTF("\n");
		PRINTF(" ");

		for(s = list_head(sensor_list); s != NULL; s = list_item_next(s)) {
			// mark this sensor as available
			s->available = 1;
/*
			for(i=ONEWIRE_ROM_LENGTH-1;i>=0;i--)
				PRINTF("%02X", s->rom_addr[i]);
			PRINTF(" ");
*/
			// If already exist in list, break
			match = 1;
			for(i=ONEWIRE_ROM_LENGTH-1;i>=0;i--){
				if(s->rom_addr[i] != ROM_NO[i])
					match = 0;
			}
			if(match)
				break;
		}
//		PRINTF("\n");
		if(s == NULL) {
			// New sensor found, add it to list
			s = memb_alloc(&sensor_memb);
			if(s != NULL) {
				memset(s, 0, sizeof(struct ow_sensor_s));
				for(i=ONEWIRE_ROM_LENGTH-1;i>=0;i--)
					s->rom_addr[i] = ROM_NO[i];
				s->channel = ch;
				s->idx = count;
				s->available = 1;
				PRINTF("Add it to list %d:%d:%d\n", s->channel, s->idx, s->available);
				list_add(sensor_list, s);
			} else
				PRINTF("Can't alloc mem for new sensor\n");
		} else
			PRINTF("\n");

		status = OWNext();
	}

	return count;
}

int find_family(int ch, uint8_t family_code) {
	int i, count = 0;
	int match;

	DS2482_channel_select(ch);
	OWTargetSetup(family_code);

	while (OWNext()) {
		// check for incorrect type
		if (ROM_NO[0] != family_code)
			break;
		count++;
		PRINTF("%s: ch[%d:%d] Device found ", __FUNCTION__, ch, count);
		for (i=ONEWIRE_ROM_LENGTH-1;i>=0;i--)
			PRINTF("%02X", ROM_NO[i]);
		PRINTF(" ");

		for(s = list_head(sensor_list); s != NULL; s = list_item_next(s)) {
			// mark this sensor as available
			s->available = 1;

			// If already exist in list, break
			match = 1;
			for(i=ONEWIRE_ROM_LENGTH-1;i>=0;i--){
				if(s->rom_addr[i] != ROM_NO[i])
					match = 0;
			}
			if(match)
				break;
		}

		if(s == NULL) {
			// New sensor found, add it to list
			s = memb_alloc(&sensor_memb);
			if(s != NULL) {
				memset(s, 0, sizeof(struct ow_sensor_s));
				for(i=ONEWIRE_ROM_LENGTH-1;i>=0;i--)
					s->rom_addr[i] = ROM_NO[i];
				s->channel = ch;
				s->idx = count;
				s->available = 1;
				PRINTF("Add it to list %d:%d:%d\n", s->channel, s->idx, s->available);
				list_add(sensor_list, s);
			} else
				PRINTF("Can't alloc mem for new sensor\n");
		} else
			PRINTF("\n");
	}
	return count;
}

#define FIND_ALL	0

PROCESS_THREAD(ow_i2c, ev, data)
{
	int i;

	PROCESS_BEGIN();
	etimer_set(&et_1wire, 1 * CLOCK_SECOND);

	list_init(sensor_list);
	memb_init(&sensor_memb);

	while(1) {
		PROCESS_WAIT_EVENT();

		if(etimer_expired(&et_1wire)) {
			etimer_set(&et_1wire, 10 * CLOCK_SECOND);
			PRINTF("\nNew interval for DS2482_detect is set\n");
			if(DS2482_detect(DS2482_ADDRESS)) {
				PRINTF("%s: DS2482 detected ----- \n", __FUNCTION__);

				// mark all sensors as unavailable
				for(s = list_head(sensor_list); s != NULL; s = list_item_next(s))
					s->available = 0;

				// prevent other activity to OW
				search_lock = 1;
#if FIND_ALL
				PRINTF("%s: Find all devices\n", __FUNCTION__);
				for(i=0;i<8;i++)
					scan_channel(i);
#else
				PRINTF("%s: Find all devices from 0x10 family\n", __FUNCTION__);
				for(i=0;i<8;i++)
					find_family(i, 0x10);
#endif
				search_lock = 0;

				for(s = list_head(sensor_list); s != NULL; s = list_item_next(s)) {
					if(!s->available) {
						PRINTF("%s: Remove undetected sensor from list ", __FUNCTION__);
						for(i=ONEWIRE_ROM_LENGTH-1;i>=0;i--)
							PRINTF("%02X", s->rom_addr[i]);
						PRINTF("\n");
						list_remove(sensor_list, s);
						memb_free(&sensor_memb, s);
					}
				}

			} else
				PRINTF("%s: No DS2482 detected\n", __FUNCTION__);
		}
	} // while(1)

	// remove all sensors and release memory
	while(list_head(sensor_list)) {
		// remove last element from list
		s = list_chop(sensor_list);
		memb_free(&sensor_memb, s);
	}

	PROCESS_END();
}


PROCESS_THREAD(read_temp, ev, data)
{
	int i, ch, ch_mask;
	int status;

	PROCESS_BEGIN();

	etimer_set(&et_read_temp, 2 * CLOCK_SECOND);

	while(1) {
		PROCESS_WAIT_EVENT();

		if(etimer_expired(&et_read_temp)) {

			// ??? do we realy need this on single CPU ???
			if(search_lock)
				continue;

			etimer_set(&et_read_temp, 2 * CLOCK_SECOND);
			PRINTF("\nNew interval for read temp is set\n");

			PRINTF("----> %s: sensor_list ", __FUNCTION__);
			for(s = list_head(sensor_list); s != NULL; s = list_item_next(s)) {
				for(i=ONEWIRE_ROM_LENGTH-1;i>=0;i--)
					PRINTF("%02X", s->rom_addr[i]);
				PRINTF(" ");
			}
			PRINTF("\n");

			ch_mask = 0;
			for(s = list_head(sensor_list); s != NULL; s = list_item_next(s)) {
				ch = s->channel;
				// if channel not sampled
				if(!(ch_mask & ( 1 << ch ))) {
					PRINTF("----> %s: Start sample temperature on ch = %d\n", __FUNCTION__, ch);
					// Inform all sensors on channel <> to start sample temperature
					DS2482_channel_select(ch);
					status = ds18x20_sample_temperature(NULL, true);
					PRINTF("----> %s: ds18x20_sample_temperature status = %d\n", __FUNCTION__, status);
				}
				// mark channel as already sampled
				ch_mask |= ( 1 << ch );
			}
			// Note: et_read_temp[time] > et_sample_temp[time], i.e. >= 1s
			etimer_set(&et_sample_temp, 1 * CLOCK_SECOND);
		}

		if(etimer_expired(&et_sample_temp)) {
			PRINTF("----> %s: Read temperature\n", __FUNCTION__);
			for(s = list_head(sensor_list); s != NULL; s = list_item_next(s)) {
				for(i=ONEWIRE_ROM_LENGTH-1;i>=0;i--)
					PRINTF("%02X", s->rom_addr[i]);
				PRINTF(" ");
				DS2482_channel_select(s->channel);
				status = ow_read_temperature(s);
				if(status)
					PRINTF("Temperature = %d\n", s->temperature);
				else
					PRINTF("Unable to read temperature, status = %d\n", status);
			}
			//TODO start post task
		}
	}
	PROCESS_END();
}
