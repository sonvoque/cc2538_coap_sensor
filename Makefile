#
all: coap-post

WITH_SE95_SENSOR=0
WITH_TMP102_SENSOR=1
WITH_BUTTON_SENSOR=1
WITH_MMS_BOARD=1

CONTIKI= contiki
CFLAGS += -DPROJECT_CONF_H=\"project-conf.h\"

SMALL=1

# REST Engine shall use Erbium CoAP implementation
APPS += er-coap
APPS += rest-engine

PROJECTDIRS += rplinfo
PROJECT_SOURCEFILES += rplinfo.c

PROJECTDIRS += dev

ifeq ($(WITH_MMS_BOARD),1)
CFLAGS += -DWITH_MMS_BOARD=1
endif
ifeq ($(WITH_SE95_SENSOR),1)
CFLAGS += -DWITH_SE95_SENSOR=1
PROJECT_SOURCEFILES += se95-sensor.c
endif
ifeq ($(WITH_TMP102_SENSOR),1)
CFLAGS += -DWITH_TMP102_SENSOR=1
PROJECT_SOURCEFILES += tmp102-sensor.c
endif
ifeq ($(WITH_BUTTON_SENSOR),1)
CFLAGS += -DWITH_BUTTON_SENSOR=1
endif

VERSION_STRING := $(shell git rev-parse --verify --short HEAD)
CFLAGS += -DVERSION_STRING=\"cc2538_coap_sensor-${VERSION_STRING}\"

include $(CONTIKI)/Makefile.include
