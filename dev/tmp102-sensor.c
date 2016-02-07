/*
 * Copyright (c) 2016, Kiril Petrov (ice@geomi.org)
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
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
 *
 * This file is part of the Contiki operating system.
 */
/**
 * \addtogroup tmp102-sensor
 * @{
 *
 * \file
 *  Driver for the TMP102 temperature sensor
 * \author
 *  Kiril Petrov (ice@geomi.org)
 */

#include "contiki.h"
#include "lib/sensors.h"
#include "tmp102-sensor.h"

/*---------------------------------------------------------------------------*/
#define DEBUG 1
#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

/*---------------------------------------------------------------------------*/
static uint8_t enabled;
/*---------------------------------------------------------------------------*/
static int
tmp102_write_reg(uint8_t *buf, uint8_t num)
{
  if((buf == NULL) || (num <= 0)) {
    return TMP102_ERROR;
  }

  i2c_master_enable();
  if(i2c_burst_send(TMP102_ADDR, buf, num) == I2C_MASTER_ERR_NONE) {
    return TMP102_SUCCESS;
  }

  return TMP102_ERROR;
}
/*---------------------------------------------------------------------------*/
static int
tmp102_read_reg(uint8_t reg, uint8_t *buf, uint8_t num)
{
  if((buf == NULL) || (num <= 0)) {
    return TMP102_ERROR;
  }

  i2c_master_enable();
  if(i2c_single_send(TMP102_ADDR, reg) == I2C_MASTER_ERR_NONE) {
    if(i2c_burst_receive(TMP102_ADDR, buf, num) == I2C_MASTER_ERR_NONE) {
      return TMP102_SUCCESS;
    }
  }
  return TMP102_ERROR;
}
/*---------------------------------------------------------------------------*/
static int
tmp102_set_configuration(uint8_t *buf, int value)
{
  i2c_init(I2C_CONF_SDA_PORT,
           I2C_CONF_SDA_PIN,
           I2C_CONF_SCL_PORT,
           I2C_CONF_SCL_PIN,
           I2C_SCL_NORMAL_BUS_SPEED);

  PRINTF("TMP102: Set config\n");
  if(tmp102_read_reg(TMP102_CONFIG_REG, buf, 2) != TMP102_SUCCESS) {
    PRINTF("TMP102: Unable to read value of config reg\n");
    return TMP102_ERROR;
  }

  PRINTF("TMP102: conf reg = 0x%02X%02X\n", buf[0], buf[1]);
  enabled = value;
  return TMP102_SUCCESS;
}
/*---------------------------------------------------------------------------*/
static int
tmp102_set_mode(uint8_t *buf, int value)
{
  PRINTF("TMP102: Change mode to %s\n", !!value ? "Extended" : "Normal");
  if(tmp102_read_reg(TMP102_CONFIG_REG, &buf[1], 2) != TMP102_SUCCESS) {
    PRINTF("TMP102: Unable to read value of config reg\n");
    return TMP102_ERROR;
  }

  buf[0] = TMP102_CONFIG_REG;
  buf[2] &= ~TMP102_MODE_MASK;
  buf[2] |= value;

  if(tmp102_write_reg(buf, 3) != TMP102_SUCCESS) {
    PRINTF("TMP102: Unable to change mode\n");
    return TMP102_ERROR;
  }
  return TMP102_SUCCESS;
}
/*---------------------------------------------------------------------------*/
static int
tmp102_set_conv_rate(uint8_t *buf, int value)
{
  PRINTF("TMP102: Change conv rate CR1:CR0 to %d\n",
         (value & TMP102_CONV_RATE_MASK) >> TMP102_CONV_RATE_SHIFT);
  if(tmp102_read_reg(TMP102_CONFIG_REG, &buf[1], 2) != TMP102_SUCCESS) {
    PRINTF("TMP102: Unable to read value of config reg\n");
    return TMP102_ERROR;
  }

  buf[0] = TMP102_CONFIG_REG;
  buf[2] &= ~TMP102_CONV_RATE_MASK;
  buf[2] |= value;

  if(tmp102_write_reg(buf, 3) != TMP102_SUCCESS) {
    PRINTF("TMP102: Unable to change conv rate\n");
    return TMP102_ERROR;
  }
  return TMP102_SUCCESS;
}
/*---------------------------------------------------------------------------*/
static int
tmp102_convert(int type, uint16_t raw_value)
{
  int16_t temp = (int16_t)raw_value;
  int grade;

  switch(type) {
  case TMP102_VALUE_X100:
    grade = 100;
  case TMP102_VALUE_X1000:
    grade = 1000;
    /* 12bit or 13bit format */
    if(temp & 0x01) {
      temp >>= 3;
      if(temp > 0x960) {
        temp -= (1 << 13);
      }
    } else {
      temp >>= 4;
      if(temp > 0x7FF) {
        temp -= (1 << 12);
      }
    }
    temp *= (0.0625 * grade);
    break;
  case TMP102_RAW_VALUE:
    break;
  }
  return temp;
}
/*---------------------------------------------------------------------------*/
static int
value(int type)
{
  uint8_t buf[2];
  uint16_t raw_temp;
  int16_t val;

  if(tmp102_read_reg(TMP102_TEMP_REG, buf, 2) == TMP102_SUCCESS) {
    raw_temp = (buf[0] << 8) + buf[1];
    val = tmp102_convert(type, raw_temp);
    return val;
  }

  PRINTF("TMP102: failed to read sensor\n");
  return TMP102_ERROR;
}
/*---------------------------------------------------------------------------*/
static int
status(int type)
{
  switch(type) {
  case SENSORS_ACTIVE:
  case SENSORS_READY:
    return enabled;
    break;
  }
  return 0;
}
/*---------------------------------------------------------------------------*/
static int
configure(int type, int value)
{
  uint8_t buf[3];

  if((type != TMP102_ACTIVE) && (type != TMP102_CONF_MODE) &&
     (type != TMP102_CONF_CONV_RATE)) {
    PRINTF("TMP102: unsupported configuration option\n");
    return TMP102_ERROR;
  }

  switch(type) {
  case TMP102_ACTIVE:
    if(value) {
      return tmp102_set_configuration(buf, value);
    }
    break;

  case TMP102_CONF_MODE:
    return tmp102_set_mode(buf, value);

  case TMP102_CONF_CONV_RATE:
    return tmp102_set_conv_rate(buf, value);
  }
  return TMP102_ERROR;
}
/*---------------------------------------------------------------------------*/
SENSORS_SENSOR(tmp102_sensor, "Temperature", value, configure, status);
/*---------------------------------------------------------------------------*/
/** @} */

