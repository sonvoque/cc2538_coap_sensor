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
 */
/**
 * \addtogroup cc2538dk
 * @{
 *
 * \defgroup cc2538-ext-sensors Sensors
 *  Additional sensors for CC2538 based modules
 *
 * \addtogroup cc2538-ext-sensors
 * @{
 *
 * \defgroup tmp102-sensor TMP102 sensor
 *
 * \addtogroup tmp102-sensor
 * @{
 *
 * \file
 *  Driver for the TMP102 temperature sensor
 */

#ifndef __TMP102_SENSOR_H_
#define __TMP102_SENSOR_H_
#include "i2c.h"
#include "lib/sensors.h"

#define TMP102_ADDR   0x48 /**< TMP102 slave address */
#define TMP102_TEMP_REG   0x00 /**< TMP102 temperature data register */
#define TMP102_CONFIG_REG 0x01 /**< TMP102 configuration register */
#define TMP102_TLOW_REG   0x02 /**< TMP102 Tlow register */
#define TMP102_THIGH_REG  0x03 /**< TMP102 Thigh register */

#define TMP102_RAW_VALUE  0
#define TMP102_VALUE_X100 1
#define TMP102_VALUE_X1000  2

#define TMP102_ACTIVE   SENSORS_ACTIVE
#define TMP102_CONF_MODE  1
#define TMP102_CONF_CONV_RATE 2

#define TMP102_MODE_SHIFT 4
#define TMP102_MODE_MASK  (1 << TMP102_MODE_SHIFT)
#define TMP102_NORMAL_MODE  (0 << TMP102_MODE_SHIFT)  /**< default */
#define TMP102_EXTENDED_MODE  (1 << TMP102_MODE_SHIFT)

#define TMP102_CONV_RATE_SHIFT  6
#define TMP102_CONV_RATE_MASK (3 << TMP102_CONV_RATE_SHIFT)
#define TMP102_CONV_RATE_025HZ  (0 << TMP102_CONV_RATE_SHIFT)
#define TMP102_CONV_RATE_1HZ  (1 << TMP102_CONV_RATE_SHIFT)
#define TMP102_CONV_RATE_4HZ  (2 << TMP102_CONV_RATE_SHIFT) /**< default */
#define TMP102_CONV_RATE_8HZ  (3 << TMP102_CONV_RATE_SHIFT)

#define TMP102_ERROR    -1
#define TMP102_SUCCESS    0

extern const struct sensors_sensor tmp102_sensor;

#endif /* ifndef __TMP102_H_ */
/**
 * @}
 * @}
 * @}
 */
