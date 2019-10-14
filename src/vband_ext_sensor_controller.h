/**
 * Copyright (c) 2014 - 2018, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
#ifndef VBAND_EXT_SENSOR_CONTROLLER_H__
#define VBAND_EXT_SENSOR_CONTROLLER_H__

#include <stdint.h>
#include "nrf.h"

#ifdef __cplusplus
extern "C" {
#endif

/**@brief Sensor Types */
typedef enum
{
    ADXL362  = 1 << 0,
    BME280   = 1 << 1,
    CCS811   = 1 << 2,
    MAX30105 = 1 << 3,
    VSENSOR  = 1 << 4,
    POZYX    = 1 << 5,
    SIMULATE = 1 << 7,
} sensor_type_t;

/**@brief Sensor Types */
typedef enum
{
    PRESENCE_NOT_DETECTED = 0,
    PRESENCE_DETECTED = 1,
} max30105_state_t;

/**@brief return value defines */
typedef enum
{
    WEDGE_SENSOR_SUCCESS = 0,
    WEDGE_SENSOR_NOT_ENABLED = 1
} sensor_return_t;


/**@brief Initialize External Sensor Interfaces
 * returns bitfield indicating which sensors are enabled
 */
uint16_t vband_sensor_init(sensor_type_t use_sensors);

/**@brief Get External Sensor Data
 *
 */
sensor_return_t get_sensor_data(sensor_type_t get_sensor, uint8_t * p_data, uint16_t * p_data_length);

static void twi_init(void);
static void twi_uninit(void);

uint32_t adxl362_configure_wakeup();
uint32_t adxl362_wait_for_sleep();

void vband_sensor_shutdown(sensor_type_t use_sensors);

void vband_sensor_wakeup(sensor_type_t use_sensors);

void vband_sensor_sleep(sensor_type_t use_sensors);

#ifdef __cplusplus
}
#endif

#endif /* VBAND_EXT_SENSOR_CONTROLLER_H__ */
