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
#ifndef VBAND_PWM_CONTROLLER_H__
#define VBAND_PWM_CONTROLLER_H__

#include <stdint.h>
#include "nrf.h"
#include "nrf_drv_pwm.h"

#ifdef __cplusplus
extern "C" {
#endif

/**@brief Buzzer status.
 *
 * @details Different buzzer states depending on alarm algorithm
 */
typedef enum
{
    BUZZER_OFF = 0,
    BUZZER_ON_WARNING,
    BUZZER_ON_ALARM,
    BUZZER_ON_LONG_BEEP
} buzzer_status_t;

/**@brief Buzzer playback repeat enum (for code readability)
*/
typedef enum
{
    BUZZER_LOOP_FOREVER = 0,
    BUZZER_LOOP_1,
    BUZZER_LOOP_2,
    BUZZER_LOOP_3,
} buzzer_loop_count_t;

/**@brief LED status.
 *
 * @details Different led states depending on BLE state
 */
typedef enum
{
    LED_OFF = 0,
    LED_BLE_ADVERTISTING,
    LED_BLE_CONNECTED
} led_status_t;

typedef struct
{
    uint8_t  gpio;
    uint16_t pwm_frequency;      
    uint16_t pwm_duty_cycle;
    uint16_t pwm_period;
    uint16_t pwm_period_on;
    bool     loop_flag;
} pwm_config_vals;

/**@brief Change buzzer state to input status
 *
 */
void set_buzzer_status(buzzer_status_t status, buzzer_loop_count_t playback_count);


/**@brief Change led state to input status
 *
 */
void set_led_status(led_status_t status);

#ifdef __cplusplus
}
#endif

#endif /* VBAND_PWM_CONTROLLER_H__ */
