/**
 * Copyright (c) 2015 - 2018, Nordic Semiconductor ASA
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
/** @file
 * @defgroup pwm_example_main main.c
 * @{
 * @ingroup pwm_example
 *
 * @brief PWM Example Application main file.
 *
 * This file contains the source code for a sample application using PWM.
 */

#include <stdio.h>
#include <string.h>
#include "nrf_drv_pwm.h"
#include "app_util_platform.h"
#include "app_error.h"
#include "boards.h"
#include "bsp.h"
#include "nrf_drv_clock.h"
#include "vband_pwm_controller.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#define BUZZER_GPIO BSP_LED_1     

#define BUZZER_WARNING_PWM_FREQUENCY          4000  /**< in Hz, the frequency at which the PWM signal changes */
#define BUZZER_WARNING_PWM_DUTY_CYCLE           50  /**< in %, the duty cycle of PWM signal */
#define BUZZER_WARNING_PWM_PERIOD             2000  /**< in msec, how often the PWM signal should be turned on */
#define BUZZER_WARNING_PWM_PERIOD_ON          1000  /**< in msec, how long the PWM signal is activated per period */

#define BUZZER_ALARM_PWM_FREQUENCY            4000  /**< in Hz, the frequency at which the PWM signal changes */
#define BUZZER_ALARM_PWM_DUTY_CYCLE             50  /**< in %, the duty cycle of PWM signal */
#define BUZZER_ALARM_PWM_PERIOD                500  /**< in msec, how often the PWM signal should be turned on */
#define BUZZER_ALARM_PWM_PERIOD_ON             250  /**< in msec, how long the PWM signal is activated per period */

static nrf_drv_pwm_t m_pwm0 = NRF_DRV_PWM_INSTANCE(0); // Assign channel 0 to Buzzer
static nrf_drv_pwm_t m_pwm1 = NRF_DRV_PWM_INSTANCE(1);
static nrf_drv_pwm_t m_pwm2 = NRF_DRV_PWM_INSTANCE(2);
static nrf_drv_pwm_t m_pwm3 = NRF_DRV_PWM_INSTANCE(3);

// This is for tracking PWM instances being used, so we can unintialize only
// the relevant ones when switching from one demo to another.
#define USED_PWM(idx) (1UL << idx)
static uint8_t m_used = 0;
static pwm_config_vals new_pwm0_vals = {BUZZER_GPIO,0,0,0,0};
static pwm_config_vals new_pwm1_vals = {0,0,0,0,0};
static pwm_config_vals new_pwm2_vals = {0,0,0,0,0};
static pwm_config_vals new_pwm3_vals = {0,0,0,0,0};
static nrf_pwm_values_common_t /*const*/ stay_off_values[2] = { 0, 0 };

static void configure_pwm_instance(nrf_drv_pwm_t *m_pwmX, pwm_config_vals *new_pwmX_vals)
{
    uint16_t top_value = (1e6 / new_pwmX_vals->pwm_frequency);
    uint16_t duty_value = (top_value * new_pwmX_vals->pwm_duty_cycle) / 100;    
    uint32_t repeats   = (uint32_t) (new_pwmX_vals->pwm_frequency / 1000) * new_pwmX_vals->pwm_period_on / 2;
    uint32_t end_delay = (uint32_t) (new_pwmX_vals->pwm_frequency / 1000) * (new_pwmX_vals->pwm_period - new_pwmX_vals->pwm_period_on) / 2;
    nrf_drv_pwm_config_t const configX =
    {
        .output_pins =
        {
            new_pwmX_vals->gpio | NRF_DRV_PWM_PIN_INVERTED, // channel 0
            NRF_DRV_PWM_PIN_NOT_USED,             // channel 1
            NRF_DRV_PWM_PIN_NOT_USED,             // channel 2
            NRF_DRV_PWM_PIN_NOT_USED,             // channel 3
        },
        .irq_priority = APP_IRQ_PRIORITY_LOWEST,
        .base_clock   = NRF_PWM_CLK_1MHz,
        .count_mode   = NRF_PWM_MODE_UP,
        .top_value    = top_value,
        .load_mode    = NRF_PWM_LOAD_COMMON,
        .step_mode    = NRF_PWM_STEP_AUTO
    };
    APP_ERROR_CHECK(nrf_drv_pwm_init(m_pwmX, &configX, NULL));;

    // This array cannot be allocated on stack (hence "static") and it must
    // be in RAM (hence no "const", though its content is not changed).
    static nrf_pwm_values_common_t pwm_values[2];
    pwm_values[0] = duty_value;
    pwm_values[1] = duty_value;

    // Sequence 0 - Send Buzzer PWM frequency for new_pwmX_vals->pwm_period_on [ms]
    nrf_pwm_sequence_t const seq0 =
    {
        .values.p_common = pwm_values,
        .length          = NRF_PWM_VALUES_LENGTH(pwm_values),
        .repeats         = repeats,
        .end_delay       = 0
    };

    // Sequence 1 - off, duration: (new_pwmX_vals->pwm_period - new_pwmX_vals->pwm_period_on) ms.
    nrf_pwm_sequence_t const seq1 =
    {
        .values.p_common = stay_off_values,
        .length          = NRF_PWM_VALUES_LENGTH(stay_off_values),
        .repeats         = end_delay,
        .end_delay       = 0
    };

    (void)nrf_drv_pwm_complex_playback(m_pwmX, &seq0, &seq1, 1, NRF_DRV_PWM_FLAG_LOOP);
}

void set_buzzer_status(buzzer_status_t new_status)
{
    static buzzer_status_t current_status = BUZZER_OFF;

    // new status = old, no change
    if (new_status == current_status)
      return;
    
    // need to turn off before any state change
    if (m_used & USED_PWM(0))
    {
        nrf_drv_pwm_uninit(&m_pwm0);
        m_used ^= USED_PWM(0);
    }

    bool config_buzzer_flag = true;
    switch (new_status)
    {
        case BUZZER_OFF:
            NRF_LOG_INFO("Turning buzzer off");
            config_buzzer_flag = false;
            break;
        case BUZZER_ON_WARNING:
            NRF_LOG_INFO("Transition to warning buzzer");
            new_pwm0_vals.pwm_frequency =  BUZZER_WARNING_PWM_FREQUENCY;
            new_pwm0_vals.pwm_duty_cycle = BUZZER_WARNING_PWM_DUTY_CYCLE;
            new_pwm0_vals.pwm_period =     BUZZER_WARNING_PWM_PERIOD;
            new_pwm0_vals.pwm_period_on =  BUZZER_WARNING_PWM_PERIOD_ON;
            break;
        case BUZZER_ON_ALARM:
            NRF_LOG_INFO("Transition to alarm buzzer");
            new_pwm0_vals.pwm_frequency =  BUZZER_ALARM_PWM_FREQUENCY;
            new_pwm0_vals.pwm_duty_cycle = BUZZER_ALARM_PWM_DUTY_CYCLE;
            new_pwm0_vals.pwm_period =     BUZZER_ALARM_PWM_PERIOD;
            new_pwm0_vals.pwm_period_on =  BUZZER_ALARM_PWM_PERIOD_ON;
            break;
        default:
            break;
    }
    
    // configure PWM instance with new vals
    if (config_buzzer_flag)
    {
      m_used |= USED_PWM(0);
      (void)configure_pwm_instance(&m_pwm0, &new_pwm0_vals);
    }
}

/** @} */