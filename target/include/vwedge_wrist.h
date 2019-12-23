/**
 * Copyright (c) 2016 - 2018, Nordic Semiconductor ASA
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
#ifndef VWEDGE_WRIST_H
#define VWEDGE_WRIST_H

#ifdef __cplusplus
extern "C" {
#endif

#include "nrf_gpio.h"

// LEDs definitions for VWEDGE_wrist
#define LEDS_NUMBER       1
#define LED_1             NRF_GPIO_PIN_MAP(0,22)
#define LED_START         LED_1
#define LED_STOP          LED_1
#define LEDS_ACTIVE_STATE 1
#define LEDS_LIST         { LED_1 }
#define LEDS_INV_MASK     LEDS_MASK
#define BSP_LED_0         LED_1

// redefine LED intervals for bsp_config.h
#define ADVERTISING_LED_ON_INTERVAL            50
#define ADVERTISING_LED_OFF_INTERVAL           4950

// Buttons definitions; needed for boards.c
#define BUTTONS_NUMBER       1
#define BUTTONS_ACTIVE_STATE 1
#define BUTTON_1             NRF_GPIO_PIN_MAP(1,7)
#define BUTTON_PULL          NRF_GPIO_PIN_PULLDOWN
#define BUTTONS_LIST         { BUTTON_1 }

// Buzzer
#define BUZZER_GPIO           NRF_GPIO_PIN_MAP(0,23)

// vibration motor
#define MOTOR_GPIO            NRF_GPIO_PIN_MAP(0,24)

// Accelerometer ADXL362
#define SPI_INSTANCE_ADXL362  1 /**< SPI instance index. */
#define SPI_SCK_PIN_ADXL362   NRF_GPIO_PIN_MAP(0, 19)
#define SPI_MISO_PIN_ADXL362  NRF_GPIO_PIN_MAP(0, 15)
#define SPI_MOSI_PIN_ADXL362  NRF_GPIO_PIN_MAP(0, 17)
#define SPI_SS_PIN_ADXL362    NRF_GPIO_PIN_MAP(0, 13)
#define ADXL362_POWER_ON      NRF_GPIO_PIN_MAP(0, 26)
#define ADXL362_INT1          NRF_GPIO_PIN_MAP(1, 12)
#define ADXL362_INT2          NRF_GPIO_PIN_MAP(1, 13)
#define ADXL362_LSB_FIRST     0
#define ADXL362_SPI_CLOCK     4000000
#define ADXL362_CLK_POL       0
#define ADXL362_CLK_EDGE      1

// analog power is controlled by GPIO
#define VSENSOR_POWER          NRF_GPIO_PIN_MAP(1, 15) // high is off, low is on

// capacitive touch sensor pins
#define CAPSENSOR_VDD          NRF_GPIO_PIN_MAP(1, 5) // supplies power to cap sensor
#define CAPSENSOR_MTSA         NRF_GPIO_PIN_MAP(1, 8) // uses PWM to set an analog level for sensitivity
#define CAPSENSOR_MODE         NRF_GPIO_PIN_MAP(1, 9) // low is low power, high is high power (fast response)
#define CAPSENSOR_BUTTON0      NRF_GPIO_PIN_MAP(1, 10) // active low
#define CAPSENSOR_BUTTON1      NRF_GPIO_PIN_MAP(1, 11) // active low

// Wristband does not have CCS811 environmental sensor
#define CCS811_RESET_PIN       NRF_GPIO_PIN_MAP(1,3)      // Enable signal pin
#define CCS811_WAKEUP_PIN      NRF_GPIO_PIN_MAP(1,2)      // Wakeup signal pin



#ifdef __cplusplus
}
#endif

#endif // VWEDGE_WRIST_H
