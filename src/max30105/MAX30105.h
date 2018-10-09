/** \file max30105.h ******************************************************
*
* Project: MAXREFDES117#
* Filename: max30105.h
* Description: This module is an embedded controller driver header file for MAX30105
*
* ------------------------------------------------------------------------- */
/*******************************************************************************
* Copyright (C) 2016 Maxim Integrated Products, Inc., All Rights Reserved.
*
* Permission is hereby granted, free of charge, to any person obtaining a
* copy of this software and associated documentation files (the "Software"),
* to deal in the Software without restriction, including without limitation
* the rights to use, copy, modify, merge, publish, distribute, sublicense,
* and/or sell copies of the Software, and to permit persons to whom the
* Software is furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included
* in all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
* OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
* IN NO EVENT SHALL MAXIM INTEGRATED BE LIABLE FOR ANY CLAIM, DAMAGES
* OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
* ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
* OTHER DEALINGS IN THE SOFTWARE.
*
* Except as contained in this notice, the name of Maxim Integrated
* Products, Inc. shall not be used except as stated in the Maxim Integrated
* Products, Inc. Branding Policy.
*
* The mere transfer of this software does not imply any licenses
* of trade secrets, proprietary technology, copyrights, patents,
* trademarks, maskwork rights, or any other form of intellectual
* property whatsoever. Maxim Integrated Products, Inc. retains all
* ownership rights.
*******************************************************************************
*/
#ifndef _MAX30105_H_
#define _MAX30105_H_

#define MAX30105_NO_ERROR   0
#define MAX30105_ERROR      -1
#define MAX30105_TEMP_ERROR  -999.0

#define MAX30105_I2C_ADDR  0x57 //0xAE
#define MAX30105_PART_ID  0x15

#define MAX30105_DEFAULT_LED_PA 0x20
#define MAX30105_DEFAULT_PILOT_PA 0x40

/**
 * MAX30105 Optical Sensor
 *
 * @code
 * #include "mbed.h"
 * #include "MAX30105.h"
 *
 * I2C i2c(I2C_SDA, I2C_SCL);
 * MAX30105 max30105(&i2c);
 *
 * int main(void) {
 *     int lastPrx = 0;    
 *
 *     max30105.softReset(); // reset the MAX30105
 *     max30105.shutDown();  // shut down while configuring
 *     max30105.enableIntr(MAX30105::INTR_PROX);  // enable proximity interrupt
 *     max30105.setProx(0x40, 0x10);   // set proximity pulse amplitude and threshold
 *     max30105.setSingleLED();  // configure single LED mode to initiate proximity detection
 *     max30105.wakeUp();        // exit shutdown to start sensing
 *
 *     while(1) {
 *         if (max30105.getIntr1() & MAX30105::INTR_PROX) { // if the proximity interrupt occurs
 *             if (!lastPrx) {
 *                 printf("Temperature = %02f\n", max30105.readTemperature());
 *             }
 *             max30105.writeReg(MAX30105::REG_MODE_CONFIG, MAX30105::MODE_1LED); // go back into proximity detection
 *             lastPrx = 1;
 *         } else {
 *             lastPrx = 0;
 *         }
 *         wait_ms(100);
 *     }
 * }
 * @endcode
 */

/*!
 * @brief Type definitions
 */
typedef int8_t (*max30105_com_fptr_t)(uint8_t dev_id, uint8_t reg_addr,
		uint8_t *data, uint16_t len);

/*!
 * @brief max30105 device structure
 */
struct max30105_dev {
	/*! Chip Id */
	uint8_t chip_id;
	/*! Device Id */
	uint8_t dev_id;
	/*! Read function pointer */
	max30105_com_fptr_t read;
	/*! Write function pointer */
	max30105_com_fptr_t write;
};

/**
 * @brief   Register Addresses
 * @details Enumerated MAX30105 register addresses
 */
typedef enum {
  REG_INTR_STATUS_1 = 0x00,    ///< Interrupt Status 1
  REG_INTR_STATUS_2 = 0x01,    ///< Interrupt Status 2
  REG_INTR_ENABLE_1 = 0x02,    ///< Interrupt Enable 1
  REG_INTR_ENABLE_2 = 0x03,    ///< Interrupt Enable 2
  REG_FIFO_WR_PTR = 0x04,      ///< FIFO Write Pointer
  REG_OVF_COUNTER = 0x05,      ///< Overflow Counter
  REG_FIFO_RD_PTR = 0x06,      ///< FIFO Read Pointer
  REG_FIFO_DATA = 0x07,        ///< FIFO Data Register
  REG_FIFO_CONFIG = 0x08,      ///< FIFO Configuration
  REG_MODE_CONFIG = 0x09,      ///< Mode Configuration
  REG_SPO2_CONFIG = 0x0A,      ///< Sensing Configuration
  REG_RSVD_0B = 0x0B,          ///< Reserved 0x0B
  REG_LED1_PA = 0x0C,          ///< LED1 Pulse Amplitude
  REG_LED2_PA = 0x0D,          ///< LED2 Pulse Amplitude
  REG_LED3_PA = 0x0E,          ///< LED3 Pulse Amplitude
  REG_RSVD_0F = 0x0F,          ///< Reserved 0x0F
  REG_PILOT_PA = 0x10,         ///< Proximity Mode LED Pulse Amplitude
  REG_MULTI_LED_CTRL1 = 0x11,  ///< LED Slots 1 and 2
  REG_MULTI_LED_CTRL2 = 0x12,  ///< LED Slots 3 and 4
  REG_TEMP_INT = 0x1F,         ///< Die Temperature Ingteger
  REG_TEMP_FRAC = 0x20,        ///< Die Temperature Fraction
  REG_TEMP_CONFIG = 0x21,      ///< Die Temperature Configuration
  REG_PROX_INTR_THRESH = 0x30, ///< Proximity Interrupt Threshold
  REG_REV_ID = 0xFE,           ///< Part Revision
  REG_PART_ID = 0xFF,          ///< Part ID Code
} registers_t;

/**
 * @brief   Sample Averaging
 * @details Number of samples to be averaged and decimated
 */
typedef enum {
  SMP_AVE_1 = (0 << 5),  ///< 1 (no averaging)
  SMP_AVE_2 = (1 << 5),  ///< 2
  SMP_AVE_4 = (2 << 5),  ///< 4
  SMP_AVE_8 = (3 << 5),  ///< 8
  SMP_AVE_16 = (4 << 5), ///< 16
  SMP_AVE_32 = (5 << 5), ///< 32
} smp_ave_t;

/**
 * @brief   FIFO Almost Full Value
 * @details Number of empty spaces that triggers FIFO_A_FULL interrupt
 */
typedef enum {
  FIFO_A_FULL_0 = 0x0, ///< 0 samples remaining, 32 used
  FIFO_A_FULL_1 = 0x1, ///< 1 samples remaining, 31 used
  FIFO_A_FULL_2 = 0x2, ///< 2 samples remaining, 30 used
  FIFO_A_FULL_3 = 0x3, ///< 3 samples remaining, 29 used
  FIFO_A_FULL_4 = 0x4, ///< 4 samples remaining, 28 used
  FIFO_A_FULL_5 = 0x5, ///< 5 samples remaining, 27 used
  FIFO_A_FULL_6 = 0x6, ///< 6 samples remaining, 26 used
  FIFO_A_FULL_7 = 0x7, ///< 7 samples remaining, 25 used
  FIFO_A_FULL_8 = 0x8, ///< 8 samples remaining, 24 used
  FIFO_A_FULL_9 = 0x9, ///< 9 samples remaining, 23 used
  FIFO_A_FULL_A = 0xA, ///< 10 samples remaining, 22 used
  FIFO_A_FULL_B = 0xB, ///< 11 samples remaining, 21 used
  FIFO_A_FULL_C = 0xC, ///< 12 samples remaining, 20 used
  FIFO_A_FULL_D = 0xD, ///< 13 samples remaining, 19 used
  FIFO_A_FULL_E = 0xE, ///< 14 samples remaining, 18 used
  FIFO_A_FULL_F = 0xF, ///< 15 samples remaining, 17 used
} fifo_a_full_t;

/**
 * @brief   Mode Control
 * @details Operating mode
 */
typedef enum {
  MODE_1LED = 0x2,  ///< Single Red LED mode
  MODE_2LED = 0x3,  ///< Red and IR LED mode
  MODE_MULTI = 0x7, ///< Multi LED mode Red, IR, and/or Green
} mode_t;

/**
 * @brief   ADC Range
 * @details Sets the full scale range of the ADC
 */
typedef enum {
  ADC_RGE_00 = (0 << 5), ///< 2048nA full scale, 7.81pA LSB
  ADC_RGE_01 = (1 << 5), ///< 4096nA full scale, 15.63pA LSB
  ADC_RGE_10 = (2 << 5), ///< 8192nA full scale, 31.25pA LSB
  ADC_RGE_11 = (3 << 5), ///< 16384nA full scale, 62.5pA LSB
} adc_rge_t;

/**
 * @brief   Sample Rate
 * @details Sets the sample rate of the pulses for each enabled LED
 */
typedef enum {
  SMP_RT_50 = (0 << 2),   ///< 50 Samples per second
  SMP_RT_100 = (1 << 2),  ///< 100 Samples per second
  SMP_RT_200 = (2 << 2),  ///< 200 Samples per second
  SMP_RT_400 = (3 << 2),  ///< 400 Samples per second
  SMP_RT_800 = (4 << 2),  ///< 800 Samples per second
  SMP_RT_1000 = (5 << 2), ///< 1000 Samples per second
  SMP_RT_1600 = (6 << 2), ///< 1600 Samples per second
  SMP_RT_3200 = (7 << 2), ///< 3200 Samples per second
}  smp_rt_t;

/**
 * @brief   Pulse Width
 * @details Sets the pulse width for the channel
 */
typedef enum {
  LED_PW_15BIT = 0, ///< 69us, 15 bit resolution
  LED_PW_16BIT = 1, ///< 118us, 16 bit resolution
  LED_PW_17BIT = 2, ///< 215us, 17 bit resolution
  LED_PW_18BIT = 3, ///< 411us, 18 bit resolution
}  led_pw_t;

/**
 * @brief   Slot Control
 * @details Sets the active LED and pulse amplitude for the slot
 */
typedef enum {
  SLOT_DISABLED = 0,    ///< No LED active
  SLOT_RED_LED1 = 1,    ///< Red, LED1_PA
  SLOT_IR_LED2 = 2,     ///< IR, LED3_PA
  SLOT_GREEN_LED3 = 3,  ///< Green, LED3_PA
  SLOT_NONE = 4,        ///< No LED active
  SLOT_RED_PILOT = 5,   ///< Red, PILOT_PA
  SLOT_IR_PILOT = 6,    ///< IR, PILOT_PA
  SLOT_GREEN_PILOT = 7, ///< Green, PILOT_PA
}  slot_t;

static const char INTR_A_FULL = (1 << 7);
static const char INTR_DATA_RDY = (1 << 6);
static const char INTR_ALC_OVF = (1 << 5);
static const char INTR_PROX = (1 << 4);
static const char INTR_PWR_RDY = (1 << 0);
static const char INTR_TEMP_RDY = (1 << 1);

static const char MASK_SMP_AVE = (7 << 5);
static const char MASK_FIFO_ROLLOVER_EN = (1 << 4);
static const char MASK_FIFO_A_FULL = (0xF << 0);

static const char MASK_SHDN = (1 << 7);
static const char MASK_RESET = (1 << 6);
static const char MASK_MODE = (7 << 0);

static const char MASK_ADC_RGE = (3 << 5);
static const char MASK_SMP_RT = (7 << 2);
static const char MASK_LED_PW = (3 << 0);

static const char MASK_SLOT2 = (7 << 4);
static const char MASK_SLOT1 = (7 << 0);
static const char MASK_SLOT4 = (7 << 4);
static const char MASK_SLOT3 = (7 << 0);

/**
     * @brief   Get Interrupt 1
     * @details Reads Interrupt 1
     * @returns contents of REG_INTR_STATUS_1, or -1 if error.
    */
int MAX30105_getIntr1();

/**
     * @brief   Get Interrupt 2
     * @details Reads Temp Interrupt
     * @returns contents of REG_INTR_STATUS_2, or -1 if error.
    */
int MAX30105_getIntr2();

/**
     * @brief   Enable Interrupt
     * @details Enables any interrupt whos bit is set
     * @param   intrBits bit mask of interrupts to enable
     * @returns 0 if no errors, -1 if error.
    */
int MAX30105_enableIntr(char intrBits);

/**
     * @brief   Disable Interrupt
     * @details Disables any interrupt whos bit is set
     * @param   intrBits bit mask of interrupts to disable
     * @returns 0 if no errors, -1 if error.
    */
int MAX30105_disableIntr(char intrBits);

/**
     * @brief   Configure Proximity Function
     * @details Sets amplitude and threshold for proximity detector
     *          Set threshold to 0x00 to disable proximity detecotr
     * @param   proxAmp The amplitude of the proximity pulse 50mA/255 steps
     * @param   proxThresh The detect threshold to start sensing
     * @returns 0 if no errors, -1 if error.
    */
int MAX30105_setProx(char proxAmp, char proxThresh);

/**
     * @brief   Configure Single LED Mode
     * @details Configures part for single LED mode.
     * @param   smpAve number of samples to average
     * @param   fifoRollOver enables FIFO to roll over when full
     * @param   fifoAFull sets FIFO almost full level
     * @param   adcRange sets ADC range
     * @param   smpRate sets sample rate
     * @param   ledPW sets LED pulse width
     * @param   led1PA sets pulse amplitude for LED1
     * @returns 0 if no errors, -1 if error.
    */
int MAX30105_setSingleLED(smp_ave_t smpAve,
    bool fifoRollOver,
    fifo_a_full_t fifoAFull,
    adc_rge_t adcRange,
    smp_rt_t smpRate,
    led_pw_t ledPW,
    char led1PA);

/**
     * @brief   Configure Dual LED Mode
     * @details Configures part for dual LED mode.
     * @param   smpAve number of samples to average
     * @param   fifoRollOver enables FIFO to roll over when full
     * @param   fifoAFull sets FIFO almost full level
     * @param   adcRange sets ADC range
     * @param   smpRate sets sample rate
     * @param   ledPW sets LED pulse width
     * @param   led1PA sets pulse amplitude for LED1
     * @param   led2PA sets pulse amplitude for LED2
     * @returns 0 if no errors, -1 if error.
    */
int MAX30105_setDualLED(smp_ave_t smpAve,
    bool fifoRollOver,
    fifo_a_full_t fifoAFull,
    adc_rge_t adcRange,
    smp_rt_t smpRate,
    led_pw_t ledPW,
    char led1PA,
    char led2PA);

/**
     * @brief   Configure Multi LED Mode
     * @details Configures part for multi LED mode.
     * @param   smpAve number of samples to average
     * @param   fifoRollOver enables FIFO to roll over when full
     * @param   fifoAFull sets FIFO almost full level
     * @param   adcRange sets ADC range
     * @param   smpRate sets sample rate
     * @param   ledPW sets LED pulse width
     * @param   led1PA sets pulse amplitude for LED1
     * @param   led2PA sets pulse amplitude for LED2
     * @param   led3PA sets pulse amplitude for LED3
     * @param   pilotPA sets pulse amplitude for pilot
     * @param   slot1 assigns LED and amplitude to slot 1
     * @param   slot2 assigns LED and amplitude to slot 2
     * @param   slot3 assigns LED and amplitude to slot 3
     * @param   slot4 assigns LED and amplitude to slot 4
     * @returns 0 if no errors, -1 if error.
    */
int MAX30105_setMultiLED(smp_ave_t smpAve,
    bool fifoRollOver,
    fifo_a_full_t fifoAFull,
    adc_rge_t adcRange,
    smp_rt_t smpRate,
    led_pw_t ledPW,
    char led1PA,
    char led2PA,
    char led3PA,
    char pilotPA,
    slot_t slot1,
    slot_t slot2,
    slot_t slot3,
    slot_t slot4);

/**
     * @brief   Initialize MAX30105
     * @details Intializes settings for the MAX30105
     * @returns 0 if no errors, -1 if error.
    */
int MAX30105_init();

/**
     * @brief   Read FIFOs
     * @details Reads from the red and IR FIFOs
     * @param   redLED Pointer for where to store red LED data
     * @param   irLED Pointer for where to store IR LED data
     * @returns 0 if no errors, -1 if error.
    */
int MAX30105_readFIFO(uint32_t *redLED, uint32_t *irLED);

/**
     * @brief   Read Temperature
     * @details Read the die temperature of the MAX30105
     * @returns Temperature in degrees C, or -999.0 if error
    */
float MAX30105_readTemperature();

/**
     * @brief   Reset MAX30105
     * @details Resets regsisters to their power on defaults
     * @returns 0 if no errors, -1 if error.
    */
int MAX30105_softReset();

/**
     * @brief   Shutdown MAX30105
     * @details Puts MAX30105 in power-save mode, retaining regsiter settings
     * @returns 0 if no errors, -1 if error.
    */
int MAX30105_shutDown();

/**
     * @brief   Wake Up MAX30105
     * @details Wakes MAX30105 from power-save mode
     * @returns 0 if no errors, -1 if error.
    */
int MAX30105_wakeUp();

/**
     * @brief   Write Register
     * @details Writes the given value to the specified register.
     * @param   reg The register to be written
     * @param   value The data to be written
     * @returns 0 if no errors, -1 if error.
    */
int MAX30105_writeReg(registers_t reg, char value);

/**
     * @brief   Read Register
     * @details Reads from the specified register
     * @param   reg The register to be read
     * @param   value Pointer for where to store the data
     * @returns 0 if no errors, -1 if error.
    */
int MAX30105_readReg(registers_t reg, char *value);

#endif /* _MAX30105_H_ */