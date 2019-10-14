/** \file max30105.cpp ******************************************************
*
* Project: MAXREFDES117#
* Filename: max30105.cpp
* Description: This module is an embedded controller driver for the MAX30105
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
#include <stdbool.h>
#include "boards.h"
#include "nrf_drv_gpiote.h"
#include "app_util_platform.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "app_error.h"
#include <string.h>
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "MAX30105.h"

// Configuration Register Cache
static char _interruptEnable1;
static char _interruptEnable2;
static char _fifoConfiguration;
static char _modeConfiguration;
static char _spo2Configuration;
static char _led1PulseAmplitude;
static char _led2PulseAmplitude;
static char _led3PulseAmplitude;
static char _pilotPulseAmplitude;
static char _multiLedControl1;
static char _multiLedControl2;
static char _proxIntThreshold;

static struct max30105_dev *i2c_dev;

//******************************************************************************
int MAX30105_writeReg(registers_t reg,  char value)
{
    char cmdData[2] = { (char)reg, value };

    if (i2c_dev->write(MAX30105_I2C_ADDR, cmdData[0], &cmdData[1], sizeof(cmdData)-1) != 0) {
        return MAX30105_ERROR;
    }

    return MAX30105_NO_ERROR;
}

//******************************************************************************
int MAX30105_readReg(registers_t reg, char *value)
{
    if (i2c_dev->read(MAX30105_I2C_ADDR, (char)reg, value, 1) != 0) {
        return MAX30105_ERROR;
    }

    return MAX30105_NO_ERROR;
}

//******************************************************************************
int MAX30105_setSingleLED(smp_ave_t smpAve,
                    bool fifoRollOver,
                    fifo_a_full_t fifoAFull,
                    adc_rge_t adcRange,
                    smp_rt_t smpRate,
                    led_pw_t ledPW,
                    char led1PA) 
{
    char dataBuf[4];                    
    _fifoConfiguration = (smpAve & MASK_SMP_AVE) | (fifoAFull & MASK_FIFO_A_FULL);
    if(fifoRollOver) _fifoConfiguration |= MASK_FIFO_ROLLOVER_EN;
    _modeConfiguration = (_modeConfiguration & MASK_SHDN) | MODE_1LED;
    _spo2Configuration = (adcRange & MASK_ADC_RGE) | (smpRate & MASK_SMP_RT) | (ledPW & MASK_LED_PW);
    _led1PulseAmplitude = led1PA;
    dataBuf[0] = REG_LED1_PA;
    dataBuf[1] = _led1PulseAmplitude;
    if (i2c_dev->write(MAX30105_I2C_ADDR, dataBuf[0], &dataBuf[1], 1) != 0) return MAX30105_ERROR;
    dataBuf[0] = REG_FIFO_CONFIG;
    dataBuf[1] = _fifoConfiguration;
    dataBuf[2] = _modeConfiguration;
    dataBuf[3] = _spo2Configuration;
    if (i2c_dev->write(MAX30105_I2C_ADDR, dataBuf[0], &dataBuf[1], 3) != 0) return MAX30105_ERROR;
    
    return MAX30105_NO_ERROR;
}

//******************************************************************************
int MAX30105_setDualLED(smp_ave_t smpAve,
                    bool fifoRollOver,
                    fifo_a_full_t fifoAFull,
                    adc_rge_t adcRange,
                    smp_rt_t smpRate,
                    led_pw_t ledPW,
                    char led1PA,
                    char led2PA) 
{
    char dataBuf[4];                    
    _fifoConfiguration = (smpAve & MASK_SMP_AVE) | (fifoAFull & MASK_FIFO_A_FULL);
    if(fifoRollOver) _fifoConfiguration |= MASK_FIFO_ROLLOVER_EN;
    _modeConfiguration = (_modeConfiguration & MASK_SHDN) | MODE_2LED;
    _spo2Configuration = (adcRange & MASK_ADC_RGE) | (smpRate & MASK_SMP_RT) | (ledPW & MASK_LED_PW);
    _led1PulseAmplitude = led1PA;
    _led2PulseAmplitude = led2PA;
    dataBuf[0] = REG_LED1_PA;
    dataBuf[1] = _led1PulseAmplitude;
    dataBuf[2] = _led2PulseAmplitude;
    if (i2c_dev->write(MAX30105_I2C_ADDR, dataBuf[0], &dataBuf[1], 2) != 0) return MAX30105_ERROR;
    dataBuf[0] = REG_FIFO_CONFIG;
    dataBuf[1] = _fifoConfiguration;
    dataBuf[2] = _modeConfiguration;
    dataBuf[3] = _spo2Configuration;
    if (i2c_dev->write(MAX30105_I2C_ADDR, dataBuf[0], &dataBuf[1], 3) != 0) return MAX30105_ERROR;
    
    return MAX30105_NO_ERROR;
}

//******************************************************************************
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
                    slot_t slot4) 
{
    char dataBuf[4];                    
    _fifoConfiguration = (smpAve & MASK_SMP_AVE) | (fifoAFull & MASK_FIFO_A_FULL);
    if(fifoRollOver) _fifoConfiguration |= MASK_FIFO_ROLLOVER_EN;
    _modeConfiguration = (_modeConfiguration & MASK_SHDN) | MODE_MULTI;
    _spo2Configuration = (adcRange & MASK_ADC_RGE) | (smpRate & MASK_SMP_RT) | (ledPW & MASK_LED_PW);
    _led1PulseAmplitude = led1PA;
    _led2PulseAmplitude = led2PA;
    _led3PulseAmplitude = led3PA;
    _pilotPulseAmplitude = pilotPA;
    _multiLedControl1 = (slot2 << 4) | slot1;
    _multiLedControl2 = (slot4 << 4) | slot3;
    dataBuf[0] = REG_PILOT_PA;
    dataBuf[1] = _pilotPulseAmplitude;
    dataBuf[2] = _multiLedControl1;
    dataBuf[3] = _multiLedControl2;
    if (i2c_dev->write(MAX30105_I2C_ADDR, dataBuf[0], &dataBuf[1], 3) != 0) return MAX30105_ERROR;
    dataBuf[0] = REG_LED1_PA;
    dataBuf[1] = _led1PulseAmplitude;
    dataBuf[2] = _led2PulseAmplitude;
    dataBuf[3] = _led3PulseAmplitude;
    if (i2c_dev->write(MAX30105_I2C_ADDR, dataBuf[0], &dataBuf[1], 3) != 0) return MAX30105_ERROR;
    dataBuf[0] = REG_FIFO_CONFIG;
    dataBuf[1] = _fifoConfiguration;
    dataBuf[2] = _modeConfiguration;
    dataBuf[3] = _spo2Configuration;
    if (i2c_dev->write(MAX30105_I2C_ADDR, dataBuf[0], &dataBuf[1], 3) != 0) return MAX30105_ERROR;
    
    return MAX30105_NO_ERROR;
}

//******************************************************************************
int MAX30105_init(struct max30105_dev *devPtr)
{
    // assign ptr address to static global var
    i2c_dev = devPtr;

    if(MAX30105_writeReg(REG_INTR_ENABLE_1,0xc0) != MAX30105_NO_ERROR) // INTR setting
        return MAX30105_ERROR;
    if(MAX30105_writeReg(REG_INTR_ENABLE_2,0x00) != MAX30105_NO_ERROR)
        return MAX30105_ERROR;
    if(MAX30105_writeReg(REG_FIFO_WR_PTR,0x00) != MAX30105_NO_ERROR)  //FIFO_WR_PTR[4:0]
        return MAX30105_ERROR;
    if(MAX30105_writeReg(REG_OVF_COUNTER,0x00) != MAX30105_NO_ERROR)  //OVF_COUNTER[4:0]
        return MAX30105_ERROR;
    if(MAX30105_writeReg(REG_FIFO_RD_PTR,0x00) != MAX30105_NO_ERROR)  //FIFO_RD_PTR[4:0]
        return MAX30105_ERROR;
    if(MAX30105_writeReg(REG_FIFO_CONFIG,0x0f) != MAX30105_NO_ERROR)  //sample avg = 1, fifo rollover=false, fifo almost full = 17
        return MAX30105_ERROR;
    if(MAX30105_writeReg(REG_MODE_CONFIG,0x03) != MAX30105_NO_ERROR)   //0x02 for Red only, 0x03 for SpO2 mode 0x07 multimode LED
        return MAX30105_ERROR;
    if(MAX30105_writeReg(REG_SPO2_CONFIG,0x20) != MAX30105_NO_ERROR)  // SPO2_ADC range = 4096nA, SPO2 sample rate (50 Hz), LED pulseWidth (50uS)
        return MAX30105_ERROR;
//    if(MAX30105_writeReg(REG_LED1_PA,0x24) != MAX30105_NO_ERROR)   //Choose value for ~ 7mA for LED1
//        return MAX30105_ERROR;
//    if(MAX30105_writeReg(REG_LED2_PA,0x24) != MAX30105_NO_ERROR)   // Choose value for ~ 7mA for LED2
//        return MAX30105_ERROR;
    //if(MAX30105_writeReg(REG_LED3_PA,0x24) != MAX30105_NO_ERROR)   // Choose value for ~ 7mA for LED3 not using Green LED
        //return MAX30105_ERROR;
//    if(MAX30105_writeReg(REG_PILOT_PA,0x7f) != MAX30105_NO_ERROR)   // Choose value for ~ 25mA for Pilot LED proximity LED current set later
        //return MAX30105_ERROR;
    return MAX30105_NO_ERROR;
}

int MAX30105_readFIFO(uint32_t *redLED, uint32_t *irLED)
{
    uint32_t un_temp;
    //char uch_temp;
    *redLED=0;
    *irLED=0;
    char ach_i2c_data[6];

    //read and clear status register
    //readReg(REG_INTR_STATUS_1, &uch_temp);
    //readReg(REG_INTR_STATUS_2, &uch_temp);

    if(i2c_dev->read(MAX30105_I2C_ADDR, (char)REG_FIFO_DATA, ach_i2c_data, 6) !=0) {
        return MAX30105_ERROR;
    }

    un_temp=(unsigned char) ach_i2c_data[0];
    un_temp<<=16;
    *redLED+=un_temp;
    un_temp=(unsigned char) ach_i2c_data[1];
    un_temp<<=8;
    *redLED+=un_temp;
    un_temp=(unsigned char) ach_i2c_data[2];
    *redLED+=un_temp;

    un_temp=(unsigned char) ach_i2c_data[3];
    un_temp<<=16;
    *irLED+=un_temp;
    un_temp=(unsigned char) ach_i2c_data[4];
    un_temp<<=8;
    *irLED+=un_temp;
    un_temp=(unsigned char) ach_i2c_data[5];
    *irLED+=un_temp;
    *redLED&=0x03FFFF;  //Mask MSB [23:18]
    *irLED&=0x03FFFF;  //Mask MSB [23:18]


    return MAX30105_NO_ERROR;
}


//******************************************************************************
int MAX30105_getIntr1()
{
    char intStatus[2] = {0};
    if(MAX30105_readReg(REG_INTR_STATUS_1, &intStatus[0]) != MAX30105_NO_ERROR) return MAX30105_TEMP_ERROR;
    return (int)intStatus[0];
}

//******************************************************************************
int MAX30105_getIntr2()
{
    char intStatus[2] = {0};
    if(MAX30105_readReg(REG_INTR_STATUS_2, &intStatus[0]) != MAX30105_NO_ERROR) return MAX30105_TEMP_ERROR;
    return (int)intStatus[0];
}

//******************************************************************************
int MAX30105_enableIntr(char intrBits)
{
    char intr1 = intrBits & (INTR_A_FULL|INTR_DATA_RDY|INTR_ALC_OVF|INTR_PROX);
    char intr2 = intrBits & INTR_TEMP_RDY;
    _interruptEnable1 |= intr1;
    if(MAX30105_writeReg(REG_INTR_ENABLE_1, _interruptEnable1) != MAX30105_NO_ERROR) return MAX30105_TEMP_ERROR;    
    _interruptEnable2 |= intr2;
    if(MAX30105_writeReg(REG_INTR_ENABLE_2, _interruptEnable2) != MAX30105_NO_ERROR) return MAX30105_TEMP_ERROR;    
    return MAX30105_NO_ERROR;
}

//******************************************************************************
int MAX30105_disableIntr(char intrBits)
{
    char intr1 = intrBits & (INTR_A_FULL|INTR_DATA_RDY|INTR_ALC_OVF|INTR_PROX);
    char intr2 = intrBits & INTR_TEMP_RDY;
    _interruptEnable1 &= ~intr1;
    if(MAX30105_writeReg(REG_INTR_ENABLE_1, _interruptEnable1) != MAX30105_NO_ERROR) return MAX30105_TEMP_ERROR;    
    _interruptEnable2 &= ~intr2;
    if(MAX30105_writeReg(REG_INTR_ENABLE_2, _interruptEnable2) != MAX30105_NO_ERROR) return MAX30105_TEMP_ERROR;    
    return MAX30105_NO_ERROR;
}

//******************************************************************************
int MAX30105_setProx(char proxAmp, char proxThresh)
{
    if(MAX30105_writeReg(REG_PILOT_PA, proxAmp) != MAX30105_NO_ERROR) return MAX30105_ERROR;
    if(MAX30105_writeReg(REG_PROX_INTR_THRESH, proxThresh) != MAX30105_NO_ERROR) return MAX30105_ERROR;
    return MAX30105_NO_ERROR;        
}


//******************************************************************************
float MAX30105_readTemperature()
{
    char dataBuf[2] = {0};
    int8_t * dataSigned = (int8_t *)(dataBuf);

    if(MAX30105_writeReg(REG_TEMP_CONFIG, 0x01) != MAX30105_NO_ERROR) return MAX30105_TEMP_ERROR;
    nrf_delay_ms(30);
    if(MAX30105_readReg(REG_TEMP_CONFIG, &dataBuf[0]) != MAX30105_NO_ERROR) return MAX30105_TEMP_ERROR;
    while (dataBuf[0]) {
//        Thread::wait(1);
        nrf_delay_ms(1);
        if(MAX30105_readReg(REG_TEMP_CONFIG, &dataBuf[0]) != MAX30105_NO_ERROR) return MAX30105_TEMP_ERROR;
    }
    dataBuf[0] = REG_TEMP_INT;
    if(i2c_dev->read(MAX30105_I2C_ADDR, (char)REG_TEMP_INT, &dataBuf[0], 1) !=0) {
        return MAX30105_TEMP_ERROR;
    }
    return ((float)dataSigned[0] + ((float)dataSigned[1] * 0.0625));
}

//******************************************************************************
int MAX30105_shutDown()
{
    _modeConfiguration |= MASK_SHDN;
    if(MAX30105_writeReg(REG_MODE_CONFIG, _modeConfiguration) != MAX30105_NO_ERROR) return MAX30105_ERROR;
    return MAX30105_NO_ERROR;
}

//******************************************************************************
int MAX30105_wakeUp()
{
    _modeConfiguration &= ~MASK_SHDN;
    if(MAX30105_writeReg(REG_MODE_CONFIG, _modeConfiguration) != MAX30105_NO_ERROR) return MAX30105_ERROR;
    return MAX30105_NO_ERROR;        
}

//******************************************************************************
int MAX30105_softReset()
{
    if(MAX30105_writeReg(REG_MODE_CONFIG, MASK_RESET) != MAX30105_NO_ERROR) return MAX30105_ERROR;
    _interruptEnable1 = 0x00;
    _interruptEnable2 = 0x00;
    _fifoConfiguration = 0x00;
    _modeConfiguration = 0x00;
    _spo2Configuration = 0x00;
    _led1PulseAmplitude = 0x00;
    _led2PulseAmplitude = 0x00;
    _led3PulseAmplitude = 0x00;
    _pilotPulseAmplitude = 0x00;
    _multiLedControl1 = 0x00;
    _multiLedControl2 = 0x00;
    _proxIntThreshold = 0x00;
    return MAX30105_NO_ERROR;
}

//******************************************************************************
int MAX30105_clearFIFO() 
{
    if(MAX30105_writeReg(REG_FIFO_WR_PTR,0x00) != MAX30105_NO_ERROR) // INTR setting
        return MAX30105_ERROR;
    if(MAX30105_writeReg(REG_OVF_COUNTER,0x00) != MAX30105_NO_ERROR)
        return MAX30105_ERROR;
    if(MAX30105_writeReg(REG_FIFO_RD_PTR,0x00) != MAX30105_NO_ERROR)  //FIFO_WR_PTR[4:0]
        return MAX30105_ERROR;
    return MAX30105_NO_ERROR;
}
