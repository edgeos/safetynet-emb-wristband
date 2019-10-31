/* Copyright (c) 2009 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

/** @file
* @brief CCS811 Air quality sensor I2C Driver
* @defgroup 
*
*/

#include <stdbool.h>
#include "boards.h"
#include "nrf_drv_gpiote.h"
#include "app_util_platform.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "ccs811.h"
#include "app_error.h"
#include <string.h>
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

static uint8_t m_sample[20];

#ifndef CCS811_RESET_PIN
#define CCS811_RESET_PIN          NRF_GPIO_PIN_MAP(1,3)      // Enable signal pin
#define CCS811_WAKEUP_PIN         NRF_GPIO_PIN_MAP(1,2)      // Wakeup signal pin
#endif

/**@brief I2C enable on - WAKEUP pin control */
void ccs811_chip_reset(void)
{
    static bool pin_cfg = false;
    if(!nrf_drv_gpiote_is_init())
    {
        APP_ERROR_CHECK(nrf_drv_gpiote_init());
    }
    if(!pin_cfg)
    {
        nrf_drv_gpiote_out_config_t out_config = GPIOTE_CONFIG_OUT_SIMPLE(false); // false = init low = WAKEUP
        APP_ERROR_CHECK(nrf_drv_gpiote_out_init(CCS811_WAKEUP_PIN, &out_config));
        APP_ERROR_CHECK(nrf_drv_gpiote_out_init(CCS811_RESET_PIN, &out_config));
        pin_cfg = true;
    }
    nrf_drv_gpiote_out_clear(CCS811_RESET_PIN);
    nrf_delay_ms(10);
    nrf_drv_gpiote_out_set(CCS811_RESET_PIN);
    nrf_delay_ms(10); // datasheet says max reset time is 2ms
}

/**@brief I2C enable on - WAKEUP pin control */
void ccs811_twi_enable(void)
{
    //nrf_gpio_pin_clear(CCS811_WAKEUP_PIN);
    nrf_drv_gpiote_out_clear(CCS811_WAKEUP_PIN);
    nrf_delay_us(500);
}

/**@brief I2C enable off - WAKEUP pin control*/
void ccs811_twi_disable(void)
{
    //nrf_gpio_pin_set(CCS811_WAKEUP_PIN);
    nrf_drv_gpiote_out_set(CCS811_WAKEUP_PIN); // set = high = off
}

/**@brief Function for initializing the CCS811 I2C driver.
 *
 * @param[in] struct ccs811_dev *dev    Pointer to i2c instance
 *
 */
bool ccs811_init(struct ccs811_dev *dev)
{
    // turn the chip on and config pins
    ccs811_chip_reset(); // reset pin not used in this board

    // enable i2c comms
    ccs811_twi_enable();
  
    /* Verify presence of the ccs811 */
    uint8_t rx_buffer[4];
    dev->read(dev->dev_id, CCS811_REG_HW_ID, rx_buffer, 1);

    // disable i2c comms
    ccs811_twi_disable();

    /* Verify Device ID matches expected */
    return (rx_buffer[0] == CCS811_HW_ID) ? true:false;
}

bool ccs811_start_mode(struct ccs811_dev *dev, uint8_t mode)
{
    // enable i2c comms
    ccs811_twi_enable();

    /* put device into APP_START, just write to the REG_APP_START register */
    dev->write(dev->dev_id, CCS811_REG_APP_START, NULL, 0);
    nrf_delay_us(200);

    uint8_t reg_mode[2] = {mode, 0}; //0x30 = measure every 60 sec, 0x10 = every second
    dev->write(dev->dev_id, CCS811_REG_MEAS_MODE, reg_mode, 1);
    nrf_delay_us(200);

    // verify status register
    uint8_t rx_buffer[4];
    uint8_t rx_buffer2[4];
    dev->read(dev->dev_id, CCS811_REG_STATUS, rx_buffer, 1);
    dev->read(dev->dev_id, CCS811_REG_MEAS_MODE, rx_buffer2, 1);

    // disable i2c comms
    ccs811_twi_disable();

    uint8_t status_rdy = rx_buffer[0] & 0x91; // bit 0 is active error
    return (status_rdy == 0x90) ? true:false;
}

/**@brief reset if error found */
void ccs811_reset(struct ccs811_dev *dev)
{
    // enable i2c comms
    ccs811_twi_enable();

    // read the error register
    uint8_t reset_reg[2] = {0, 0};
    dev->write(dev->dev_id, CCS811_REG_ERROR_ID, &reset_reg[0], 1);
    nrf_delay_us(50);

    // sent reset code
    uint8_t sw_reset[4] = {0x11, 0xE5, 0x72, 0x8A};
    dev->write(dev->dev_id, CCS811_REG_SW_RESET, sw_reset, 4);
    nrf_delay_us(50);
          
    // disable i2c comms
    ccs811_twi_disable();
}

/**@brief full calibration sequence */
void ccs811_calibrate(struct ccs811_dev *dev, int16_t temperature_c, uint8_t humidity_pct)
{
    // format and write temperature values
    uint8_t cal_vals[4] = {(uint8_t)((temperature_c+25) << 1), 0, (uint8_t)(humidity_pct << 1), 0};
    dev->write(dev->dev_id, CCS811_REG_ENV_DATA, &cal_vals[0], 4);
    nrf_delay_us(50);
}

/**@brief read measurement */
bool ccs811_measure(struct ccs811_dev *dev, uint16_t *eCO2, uint16_t *TVOC, uint8_t* pError)
{
    // enable i2c comms
    ccs811_twi_enable();

    // read the status register to check for a new measurement
    uint8_t rx_buffer[4];
    dev->read(dev->dev_id, CCS811_REG_STATUS, rx_buffer, 1);
    uint8_t status_rdy = rx_buffer[0] & 0x99; // bit 0 is active error
    dev->read(dev->dev_id, CCS811_REG_MEAS_MODE, rx_buffer + 1, 1);
    uint8_t meas_mode = rx_buffer[1];
	if(pError) dev->read(dev->dev_id, CCS811_REG_ERROR_ID, pError, 1);
    if (status_rdy == 0x98) {
        dev->read(dev->dev_id, CCS811_REG_ALG_RESULTS_DATA, rx_buffer, 4);
        *eCO2 = (uint16_t) (rx_buffer[0] << 8) + rx_buffer[1];
        *TVOC = (uint16_t) (rx_buffer[2] << 8) + rx_buffer[3];
    }
    // disable i2c comms
    ccs811_twi_disable();
    return (status_rdy == 0x98) ? true:false;
}

/**@brief idle mode has lowest consumption */
bool ccs811_idle(struct ccs811_dev *dev)
{
    // enable i2c comms
    ccs811_twi_enable();

    uint8_t reg_mode[2] = {0x00, 0}; // 0x00 goes to IDLE mode
    dev->read(dev->dev_id, CCS811_REG_MEAS_MODE, reg_mode, 1);

    uint8_t rx_buffer[2] = {0xff};
    dev->read(dev->dev_id, CCS811_REG_MEAS_MODE, rx_buffer, 1);

    // disable i2c comms
    ccs811_twi_disable();
    return (rx_buffer[0] == 0x00) ? true:false;
}