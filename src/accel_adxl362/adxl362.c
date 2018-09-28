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
* @brief Analog Devices SPI ADXL375 Driver
* @defgroup 
*
*/

#include <stdbool.h>
#include "boards.h"
#include "nrf_drv_spi.h"
#include "nrf_drv_gpiote.h"
#include "app_util_platform.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "adxl362.h"
#include "app_error.h"
#include <string.h>
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#ifndef SPI_INSTANCE_ADXL362
#define SPI_INSTANCE_ADXL362  0 /**< SPI instance index. */
#define SPI_SCK_PIN_ADXL362   NRF_GPIO_PIN_MAP(1, 4)
#define SPI_MISO_PIN_ADXL362  NRF_GPIO_PIN_MAP(1, 2)
#define SPI_MOSI_PIN_ADXL362  NRF_GPIO_PIN_MAP(1, 3)
#define SPI_SS_PIN_ADXL362    NRF_GPIO_PIN_MAP(1, 1)
#define ADXL362_POWER_ON      NRF_GPIO_PIN_MAP(0, 16)
#define ADXL362_INT1          NRF_GPIO_PIN_MAP(1, 5)
#define ADXL362_INT2          NRF_GPIO_PIN_MAP(1, 6)
#endif

static const nrf_drv_spi_t p_spi_master_0 = NRF_DRV_SPI_INSTANCE(SPI_INSTANCE_ADXL362);
static volatile bool spi_xfer_done;  /**< Flag used to indicate that SPI instance completed the transfer. */
static uint8_t m_tx_data_spi[64]  = {0};
static uint8_t m_rx_data_spi[512] = {0}; 
static volatile bool is_active = false;
static int16_t xyz_data[3] = {0};

/**
 * @brief SPI user event handler.
 * @param event
 */
static void adxl362_spi_event_handler(nrf_drv_spi_evt_t const * p_event,
                       void *                    p_context)
{
    spi_xfer_done = true;
}

/**@brief ACTIVE state toggle handler, set to ACTIVE HIGH */
static void adxl362_int1_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    if(nrf_drv_gpiote_in_is_set(pin))
        is_active = true;    
    else
        is_active = false;
    NRF_LOG_INFO("ADXL362 active state change");
}

static void adxl362_int2_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    adxl362_readAccelerometerData(&xyz_data[0], &xyz_data[1], &xyz_data[2]);
}

/**@brief chip enable on */
void adxl362_enable(void)
{
    if(!nrf_drv_gpiote_is_init())
    {
        APP_ERROR_CHECK(nrf_drv_gpiote_init());
    }
    nrf_drv_gpiote_out_config_t out_config = GPIOTE_CONFIG_OUT_SIMPLE(true);
    APP_ERROR_CHECK(nrf_drv_gpiote_out_init(ADXL362_POWER_ON, &out_config));
}

/**@brief chip enable off */
void adxl362_disable(void)
{
    nrf_drv_gpiote_out_clear(ADXL362_POWER_ON);
}

/**@brief Function for initializing the ADXL375 SPI driver.
 *
 * @param[in] nrf_drv_spi_t * m_spi_master    Pointer to spi instance
 *
 */

bool adxl362_init(void)
{
    bool success = false;

    // chip enable
    adxl362_enable();

    // initialize spi driver 
    nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;
    spi_config.ss_pin   = SPI_SS_PIN_ADXL362;
    spi_config.miso_pin = SPI_MISO_PIN_ADXL362;
    spi_config.mosi_pin = SPI_MOSI_PIN_ADXL362;
    spi_config.sck_pin  = SPI_SCK_PIN_ADXL362;
    spi_config.frequency    = NRF_DRV_SPI_FREQ_4M; 
    APP_ERROR_CHECK(nrf_drv_spi_init(&p_spi_master_0, &spi_config, adxl362_spi_event_handler, NULL));

    NRF_LOG_INFO("Initializing ADXL362 over SPI...");

    // Check for expected values
    success = adxl362_verifyChip();

    // Configure timeout settings 
    adxl362_configTrigger();

    // Configure GPIO interrupts
    adxl362_configIntPins();

    // Get current active status in case we missed the trigger
    adxl362_int1_handler(ADXL362_INT1, NRF_GPIOTE_POLARITY_TOGGLE);

    // Get accel data to reset interrupt if we missed it
    adxl362_int2_handler(ADXL362_INT2, NRF_GPIOTE_POLARITY_LOTOHI);


    return success;
}

/**@brief Function for reading to the ADXL375 registers over SPI.
 *
 * @param[in] address   ADXL375 Register adress    
 * @param[in] p_data    Pointer to rx buffer 
 * @param[in] bytes     Number of bytes to be read 
 *
 */

void adxl362_readRegister(uint8_t address, uint8_t bytes)
{
    uint32_t err_code;
        
    m_tx_data_spi[0] = CMD_READ;    
    m_tx_data_spi[1] = address;

    spi_xfer_done = false;
    APP_ERROR_CHECK(nrf_drv_spi_transfer(&p_spi_master_0, m_tx_data_spi, bytes, m_rx_data_spi, bytes));
    while (!spi_xfer_done) __WFE();
}

/**@brief Function for writing to the ADXL375 registers over SPI.
 *
 * @param[in] address   ADXL375 Register adress    
 * @param[in] p_data    Pointer to tx buffer 
 * @param[in] bytes     Number of bytes to be written
 *
 */

void adxl362_writeRegister(uint8_t address, uint8_t bytes)
{
    uint32_t err_code;
    
    m_tx_data_spi[0] = CMD_WRITE;    
    m_tx_data_spi[1] = address;
    
    spi_xfer_done = false;
    APP_ERROR_CHECK(nrf_drv_spi_transfer(&p_spi_master_0, m_tx_data_spi, bytes, NULL, 0));
    while (!spi_xfer_done) __WFE();
}

/**@brief Function for verfiying the presence of the ADXL362 . */

bool adxl362_verifyChip(void)
{
    bool success = true;
    uint8_t num_bytes_read = 4;

    adxl362_readRegister(REG_DEVICE_ID_1, num_bytes_read);
    if(m_rx_data_spi[2] != DEVICE_ID_1) success = false;

    adxl362_readRegister(REG_DEVICE_ID_2, num_bytes_read);
    if(m_rx_data_spi[2] != DEVICE_ID_2) success = false;

    adxl362_readRegister(REG_PART_ID,num_bytes_read);
    if(m_rx_data_spi[2] != PART_ID) success = false;

    if(!success)
    { 
        NRF_LOG_INFO("Unable to detect ADXL362!");
    }
    else
    {
        NRF_LOG_INFO("ADXL362 found!");
    }
    return success;
}

/**@brief Function for verfiying the presence of the ADXL362 . */

void adxl362_configTrigger(void)
{
    static uint8_t def_trig_settings[] = {0x3A, 0x00, 0x03, 0x58, 0x00, 0xC2, 0x01, 0x3F, 0x00, 0x80, 0x40, 0x01, 0x10, 0x0E}; 
    memcpy(&m_tx_data_spi[2], def_trig_settings, sizeof(def_trig_settings));
    
    // multi-byte write starting at the first write register
    adxl362_writeRegister(REG_THRESH_ACT_L, sizeof(def_trig_settings) + 2);
}

/**@brief Function for configuring interrupt handlers for the ADXL362 . */
void adxl362_configIntPins(void)
{
    // config ACTIVE state interrupt
    nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_TOGGLE(true);
    APP_ERROR_CHECK(nrf_drv_gpiote_in_init(ADXL362_INT1, &in_config, adxl362_int1_handler));
    nrf_drv_gpiote_in_event_enable(ADXL362_INT1, true);

    // config DATA RDY interrupt
    nrf_drv_gpiote_in_config_t in_config1 = GPIOTE_CONFIG_IN_SENSE_LOTOHI(true);
    APP_ERROR_CHECK(nrf_drv_gpiote_in_init(ADXL362_INT2, &in_config1, adxl362_int2_handler));
    nrf_drv_gpiote_in_event_enable(ADXL362_INT2, true);
} 

/**@brief Function for reading the data registers of the ADXL375 .
 *
 * @param[in] x_val         Pointer to x value  
 * @param[in] y_val         Pointer to y value    
 * @param[in] z_val         Pointer to z value    
 *
 */

void adxl362_readAccelerometerData(int16_t *x_val, int16_t *y_val, int16_t *z_val)
{  
    uint8_t num_bytes_read = 8;
    
    adxl362_readRegister(REG_XDATA_L,num_bytes_read);
 
    *x_val = (m_rx_data_spi[3]<<8) | m_rx_data_spi[2];
    *y_val = (m_rx_data_spi[5]<<8) | m_rx_data_spi[4];
    *z_val = (m_rx_data_spi[7]<<8) | m_rx_data_spi[6];
    
    NRF_LOG_INFO(0,"X:  %d  Y:  %d  Z:  %d  \n",*x_val,*y_val,*z_val);        
}

