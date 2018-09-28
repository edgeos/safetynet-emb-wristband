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
* @brief Example template project.
* @defgroup nrf_templates_example Example Template
*
*/

#include <stdbool.h>
#include "SEGGER_RTT.h"
#include "nrf_delay.h"
#include "nrf_drv_spi.h"
#include "ADXL375_SPI.h"
#include "boards.h"
#include "app_error.h"
#include "app_util_platform.h"
#include "nrf_gpio.h"
#include <string.h>


#define SPIM0_CS_PIN    12 

static const nrf_drv_spi_t m_spi_master = NRF_DRV_SPI_INSTANCE(0);

/**@brief Handler for SPI0 master events.
 *
 * @param[in] event SPI master event.
 */
 
 /*
void spi_master_0_event_handler(nrf_drv_spi_event_t event)
{
      uint32_t err_code = NRF_SUCCESS;
   // bool result = false;

    switch (event)
    {
        case NRF_DRV_SPI_EVENT_DONE:
            // Process received data
            //nrf_gpio_pin_set(LED_1);
            spi_busy = 0;
            break;

        default:
            // No implementation needed.
            break;
    }
    APP_ERROR_CHECK(err_code);
}
*/
/**@brief Function for initializing a SPI master driver.
 *
 * @param[in] p_instance    Pointer to SPI master driver instance.
 * @param[in] lsb           Bits order LSB if true, MSB if false.
 */
 
static void spi_master_init(nrf_drv_spi_t const * p_instance, bool lsb)
{
    uint32_t err_code;

    nrf_drv_spi_config_t config =
    {
        .ss_pin       = NRF_DRV_SPI_PIN_NOT_USED, 
        .irq_priority = APP_IRQ_PRIORITY_LOW,
        .orc          = 0xCC,
        .frequency    = NRF_DRV_SPI_FREQ_1M, 
        .mode         = NRF_DRV_SPI_MODE_3, 
        .bit_order    = (lsb ?
            NRF_DRV_SPI_BIT_ORDER_LSB_FIRST : NRF_DRV_SPI_BIT_ORDER_MSB_FIRST),
    };
    
    #if (SPI0_ENABLED == 1)
    if (p_instance == &m_spi_master)
    {
        config.sck_pin  = SPIM0_SCK_PIN;
        config.mosi_pin = SPIM0_MOSI_PIN;
        config.miso_pin = SPIM0_MISO_PIN;
        err_code = nrf_drv_spi_init(p_instance, &config,NULL); //spi_master_0_event_handler
        
        APP_ERROR_CHECK(err_code);
    }
    else
    {
        // Do nothing
    }
    #endif // (SPI0_ENABLED == 1)
}


/**
 * @brief Function for application main entry.
 */
int main(void)
{  
    // Initialize the SPI instance
    spi_master_init(&m_spi_master, false);
    // Initialize the ADXL375 SPI Driver 
    ADXL375_SPI_init(&m_spi_master);

    
    // Accelerometer Data Buffer
    uint8_t rx_buffer[7];
    
    // Sensor Values
    int16_t x_value;
    int16_t y_value;
    int16_t z_value;
    
    
    ADXL375_SPI_readAccelerometerData(rx_buffer,sizeof(rx_buffer),&x_value,&y_value,&z_value);
  
    while (true)
    {   
        // Read the data registers of the ADXL375 
        ADXL375_SPI_readAccelerometerData(rx_buffer,sizeof(rx_buffer),&x_value,&y_value,&z_value);
        // Delay for 1s
        nrf_delay_ms(1000);
    }
}
/** @} */
