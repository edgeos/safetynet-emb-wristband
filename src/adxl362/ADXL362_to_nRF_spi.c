/***************************************************************************//**
 *   @file   ADXL362_to_nRF_spi.c
 *   @brief  Implementation of Communication Driver.
 *   @author DBogdan (dragos.bogdan@analog.com)
********************************************************************************
 * Copyright 2012(c) Analog Devices, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *  - Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  - Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *  - Neither the name of Analog Devices, Inc. nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *  - The use of this software may or may not infringe the patent rights
 *    of one or more patent holders.  This license does not release you
 *    from the requirement that you obtain separate licenses from these
 *    patent holders to use this software.
 *  - Use of the software either in source or binary form, must be run
 *    on or directly connected to an Analog Devices Inc. component.
 *
 * THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT,
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL ANALOG DEVICES BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, INTELLECTUAL PROPERTY RIGHTS, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
********************************************************************************
 *   SVN Revision: 797
*******************************************************************************/

/******************************************************************************/
/* Include Files                                                              */
/******************************************************************************/
#include <stdbool.h>
#include "boards.h"
#include "nrf_drv_spi.h"
//#include "nrf_drv_gpiote.h"
//#include "app_util_platform.h"
//#include "nrf_gpio.h"
//#include "nrf_delay.h"
#include "app_error.h"
//#include <string.h>
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "ADXL362_to_nRF_spi.h"

#define SPI_BUFFER_SIZE 32

static const nrfx_spim_t p_spim = NRFX_SPIM_INSTANCE(SPI_INSTANCE_ADXL362);
static volatile bool spi_xfer_done;  /**< Flag used to indicate that SPI instance completed the transfer. */
static volatile bool spi_initialized;  /**< Flag used to indicate that SPI instance has been initialized. */
static unsigned char tx_buffer[SPI_BUFFER_SIZE] = {0};
static unsigned char rx_buffer[SPI_BUFFER_SIZE] = {0};

/**
 * @brief SPI user event handler.
 * @param event
 */
static void spi_event_handler(nrfx_spim_evt_t const * p_event, void * p_context)
{
  if(p_event->type == NRFX_SPIM_EVENT_DONE)
  {
    spi_xfer_done = true;
  }
  //else NRF_LOG_INFO("some other events from spi");
}

/***************************************************************************//**
 * @brief Initializes the SPI communication peripheral.
 *
 * @param lsbFirst - Transfer format (0 or 1).
 *                   Example: 0x0 - MSB first.
 *                            0x1 - LSB first.
 * @param clockFreq - SPI clock frequency (Hz).
 *                    Example: 1000 - SPI clock frequency is 1 kHz.
 * @param clockPol - SPI clock polarity (0 or 1).
 *                   Example: 0x0 - Idle state for clock is a low level; active
 *                                  state is a high level;
 *	                      0x1 - Idle state for clock is a high level; active
 *                                  state is a low level.
 * @param clockEdg - SPI clock edge (0 or 1).
 *                   Example: 0x0 - Serial output data changes on transition
 *                                  from idle clock state to active clock state;
 *                            0x1 - Serial output data changes on transition
 *                                  from active clock state to idle clock state.
 *
 * @return status - Result of the initialization procedure.
 *                  Example: 1 - if initialization was successful;
 *                           0 - if initialization was unsuccessful.
*******************************************************************************/
unsigned char SPI_Init(unsigned char lsbFirst,
                       unsigned long clockFreq,
                       unsigned char clockPol,
                       unsigned char clockEdg)
{
  if(!spi_initialized)
  {
    // initialize spi driver 
    nrfx_spim_config_t config = NRFX_SPIM_DEFAULT_CONFIG;
    config.sck_pin        = SPI_SCK_PIN_ADXL362;
    config.mosi_pin       = SPI_MOSI_PIN_ADXL362;
    config.miso_pin       = SPI_MISO_PIN_ADXL362;
    config.ss_pin         = SPI_SS_PIN_ADXL362;
//    config.ss_active_high = false,                  // use default
//    config.irq_priority   = p_config->irq_priority; // use default
//    config.orc            = p_config->orc; // use default
    config.frequency      = NRF_SPIM_FREQ_1M;
//    config.mode           = (nrf_spim_mode_t)p_config->mode; // use default
//    config.bit_order      = (nrf_spim_bit_order_t)p_config->bit_order; // use default

    APP_ERROR_CHECK(nrfx_spim_init(&p_spim, &config, spi_event_handler, NULL));
    spi_initialized = true;
  }
  return 1;
}

void SPI_uninit(void)
{
  ret_code_t err_code = 0;
  
  //NRF_LOG_INFO("entered SPI_uninit, before while loop");
  // hard fault after the above line? at 0xFFFFFF00 or 0xFFFF00F8

  // wait until transfers are done
  while(!spi_xfer_done)
  {
//      err_code++;
//    err_code = sd_app_evt_wait();
//    APP_ERROR_CHECK(err_code);
  }
//  NRF_LOG_INFO("going to call nrfx_spim_uninit, waited %d times",err_code);
//  NRF_LOG_FLUSH();
  nrfx_spim_uninit(&p_spim);
  spi_initialized = false;
/*
  // workaround for errata 89 static current consumption 400uA
  *(volatile uint32_t *)0x40004FFC = 0;
  *(volatile uint32_t *)0x40004FFC;
  *(volatile uint32_t *)0x40004FFC = 1;
  */
}

/***************************************************************************//**
 * @brief Reads data from SPI.
 *
 * @param slaveDeviceId - The ID of the selected slave device.
 * @param data - Data represents the write buffer as an input parameter and the
 *               read buffer as an output parameter.
 * @param bytesNumber - Number of bytes to read.
 *
 * @return Number of read bytes.
*******************************************************************************/
unsigned char SPI_Read(unsigned char slaveDeviceId,
                       unsigned char* data,
                       unsigned char bytesNumber)
{
    ret_code_t err_code;

    if(bytesNumber <= SPI_BUFFER_SIZE)
    {
      spi_xfer_done = false;
      do
      {
        nrfx_spim_xfer_desc_t xfer = NRFX_SPIM_SINGLE_XFER(data, bytesNumber, rx_buffer, bytesNumber);
        err_code = nrfx_spim_xfer(&p_spim, &xfer, 0);
      } while(err_code == NRF_ERROR_BUSY);
      do
      {
        APP_ERROR_CHECK(err_code);
  //      err_code = sd_app_evt_wait(); // causes hardfault
      } while(!spi_xfer_done);
      memcpy(data, &rx_buffer[0], bytesNumber);
    }
    else
    {
      // something is not right
      ASSERT(false);
    }
}

/***************************************************************************//**
 * @brief Writes data to SPI.
 *
 * @param slaveDeviceId - The ID of the selected slave device.
 * @param data - Data represents the write buffer.
 * @param bytesNumber - Number of bytes to write.
 *
 * @return Number of written bytes.
*******************************************************************************/
unsigned char SPI_Write(unsigned char slaveDeviceId,
                        unsigned char* data,
                        unsigned char bytesNumber)
{
    ret_code_t err_code;
    spi_xfer_done = false;
    if(bytesNumber <= SPI_BUFFER_SIZE)
    {
      memcpy(&tx_buffer[0], data, bytesNumber);
    }
    else
    {
      // something is not right
      ASSERT(false);
    }
    do
    {
      nrfx_spim_xfer_desc_t xfer = NRFX_SPIM_SINGLE_XFER(tx_buffer, bytesNumber, rx_buffer, bytesNumber);
      err_code = nrfx_spim_xfer(&p_spim, &xfer, 0);
    } while(err_code == NRFX_ERROR_BUSY);
    APP_ERROR_CHECK(err_code);
}
