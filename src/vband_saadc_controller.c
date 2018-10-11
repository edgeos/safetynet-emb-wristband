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
#include "nrf.h"
#include "nrf_drv_saadc.h"
#include "nrf_drv_rtc.h"
#include "boards.h"
#include "app_error.h"
#include "app_util_platform.h"
#include "vband_saadc_controller.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#define SAADC_CALIBRATION_INTERVAL 5    // Determines how often the SAADC should be calibrated relative to NRF_DRV_SAADC_EVT_DONE event. E.g. value 5 will make the SAADC calibrate every fifth time the NRF_DRV_SAADC_EVT_DONE is received.
#define SAADC_SAMPLES_IN_BUFFER    3    // Number of SAADC samples in RAM before returning a SAADC event. For low power SAADC set this constant to 1. Otherwise the EasyDMA will be enabled for an extended time which consumes high current.
#define SAADC_OVERSAMPLE           NRF_SAADC_OVERSAMPLE_DISABLED  //Oversampling setting for the SAADC. Setting oversample to 4x This will make the SAADC output a single averaged value when the SAMPLE task is triggered 4 times. Enable BURST mode to make the SAADC sample 4 times when triggering SAMPLE task once.
#define SAADC_BURST_MODE           0    //Set to 1 to enable BURST mode, otherwise set to 0.

#define RTC_FREQUENCY              1024 // must be a factor of 32768
#define RTC_CC_VALUE               1    //Determines the RTC interrupt frequency and thereby the SAADC sampling frequency
#define NUM_MEASUREMENTS           64

const  nrf_drv_rtc_t           rtc = NRF_DRV_RTC_INSTANCE(2); // Declaring an instance of nrf_drv_rtc for RTC2. */

static nrf_saadc_value_t       m_buffer_pool[2][SAADC_SAMPLES_IN_BUFFER];
static uint32_t                m_adc_evt_counter = 0;
static bool                    m_saadc_initialized = false;
static bool                    m_rtc_initialized = false;
static nrf_saadc_value_t       m_electrode_saadc_vals_temp[3][NUM_MEASUREMENTS];
static nrf_saadc_value_t       m_electrode_saadc_vals_worker[3][NUM_MEASUREMENTS];
static uint16_t                m_electrode_measurement_counter = 0;
static bool                    m_electrode_measurement_in_progress = false;

/** @brief Function callback for SAADC electrode measurement done
 */
static void saadc_electrode_callback(nrf_drv_saadc_evt_t const * p_event)
{
    if (p_event->type == NRF_DRV_SAADC_EVT_DONE)                                                        //Capture offset calibration complete event
    {
        ret_code_t err_code;
        err_code = nrf_drv_saadc_buffer_convert(p_event->data.done.p_buffer, SAADC_SAMPLES_IN_BUFFER);  //Set buffer so the SAADC can write to it again. This is either "buffer 1" or "buffer 2"
        APP_ERROR_CHECK(err_code);
        //NRF_LOG_INFO("ADC event number: %d\r\n",(int)m_adc_evt_counter);                                      //Print the event number on UART
        for (int i = 0; i < SAADC_SAMPLES_IN_BUFFER; i++)
        {
            //copy measurements to array
            m_electrode_saadc_vals_temp[i][m_electrode_measurement_counter] = p_event->data.done.p_buffer[i];
            //NRF_LOG_INFO("%d\r\n", p_event->data.done.p_buffer[i]);                                           //Print the SAADC result on UART
        }
        
        m_electrode_measurement_counter++;
        nrf_drv_saadc_uninit();                                                                   //Unintialize SAADC to disable EasyDMA and save power
        NRF_SAADC->INTENCLR = (SAADC_INTENCLR_END_Clear << SAADC_INTENCLR_END_Pos);               //Disable the SAADC interrupt
        NVIC_ClearPendingIRQ(SAADC_IRQn);                                                         //Clear the SAADC interrupt if set
        m_saadc_initialized = false;                                                              //Set SAADC as uninitialized
    }
}

/** @brief Function to initialize the SAADC settings for the electrode channels
 */
static void vband_saadc_electrode_channels_init(void)
{
    ret_code_t err_code;
    nrf_drv_saadc_config_t saadc_config;
    nrf_saadc_channel_config_t channel_config0;
    nrf_saadc_channel_config_t channel_config1;
    nrf_saadc_channel_config_t channel_config2;
	
    //Configure SAADC
    saadc_config.resolution = NRF_SAADC_RESOLUTION_12BIT;                                 //Set SAADC resolution to 12-bit. This will make the SAADC output values from 0 (when input voltage is 0V) to 2^12=2048 (when input voltage is 3.6V for channel gain setting of 1/6).
    saadc_config.oversample = SAADC_OVERSAMPLE;                                           //Set oversample to 4x. This will make the SAADC output a single averaged value when the SAMPLE task is triggered 4 times.
    saadc_config.interrupt_priority = APP_IRQ_PRIORITY_LOW;                               //Set SAADC interrupt to low priority.
    saadc_config.low_power_mode = true;                                                   //Set SAADC interrupt to low priority.
	
    //Initialize SAADC
    APP_ERROR_CHECK(nrf_drv_saadc_init(&saadc_config, saadc_electrode_callback));                   //Initialize the SAADC with configuration and callback function. The application must then implement the saadc_callback function, which will be called when SAADC interrupt is triggered
		
    //Configure SAADC channel
    channel_config0.reference = NRF_SAADC_REFERENCE_INTERNAL;                              //Set internal reference of fixed 0.6 volts
    channel_config0.gain = NRF_SAADC_GAIN1_6;                                              //Set input gain to 1/6. The maximum SAADC input voltage is then 0.6V/(1/6)=3.6V. The single ended input range is then 0V-3.6V
    channel_config0.acq_time = NRF_SAADC_ACQTIME_10US;                                     //Set acquisition time. Set low acquisition time to enable maximum sampling frequency of 200kHz. Set high acquisition time to allow maximum source resistance up to 800 kohm, see the SAADC electrical specification in the PS. 
    channel_config0.mode = NRF_SAADC_MODE_SINGLE_ENDED;                                    //Set SAADC as single ended. This means it will only have the positive pin as input, and the negative pin is shorted to ground (0V) internally.
    channel_config0.pin_p = NRF_SAADC_INPUT_AIN0;                                          //Select the input pin for the channel. AIN0 pin maps to physical pin P0.02.
    channel_config0.pin_n = NRF_SAADC_INPUT_DISABLED;                                      //Since the SAADC is single ended, the negative pin is disabled. The negative pin is shorted to ground internally.
    channel_config0.resistor_p = NRF_SAADC_RESISTOR_DISABLED;                              //Disable pullup resistor on the input pin
    channel_config0.resistor_n = NRF_SAADC_RESISTOR_DISABLED;                              //Disable pulldown resistor on the input pin

    channel_config1.reference = NRF_SAADC_REFERENCE_INTERNAL;                              //Set internal reference of fixed 0.6 volts
    channel_config1.gain = NRF_SAADC_GAIN1_6;                                              //Set input gain to 1/6. The maximum SAADC input voltage is then 0.6V/(1/6)=3.6V. The single ended input range is then 0V-3.6V
    channel_config1.acq_time = NRF_SAADC_ACQTIME_10US;                                     //Set acquisition time. Set low acquisition time to enable maximum sampling frequency of 200kHz. Set high acquisition time to allow maximum source resistance up to 800 kohm, see the SAADC electrical specification in the PS. 
    channel_config1.mode = NRF_SAADC_MODE_SINGLE_ENDED;                                    //Set SAADC as single ended. This means it will only have the positive pin as input, and the negative pin is shorted to ground (0V) internally.
    channel_config1.pin_p = NRF_SAADC_INPUT_AIN4;                                          //Select the input pin for the channel. AIN0 pin maps to physical pin P0.02.
    channel_config1.pin_n = NRF_SAADC_INPUT_DISABLED;                                      //Since the SAADC is single ended, the negative pin is disabled. The negative pin is shorted to ground internally.
    channel_config1.resistor_p = NRF_SAADC_RESISTOR_DISABLED;                              //Disable pullup resistor on the input pin
    channel_config1.resistor_n = NRF_SAADC_RESISTOR_DISABLED;                              //Disable pulldown resistor on the input pin
    
    channel_config2.reference = NRF_SAADC_REFERENCE_INTERNAL;                              //Set internal reference of fixed 0.6 volts
    channel_config2.gain = NRF_SAADC_GAIN1_6;                                              //Set input gain to 1/6. The maximum SAADC input voltage is then 0.6V/(1/6)=3.6V. The single ended input range is then 0V-3.6V
    channel_config2.acq_time = NRF_SAADC_ACQTIME_10US;                                     //Set acquisition time. Set low acquisition time to enable maximum sampling frequency of 200kHz. Set high acquisition time to allow maximum source resistance up to 800 kohm, see the SAADC electrical specification in the PS. 
    channel_config2.mode = NRF_SAADC_MODE_SINGLE_ENDED;                                    //Set SAADC as single ended. This means it will only have the positive pin as input, and the negative pin is shorted to ground (0V) internally.
    channel_config2.pin_p = NRF_SAADC_INPUT_AIN6;                                          //Select the input pin for the channel. AIN0 pin maps to physical pin P0.02.
    channel_config2.pin_n = NRF_SAADC_INPUT_DISABLED;                                      //Since the SAADC is single ended, the negative pin is disabled. The negative pin is shorted to ground internally.
    channel_config2.resistor_p = NRF_SAADC_RESISTOR_DISABLED;                              //Disable pullup resistor on the input pin
    channel_config2.resistor_n = NRF_SAADC_RESISTOR_DISABLED;                              //Disable pulldown resistor on the input pin
	
    //Initialize SAADC channel
    err_code = nrf_drv_saadc_channel_init(0, &channel_config0);                            //Initialize SAADC channel 0 with the channel configuration
    APP_ERROR_CHECK(err_code);
    err_code = nrf_drv_saadc_channel_init(1, &channel_config1);                            //Initialize SAADC channel 0 with the channel configuration
    APP_ERROR_CHECK(err_code);
    err_code = nrf_drv_saadc_channel_init(2, &channel_config2);                            //Initialize SAADC channel 0 with the channel configuration
    APP_ERROR_CHECK(err_code);
		
    if(SAADC_BURST_MODE)
    {
        NRF_SAADC->CH[0].CONFIG |= 0x01000000;                                            //Configure burst mode for channel 0. Burst is useful together with oversampling. When triggering the SAMPLE task in burst mode, the SAADC will sample "Oversample" number of times as fast as it can and then output a single averaged value to the RAM buffer. If burst mode is not enabled, the SAMPLE task needs to be triggered "Oversample" number of times to output a single averaged value to the RAM buffer.		
        NRF_SAADC->CH[1].CONFIG |= 0x01000000; 
        NRF_SAADC->CH[2].CONFIG |= 0x01000000; 
    }

    err_code = nrf_drv_saadc_buffer_convert(m_buffer_pool[0],SAADC_SAMPLES_IN_BUFFER);    //Set SAADC buffer 1. The SAADC will start to write to this buffer
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_buffer_convert(m_buffer_pool[1],SAADC_SAMPLES_IN_BUFFER);    //Set SAADC buffer 1. The SAADC will start to write to this buffer
    APP_ERROR_CHECK(err_code);

    m_saadc_initialized = true; 
}

/** @brief Function callback for SAADC electrode measurement timer, will start the task for SAADC measurements
 */
static void vband_electrode_sample_rtc_handler(nrf_drv_rtc_int_type_t int_type)
{
    uint32_t err_code;
    uint32_t current_ctr;
	
    if (int_type == NRF_DRV_RTC_INT_COMPARE0)
    {
        if(!m_saadc_initialized)
        {
            vband_saadc_electrode_channels_init();                     //Initialize the SAADC. In the case when SAADC_SAMPLES_IN_BUFFER > 1 then we only need to initialize the SAADC when the the buffer is empty.
        }
        nrf_drv_saadc_sample();                                        //Trigger the SAADC SAMPLE task
               
        current_ctr =0;// nrf_drv_rtc_counter_get(&rtc); //
        err_code = nrf_drv_rtc_cc_set(&rtc,0,RTC_CC_VALUE,true);       //Set RTC compare value. This needs to be done every time as the nrf_drv_rtc clears the compare register on every compare match
        nrf_drv_rtc_counter_clear(&rtc);                               //Clear the RTC counter to start count from zero
        APP_ERROR_CHECK(err_code);
          
        // stop after predefined number of measurements
        if (m_electrode_measurement_counter == NUM_MEASUREMENTS)
        {   
            m_adc_evt_counter++;
            NRF_LOG_INFO("ADC event number: %d, AIN0 = %d, AIN4 = %d, AIN6 = %d",(int)m_adc_evt_counter,m_electrode_saadc_vals_temp[0][0],m_electrode_saadc_vals_temp[1][0],m_electrode_saadc_vals_temp[2][0]);
            m_electrode_measurement_counter = 0;
            m_electrode_measurement_in_progress = false;
            nrf_drv_rtc_disable(&rtc);
            nrf_drv_rtc_uninit(&rtc);
        }
    }
}


/** @brief Function to initialize the SAADC electrode measurement timer
 */
static void vband_electrode_sample_rtc_config(void)
{

    uint32_t err_code;

    //Initialize RTC instance
    nrf_drv_rtc_config_t rtc_config;
    rtc_config.interrupt_priority = APP_IRQ_PRIORITY_LOW;
    rtc_config.prescaler = RTC_FREQ_TO_PRESCALER(RTC_FREQUENCY);
    rtc_config.reliable = false;
    err_code = nrf_drv_rtc_init(&rtc, &rtc_config, vband_electrode_sample_rtc_handler);  //Initialize the RTC with callback function rtc_handler. The rtc_handler must be implemented in this applicaiton. Passing NULL here for RTC configuration means that configuration will be taken from the sdk_config.h file.
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_rtc_cc_set(&rtc,0,RTC_CC_VALUE,true);           //Set RTC compare value to trigger interrupt. Configure the interrupt frequency by adjust RTC_CC_VALUE and RTC_FREQUENCY constant in top of main.c
    APP_ERROR_CHECK(err_code);

    m_rtc_initialized = true;

    //Power on RTC instance
    nrf_drv_rtc_enable(&rtc);                                          //Enable RTC
}

/** @brief Function to sample the SAADC electrode channels
 */
void vband_saadc_sample_electrode_channels(void)
{   
    // defer if measurement still in progress for any reason
    if(!m_electrode_measurement_in_progress)
    {
        // protect the data array until it's ready
        m_electrode_measurement_in_progress = true;

        // start the RTC that controls the sampling intervals for the electrode channels
        vband_electrode_sample_rtc_config();
    }
}
/** @} */