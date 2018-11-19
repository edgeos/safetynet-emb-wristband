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
#define NUM_MEASUREMENTS           128

const  nrf_drv_rtc_t           rtc = NRF_DRV_RTC_INSTANCE(2); // Declaring an instance of nrf_drv_rtc for RTC2. */

static nrf_saadc_value_t       m_buffer_pool[2][SAADC_SAMPLES_IN_BUFFER];
static uint32_t                m_adc_evt_counter = 0;
static bool                    m_saadc_initialized = false;
static bool                    m_rtc_initialized = false;
static float                   m_electrode_saadc_vals_temp[3][NUM_MEASUREMENTS];
static nrf_saadc_value_t       m_electrode_saadc_vals_worker[3][NUM_MEASUREMENTS];
static uint16_t                m_electrode_measurement_counter = 0;
static bool                    m_electrode_measurement_in_progress = false;
static float                   vRef = 3.6;
static float                   resCts = 16384; // 8-bit = 255, 10-bit = 1024, 12-bit = 4096, 14-bit = 16384

static saadc_finished_fnptr_t saadc_task_finished_fn = NULL;

static void clear_FPU_interrupts(void)
{
    __set_FPSCR(__get_FPSCR() & ~(0x0000009F)); 
    (void) __get_FPSCR();
    NVIC_ClearPendingIRQ(FPU_IRQn);
}

/** @brief Function callback for SAADC electrode measurement done
 */
static void saadc_electrode_callback(nrf_drv_saadc_evt_t const * p_event)
{
    if (p_event->type == NRF_DRV_SAADC_EVT_DONE)                                                        //Capture offset calibration complete event
    {
        ret_code_t err_code;

        // convert sample
        err_code = nrf_drv_saadc_buffer_convert(p_event->data.done.p_buffer, SAADC_SAMPLES_IN_BUFFER);  //Set buffer so the SAADC can write to it again. This is either "buffer 1" or "buffer 2"
        APP_ERROR_CHECK(err_code);

        for (int i = 0; i < SAADC_SAMPLES_IN_BUFFER; i++)
        {
            // convert to float and stuff in array
            m_electrode_saadc_vals_temp[i][m_electrode_measurement_counter] = (vRef*p_event->data.done.p_buffer[i])/resCts;         
            clear_FPU_interrupts();
        }

        // increment sample
        m_electrode_measurement_counter++;

        // turn off SAADC
        m_saadc_initialized = false;
        nrf_drv_saadc_uninit();                                                                   //Unintialize SAADC to disable EasyDMA and save power
        NRF_SAADC->INTENCLR = (SAADC_INTENCLR_END_Clear << SAADC_INTENCLR_END_Pos);               //Disable the SAADC interrupt
        NVIC_ClearPendingIRQ(SAADC_IRQn);                                                         //Clear the SAADC interrupt if set

        //NRF_LOG_INFO("msmt ctr: %ld", m_electrode_measurement_counter);
        if (m_electrode_measurement_counter == NUM_MEASUREMENTS)
        {
            //Power on RTC instance
            nrf_drv_rtc_uninit(&rtc);                                         
            m_rtc_initialized = false;

            // reset measurement count
            m_electrode_measurement_counter = 0;

            // set flag to false, done for now
            m_electrode_measurement_in_progress = false;
            
            // log event
            m_adc_evt_counter++;
            NRF_LOG_INFO("ADC event number: %ld",m_adc_evt_counter);
            //NRF_LOG_INFO("ADC event number: %d, AIN0 = %d, AIN4 = %d, AIN6 = %d",(int)m_adc_evt_counter,m_electrode_saadc_vals_temp[0][0],m_electrode_saadc_vals_temp[1][0],m_electrode_saadc_vals_temp[2][0]);
            
            // execute saadc_finished task set during init
            clear_FPU_interrupts();
            saadc_task_finished_fn(&m_electrode_saadc_vals_temp[0][0], &m_electrode_saadc_vals_temp[1][0], &m_electrode_saadc_vals_temp[2][0], NUM_MEASUREMENTS);
        }  
    }
}

/** @brief Function to initialize the SAADC settings for the electrode channels
 */
static void vband_saadc_electrode_channels_init(void)
{
    ret_code_t err_code;
    nrf_drv_saadc_config_t saadc_config;
    nrf_saadc_channel_config_t channel_config0 =
        NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN0);
    nrf_saadc_channel_config_t channel_config1 =
        NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN4);
    nrf_saadc_channel_config_t channel_config2 =
        NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN6);
	
    //Configure SAADC
    saadc_config.resolution = NRF_SAADC_RESOLUTION_14BIT;                                 //Set SAADC resolution to 12-bit. This will make the SAADC output values from 0 (when input voltage is 0V) to 2^12=2048 (when input voltage is 3.6V for channel gain setting of 1/6).
    saadc_config.oversample = SAADC_OVERSAMPLE;                                           //Set oversample to 4x. This will make the SAADC output a single averaged value when the SAMPLE task is triggered 4 times.
    saadc_config.interrupt_priority = APP_IRQ_PRIORITY_LOW;                               //Set SAADC interrupt to low priority.
    saadc_config.low_power_mode = true;                                                   //Set SAADC interrupt to low priority.
	
    //Initialize SAADC
    APP_ERROR_CHECK(nrf_drv_saadc_init(&saadc_config, saadc_electrode_callback));                   //Initialize the SAADC with configuration and callback function. The application must then implement the saadc_callback function, which will be called when SAADC interrupt is triggered
	
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

    err_code = nrf_drv_saadc_buffer_convert(m_buffer_pool[0], SAADC_SAMPLES_IN_BUFFER);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_buffer_convert(m_buffer_pool[1], SAADC_SAMPLES_IN_BUFFER);
    APP_ERROR_CHECK(err_code);

    m_saadc_initialized = true; 
}

/** @brief Function callback for SAADC electrode measurement timer, will start the task for SAADC measurements
 */
static void rtc_handler(nrf_drv_rtc_int_type_t int_type)
{
    uint32_t err_code;
	
    if (int_type == NRF_DRV_RTC_INT_TICK) // sample SAADC event
    {
        // stop after predefined number of measurements
        if (m_electrode_measurement_in_progress == true)
        {   
            if (m_saadc_initialized == false)
            {
                vband_saadc_electrode_channels_init();                     //Initialize the SAADC. In the case when SAADC_SAMPLES_IN_BUFFER > 1 then we only need to initialize the SAADC when the the buffer is empty.
            }
            nrf_drv_saadc_sample();                                    //Trigger the SAADC SAMPLE task
        }
    }
}


/** @brief Function to initialize the SAADC electrode measurement timer
 */
static void vband_electrode_sample_rtc_config(void)
{
    uint32_t err_code;

    //Initialize RTC instance
    nrf_drv_rtc_config_t config = NRF_DRV_RTC_DEFAULT_CONFIG;
    config.prescaler = 32768/RTC_FREQUENCY; // 32768 / X = Tick Frequency
    err_code = nrf_drv_rtc_init(&rtc, &config, rtc_handler);
    APP_ERROR_CHECK(err_code);

    //Enable tick event & interrupt
    nrf_drv_rtc_tick_enable(&rtc,true);
    APP_ERROR_CHECK(err_code);

    //Power on RTC instance
    nrf_drv_rtc_enable(&rtc);                                         
    m_rtc_initialized = true;
}

/**@brief Assign callback to main file
 *
 */
void saadc_assign_callback_fn(saadc_finished_fnptr_t fn)
{
    saadc_task_finished_fn = fn;
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
        if (!m_rtc_initialized)
        {
            vband_electrode_sample_rtc_config();
        }
    }
}
/** @} */