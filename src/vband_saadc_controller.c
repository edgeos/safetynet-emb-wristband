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
//#include "nrf_drv_saadc.h"
#include "nrfx_saadc.h"
#include "nrfx_rtc.h"
#include "boards.h"
#include "app_error.h"
#include "app_util_platform.h"
#include "vband_saadc_controller.h"
#include "vband_common.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#if defined(BOARD_VWEDGE_V1)
  #define SAADC_SAMPLES_IN_BUFFER    5    // Number of SAADC samples in RAM before returning a SAADC event. For low power SAADC set this constant to 1. Otherwise the EasyDMA will be enabled for an extended time which consumes high current.
#elif defined(BOARD_VWEDGE_V2)
  #define SAADC_SAMPLES_IN_BUFFER    4    // Number of SAADC samples in RAM before returning a SAADC event. For low power SAADC set this constant to 1. Otherwise the EasyDMA will be enabled for an extended time which consumes high current.
#elif defined(BOARD_VWEDGE_WRIST)
  #define SAADC_SAMPLES_IN_BUFFER    5    // Number of SAADC samples in RAM before returning a SAADC event. For low power SAADC set this constant to 1. Otherwise the EasyDMA will be enabled for an extended time which consumes high current.
#else
  #define SAADC_SAMPLES_IN_BUFFER    4    // Number of SAADC samples in RAM before returning a SAADC event. For low power SAADC set this constant to 1. Otherwise the EasyDMA will be enabled for an extended time which consumes high current.
#endif
#define SAADC_VOLTAGE_SAMPLES      3
#define SAADC_OVERSAMPLE           NRF_SAADC_OVERSAMPLE_DISABLED  //Oversampling setting for the SAADC. Setting oversample to 4x This will make the SAADC output a single averaged value when the SAMPLE task is triggered 4 times. Enable BURST mode to make the SAADC sample 4 times when triggering SAMPLE task once.
#define SAADC_BURST_MODE           0    //Set to 1 to enable BURST mode, otherwise set to 0.

#define RTC_FREQUENCY              4096 // must be a factor of 32768. this is ADC sampling rate in Hz
//#define RTC_FREQUENCY              1024 // must be a factor of 32768. this is ADC sampling rate in Hz
#define NUM_MEASUREMENTS           512 // sets the data array size
//#define NUM_MEASUREMENTS           2048
#define SAADC_VOLTAGE_DECIMATION   4    // decimation factor. takes every 4th sample if value is 4
#define BATTERY_SAMPLING_TIME_OUT  512  // the number of times to skip sampling battery voltage and detecting 5V charging during ADC sampling

const  nrfx_rtc_t           rtc = NRFX_RTC_INSTANCE(2); // Declaring an instance of nrfx_rtc for RTC2. */

static nrf_saadc_value_t       m_buffer_pool[2][SAADC_SAMPLES_IN_BUFFER]; // set up two buffers for double buffering
static uint32_t                m_adc_evt_counter = 0;
static bool                    m_saadc_initialized = false;
static bool                    m_rtc_initialized = false;
static bool                    m_rtc_enabled = false;
static float                   m_electrode_saadc_vals[SAADC_VOLTAGE_SAMPLES][NUM_MEASUREMENTS*2];
static float                   m_electrode_saadc_vals_temp[SAADC_VOLTAGE_SAMPLES][NUM_MEASUREMENTS*2];
static uint16_t                m_electrode_measurement_counter = 0;
static bool                    m_electrode_measurement_in_progress = false;
static uint16_t                m_saadc_ind_st = 0;
static uint16_t                m_battery_sample_time_out = 0;
static uint16_t                m_voltage_sample_decimation_counter = 0;
static float                   vRef = 3.6;
static float                   resCts = 16383; // 8-bit = 255, 10-bit = 1023, 12-bit = 4095, 14-bit = 16383

static saadc_finished_fnptr_t saadc_task_finished_fn = NULL;
static float battery_voltage = 0;
static float usb_voltage = 0;

/** @brief Function callback for SAADC electrode measurement done
 */
static void saadc_electrode_callback(nrfx_saadc_evt_t const * p_event)
{
    static uint16_t fill_index = 0;
    if (p_event->type == NRFX_SAADC_EVT_DONE)                                                        //Capture offset calibration complete event
    {
        ret_code_t err_code;

        // convert sample
        if(m_battery_sample_time_out == 0)
        {
          err_code = nrfx_saadc_buffer_convert(p_event->data.done.p_buffer, SAADC_SAMPLES_IN_BUFFER);  //Set buffer so the SAADC can write to it again. This is either "buffer 1" or "buffer 2
        }
        else
        {
          err_code = nrfx_saadc_buffer_convert(p_event->data.done.p_buffer, SAADC_VOLTAGE_SAMPLES);  //Set buffer so the SAADC can write to it again. This is either "buffer 1" or "buffer 2
        }
        APP_ERROR_CHECK(err_code);

        // apply decimation of 4kHz sampling down to 1kHz effective
        if(m_voltage_sample_decimation_counter == 0)
        {

          // the first 3 channels are the waves for detecting AC voltage
          for (int i = 0; i < SAADC_VOLTAGE_SAMPLES; i++)
          {
              // convert to float and stuff in array
              m_electrode_saadc_vals[i][fill_index] = (p_event->data.done.p_buffer[i])/resCts;
              clear_FPU_interrupts();
          }
          if(m_battery_sample_time_out == 0)
          {
            // the 4th channel is the battery voltage
            if(battery_voltage == 0)
            {
              // first battery voltage data does not count towards running average
              battery_voltage = (3.6f*p_event->data.done.p_buffer[3])/resCts;
            }
            else
            {
              // compute running average to smooth out battery voltage readings
              battery_voltage = battery_voltage*0.9f + 0.1f*(3.6f*p_event->data.done.p_buffer[3])/resCts;
            }
  #if defined(BOARD_VWEDGE_V1)
            // the 5th channel is the 5V detect
            if(usb_voltage == 0)
            {
              // first battery voltage data does not count towards running average
              usb_voltage = (3.6f*p_event->data.done.p_buffer[4])/resCts;
            }
            else
            {
              usb_voltage = usb_voltage*0.1f + 0.9f*(3.6f*p_event->data.done.p_buffer[4])/resCts;
              //usb_voltage = usb_voltage*0.9f + 0.1f*(3.6f*p_event->data.done.p_buffer[4])/resCts;
            }
  #endif
            m_battery_sample_time_out = BATTERY_SAMPLING_TIME_OUT;
          }
          else
          {
            m_battery_sample_time_out--;
          }

          // increment sample num and index
          if((fill_index >= NUM_MEASUREMENTS) && (fill_index < (NUM_MEASUREMENTS*2-1)))
          {
              m_saadc_ind_st = fill_index - NUM_MEASUREMENTS + 1;
          }

          // handle shifting to prevent overflow
          if(fill_index == (NUM_MEASUREMENTS*2-1))
          {
              // copy whole array to temporary array
              //memcpy(&m_electrode_saadc_vals_temp[0][0], &m_electrode_saadc_vals[0][0], sizeof(m_electrode_saadc_vals));
              for (int y = 0; y < SAADC_VOLTAGE_SAMPLES; y++)
              {
                  for (int z = 0; z < NUM_MEASUREMENTS; z++)
                  {
                    // copy from second half of array to first half of data buffer
                    m_electrode_saadc_vals[y][z] = m_electrode_saadc_vals[y][z+NUM_MEASUREMENTS];
                  }
                  //memcpy(&m_electrode_saadc_vals[y][0], &m_electrode_saadc_vals_temp[y][NUM_MEASUREMENTS], (sizeof(float) * NUM_MEASUREMENTS));
              }
              fill_index = NUM_MEASUREMENTS;
              m_saadc_ind_st = 0;
          }
          else
          {
              fill_index += 1;
          }

          m_electrode_measurement_counter = (m_electrode_measurement_counter ==  NUM_MEASUREMENTS) 
                                            ? NUM_MEASUREMENTS : (m_electrode_measurement_counter + 1);

          m_voltage_sample_decimation_counter = SAADC_VOLTAGE_DECIMATION-1;
        }
        else
        {
          m_voltage_sample_decimation_counter--;
        }

        // turn off SAADC
        m_saadc_initialized = false;
        nrfx_saadc_uninit();                                                                   //Unintialize SAADC to disable EasyDMA and save power

        //NRF_SAADC->INTENCLR = (SAADC_INTENCLR_END_Clear << SAADC_INTENCLR_END_Pos);               //Disable the SAADC interrupt
        err_code = sd_nvic_DisableIRQ(SAADC_IRQn); // use the softdevice version of nvic function
        APP_ERROR_CHECK(err_code);
        //NVIC_ClearPendingIRQ(SAADC_IRQn);                                                         //Clear the SAADC interrupt if set
        err_code = sd_nvic_ClearPendingIRQ(SAADC_IRQn); // use the softdevice version of nvic function
        APP_ERROR_CHECK(err_code);

        // log event
        m_adc_evt_counter++;
        //NRF_LOG_INFO("finished ADC sampling %d", m_adc_evt_counter);
        //NRF_LOG_INFO("ADC event number: %ld",m_adc_evt_counter);
        //saadc_task_finished_fn(&m_electrode_saadc_vals[0][0], &m_electrode_saadc_vals[1][0], &m_electrode_saadc_vals[2][0], NUM_MEASUREMENTS);  
    }
}

/** @brief Function to initialize the SAADC settings for the electrode channels
 */
static void vband_saadc_electrode_channels_init(void)
{
    ret_code_t err_code;
    nrfx_saadc_config_t saadc_config;

#if defined(BOARD_VWEDGE_V1)
    nrf_saadc_channel_config_t channel_config0 =
        NRFX_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN1);
    nrf_saadc_channel_config_t channel_config1 =
        NRFX_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN2);
    nrf_saadc_channel_config_t channel_config2 =
        NRFX_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN3);
    // battery VDD, measuring half VDD on pin 0.28 AIN4
    //nrf_saadc_channel_config_t channel_config3 =
//        NRFX_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN4);

    // using VDDH/5 as input for battery monitoring, need to modify sdk to allow for this selection, otherwise we get ASSERT error
    nrf_saadc_channel_config_t channel_config3 =
        NRFX_SAADC_DEFAULT_CHANNEL_CONFIG_SE(SAADC_CH_PSELP_PSELP_VDDHDIV5);

    // detect presence of 5V charging on pin 0.02 AIN0
    nrf_saadc_channel_config_t channel_config4 =
        NRFX_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN0);
#elif defined(BOARD_VWEDGE_V2)
    nrf_saadc_channel_config_t channel_config0 =
        NRFX_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN1);
    nrf_saadc_channel_config_t channel_config1 =
        NRFX_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN2);
    nrf_saadc_channel_config_t channel_config2 =
        NRFX_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN3);
    // using VDDH/5 as input for battery monitoring, need to modify sdk to allow for this selection, otherwise we get ASSERT error
    nrf_saadc_channel_config_t channel_config3 =
        NRFX_SAADC_DEFAULT_CHANNEL_CONFIG_SE(SAADC_CH_PSELP_PSELP_VDDHDIV5);

    // detect presence of 5V charging on pin 0.02 AIN0
//    nrf_saadc_channel_config_t channel_config4 =
//        NRFX_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN0);
#elif defined(BOARD_VWEDGE_WRIST)
    nrf_saadc_channel_config_t channel_config0 =
        NRFX_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN1);
    nrf_saadc_channel_config_t channel_config1 =
        NRFX_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN2);
    nrf_saadc_channel_config_t channel_config2 =
        NRFX_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN3);

    // using VDDH/5 as input for battery monitoring, need to modify sdk to allow for this selection, otherwise we get ASSERT error
    nrf_saadc_channel_config_t channel_config3 =
        NRFX_SAADC_DEFAULT_CHANNEL_CONFIG_SE(SAADC_CH_PSELP_PSELP_VDDHDIV5);

    // detect presence of 5V charging on pin 0.02 AIN0
    nrf_saadc_channel_config_t channel_config4 =
        NRFX_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN0);
#else
    nrf_saadc_channel_config_t channel_config0 =
        NRFX_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN0);
    nrf_saadc_channel_config_t channel_config1 =
        NRFX_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN4);
    nrf_saadc_channel_config_t channel_config2 =
        NRFX_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN6);
    // battery VDD, can use defaults
    nrf_saadc_channel_config_t channel_config3 =
        NRFX_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_VDD);
#endif

    channel_config0.gain =  channel_config1.gain =  channel_config2.gain = NRF_SAADC_GAIN1_4; // set gain to 1/4
    channel_config0.reference = channel_config1.reference = channel_config2.reference = NRF_SAADC_REFERENCE_VDD4; // set reference to VDD/4
    // ADC range is 0 to VDD for voltage sensor channels 0,1,2
    // ADC range is 0 to 3.6V using internal reference and 1/6 gain for channels 3,4

    //Configure SAADC
    saadc_config.resolution = NRF_SAADC_RESOLUTION_14BIT;                                 //Set SAADC resolution to 14-bit. This will make the SAADC output values from 0 (when input voltage is 0V) to 2^14=4096 (when input voltage is VDD 2.7V).
    saadc_config.oversample = SAADC_OVERSAMPLE;                                           //oversample set at the top of this file
    saadc_config.interrupt_priority = APP_IRQ_PRIORITY_LOW;                               //Set SAADC interrupt to low priority.
    saadc_config.low_power_mode = true;                                                   //Set SAADC low power mode.
	
    //Initialize SAADC
    APP_ERROR_CHECK(nrfx_saadc_init(&saadc_config, saadc_electrode_callback));                   //Initialize the SAADC with configuration and callback function. The application must then implement the saadc_callback function, which will be called when SAADC interrupt is triggered
	
    //Initialize SAADC channel
    err_code = nrfx_saadc_channel_init(0, &channel_config0);                            //Initialize SAADC channel 0 with the channel configuration
    APP_ERROR_CHECK(err_code);
    err_code = nrfx_saadc_channel_init(1, &channel_config1);                            //Initialize SAADC channel 1 with the channel configuration
    APP_ERROR_CHECK(err_code);
    err_code = nrfx_saadc_channel_init(2, &channel_config2);                            //Initialize SAADC channel 2 with the channel configuration
    APP_ERROR_CHECK(err_code);
    if(m_battery_sample_time_out == 0)
    {
      err_code = nrfx_saadc_channel_init(3, &channel_config3);                            //Initialize SAADC channel 3 with the channel configuration
      APP_ERROR_CHECK(err_code);
#if defined(BOARD_VWEDGE_V1) || defined(BOARD_VWEDGE_WRIST)
      err_code = nrfx_saadc_channel_init(4, &channel_config4);                            //Initialize SAADC channel 4 with the channel configuration
      APP_ERROR_CHECK(err_code);
#endif
    }

    if(SAADC_BURST_MODE)
    {
        NRF_SAADC->CH[0].CONFIG |= 0x01000000;                                            //Configure burst mode for channel 0. Burst is useful together with oversampling. When triggering the SAMPLE task in burst mode, the SAADC will sample "Oversample" number of times as fast as it can and then output a single averaged value to the RAM buffer. If burst mode is not enabled, the SAMPLE task needs to be triggered "Oversample" number of times to output a single averaged value to the RAM buffer.
        NRF_SAADC->CH[1].CONFIG |= 0x01000000; 
        NRF_SAADC->CH[2].CONFIG |= 0x01000000; 
        if(m_battery_sample_time_out == 0)
        {
          NRF_SAADC->CH[3].CONFIG |= 0x01000000;
        }
    }
    if(m_battery_sample_time_out == 0) // do voltage sensor plus battery and usb voltages
    {
      err_code = nrfx_saadc_buffer_convert(m_buffer_pool[0], SAADC_SAMPLES_IN_BUFFER);
      APP_ERROR_CHECK(err_code);

      err_code = nrfx_saadc_buffer_convert(m_buffer_pool[1], SAADC_SAMPLES_IN_BUFFER);
      APP_ERROR_CHECK(err_code);
    }
    else // only do voltage sensor channels (3 samples)
    {
      err_code = nrfx_saadc_buffer_convert(m_buffer_pool[0], SAADC_VOLTAGE_SAMPLES);
      APP_ERROR_CHECK(err_code);

      err_code = nrfx_saadc_buffer_convert(m_buffer_pool[1], SAADC_VOLTAGE_SAMPLES);
      APP_ERROR_CHECK(err_code);
    }

    m_saadc_initialized = true;
    //NRF_LOG_INFO("ADC electrode_channels_init done");
}

/** @brief Function callback for SAADC electrode measurement timer, will start the task for SAADC measurements
 */
static void rtc_handler(nrfx_rtc_int_type_t int_type)
{
    uint32_t err_code;
	
    if (int_type == NRFX_RTC_INT_TICK) // sample SAADC event
    {
        if (m_saadc_initialized == false)
        {
            //NRF_LOG_INFO("rtc_handler going to saadc_electrode_channels_init");
            vband_saadc_electrode_channels_init();                     //Initialize the SAADC. In the case when SAADC_SAMPLES_IN_BUFFER > 1 then we only need to initialize the SAADC when the the buffer is empty.
        }
        //NRF_LOG_INFO("rtc_handler going to trigger saadc sample");
        nrfx_saadc_sample();                                    //Trigger the SAADC SAMPLE task
    }
}


/** @brief Function to initialize the SAADC electrode measurement timer
 */
static void vband_electrode_sample_rtc_config(void)
{
    uint32_t err_code;

    //Initialize RTC instance
    nrfx_rtc_config_t config = NRFX_RTC_DEFAULT_CONFIG;
    config.prescaler = (32768/RTC_FREQUENCY)-1; // (32768 / X) - 1 = Tick Frequency
    err_code = nrfx_rtc_init(&rtc, &config, rtc_handler);
    APP_ERROR_CHECK(err_code);

    //Enable tick event & interrupt
    nrfx_rtc_tick_enable(&rtc,true);
    APP_ERROR_CHECK(err_code);

    //Power on RTC instance
    nrfx_rtc_enable(&rtc);
    m_rtc_initialized = true;
    m_rtc_enabled = true;
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
    // start the RTC that controls the sampling intervals for the electrode channels
    if (!m_rtc_initialized)
    {
        vband_electrode_sample_rtc_config();
    }
    if (!m_rtc_enabled)
    {
        nrfx_rtc_enable(&rtc);
    }
    // SAADC always runs and continuously fills the buffer, just run the algorithm
    if (m_electrode_measurement_counter >= NUM_MEASUREMENTS)
    {
        //NRF_LOG_INFO("saadc_sample_electrode_channels, got enough samples");
        saadc_task_finished_fn(&m_electrode_saadc_vals[0][m_saadc_ind_st], &m_electrode_saadc_vals[1][m_saadc_ind_st], &m_electrode_saadc_vals[2][m_saadc_ind_st], NUM_MEASUREMENTS);
    }
}

/** @brief returns battery voltage (float) in a pointer reference
 */
void vband_saadc_read_battery_voltage(float *battery_volt)
{
    *battery_volt = battery_voltage;
}

/** @brief returns usb voltage (float) in a pointer reference
 */
void vband_saadc_read_usb_voltage(float *usb_volt)
{
    *usb_volt = usb_voltage;
}

/** @brief turns off rtc to stop sampling
 */
void vband_saadc_disable_rtc(void)
{
    nrfx_rtc_disable(&rtc);
}





/** @} */