/**
 * Copyright (c) 2014 - 2018, Nordic Semiconductor ASA
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
// Board/nrf6310/ble/ble_app_hrs_rtx/main.c
/**
 *
 * @brief Heart Rate Service Sample Application with RTX main file.
 *
 * This file contains the source code for a sample application using RTX and the
 * Heart Rate service (and also Battery and Device Information services).
 * This application uses the @ref srvlib_conn_params module.
 */

#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_bas.h"
#include "ble_dis.h"
#include "ble_conn_params.h"
#include "sensorsim.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "nrf_sdh_freertos.h"
#include "app_timer.h"
#include "peer_manager.h"
#include "peer_manager_handler.h"
#include "bsp_btn_ble.h"
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"
#include "fds.h"
#include "ble_conn_state.h"
#include "nrf_drv_clock.h"
#include "nrf_drv_rtc.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"
#include "nrf_delay.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

// custom BLE service
#include "ble_vband_srv.h"

// internal peripheral contollers
#include "vband_pwm_controller.h"
#include "vband_saadc_controller.h"
#include "vband_ext_sensor_controller.h"
#include "voltage_alarm_algorithm.h"
#include "vband_flash_controller.h"
#include "vband_common.h"

// external sensors
#include "ccs811.h"
#include "bme280.h"
#include "ADXL362.h"
#include "MAX30105.h"

#define SAADC_WAKEUP_SAMPLE_INTERVAL_DISCONNECTED              100                                    /**< SAADC measurement interval (ms). */
#define SAADC_WAKEUP_SAMPLE_INTERVAL_CONNECTED                 100                                     /**< SAADC measurement interval (ms). */

// defines for ext sensors, the first SAMPLE_INTERVAL defines how often we wake up, the
// MEASURE_INTERVALs should be a multiple of the SAMPLE_INTERVAL
#define EXT_SENSORS_WAKEUP_SAMPLE_INTERVAL          100                                      /**< Ext. Sensor measurement interval (ms). */
#define EXT_SENSORS_WAKEUP_SAMPLE_INTERVAL_TICKS    APP_TIMER_TICKS(100)
#define HEART_BEAT_CHECK_INTERVAL                   5000                                    /**< interval (ms) between checking on BLE connection heart beat. */
#define CCS811_MEASURE_INTERVAL                     60000                                    /**< CCS811 measurement interval (ms). */
#define BME280_MEASURE_INTERVAL                     2000                                    /**< BME280 measurement interval (ms). */
#define MAX30105_MEASURE_PROXIMITY_INTERVAL         1000                                     /**< MAX30105 proximity measurement interval (ms). */
#define MAX30105_MEASURE_HEART_RATE_INTERVAL        15                                      /**< MAX30105 proximity measurement interval (ms). */
#define ADXL362_MEASURE_INTERVAL                    75                                      /**< ADXL362 measurement interval (ms). */
#define ADXL362_INACTIVITY_WHILE_CONNECTED_TIMEOUT  600                                     /**< ADXL362 sleep interval while connected (seconds). */
#define USB_DETECT_TIMEOUT_VALUE                    2                                      /**< the number of consecutive times the USB detect must be the opposite state to flip the state. decrement every battery level check (2 sec) */

#define DEVICE_NAME                         "GER WedgeV2"                        /**< Name of device. Will be included in the advertising data. */
#define MANUFACTURER_NAME                   "GE"                                    /**< Manufacturer. Will be passed to Device Information Service. */
#define VBAND_SERVICE_UUID_TYPE             BLE_UUID_TYPE_VENDOR_BEGIN              /**< UUID type for the Voltage Band Service (vendor specific). */

#define APP_BLE_OBSERVER_PRIO               3                                       /**< Application's BLE observer priority. You shouldn't need to modify this value. */
#define APP_BLE_CONN_CFG_TAG                1                                       /**< A tag identifying the SoftDevice BLE configuration. */

#define APP_ADV_INTERVAL                    160                                     /**< The advertising interval (in units of 0.625 ms. This value corresponds to 1000 ms). */
#ifdef BOARD_VBAND_V1
  #define APP_ADV_DURATION                  3000                                    /**< The advertising duration (30 seconds) in units of 10 milliseconds. */
#elif BOARD_VWEDGE_V2
  #define APP_ADV_DURATION                  1500                                    /**< The advertising duration (30 seconds) in units of 10 milliseconds. */
#else
  #define APP_ADV_DURATION                  3000                                    /**< The advertising duration (30 seconds) in units of 10 milliseconds. */
#endif

#define BATTERY_LEVEL_MEAS_INTERVAL         2000                                    /**< Battery level measurement interval (ms). */
#define MIN_BATTERY_LEVEL                   81                                      /**< Minimum simulated battery level. */
#define MAX_BATTERY_LEVEL                   100                                     /**< Maximum simulated battery level. */
#define BATTERY_LEVEL_INCREMENT             1                                       /**< Increment between each simulated battery level measurement. */

//#define MIN_CONN_INTERVAL                   MSEC_TO_UNITS(70, UNIT_1_25_MS)        /**< Minimum acceptable connection interval (0.4 seconds). */
//#define MAX_CONN_INTERVAL                   MSEC_TO_UNITS(140, UNIT_1_25_MS)        /**< Maximum acceptable connection interval (0.65 second). */
#define MIN_CONN_INTERVAL                   MSEC_TO_UNITS(8, UNIT_1_25_MS)        /**< Minimum acceptable connection interval (0.008 seconds). */
#define MAX_CONN_INTERVAL                   MSEC_TO_UNITS(20, UNIT_1_25_MS)        /**< Maximum acceptable connection interval (0.020 second).  We're sending a lot of updates*/
#define SLAVE_LATENCY                       0                                       /**< Slave latency. */
#define CONN_SUP_TIMEOUT                    MSEC_TO_UNITS(4000, UNIT_10_MS)         /**< Connection supervisory time-out (4 seconds). */

#define FIRST_CONN_PARAMS_UPDATE_DELAY      5000                                    /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY       30000                                   /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT        3                                       /**< Number of attempts before giving up the connection parameter negotiation. */

#define SEC_PARAM_BOND                      0                                       /**< Perform bonding. */
#define SEC_PARAM_MITM                      0                                       /**< Man In The Middle protection not required. */
#define SEC_PARAM_LESC                      0                                       /**< LE Secure Connections not enabled. */
#define SEC_PARAM_KEYPRESS                  0                                       /**< Keypress notifications not enabled. */
#define SEC_PARAM_IO_CAPABILITIES           BLE_GAP_IO_CAPS_NONE                    /**< No I/O capabilities. */
#define SEC_PARAM_OOB                       0                                       /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE              7                                       /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE              16                                      /**< Maximum encryption key size. */

#define DEAD_BEEF                           0xDEADBEEF                              /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define OSTIMER_WAIT_FOR_QUEUE              2                                       /**< Number of ticks to wait for the timer queue to be ready */

BLE_VBAND_SRV_DEF(m_vband);                                         /**< Voltage Band service instance. */
BLE_BAS_DEF(m_bas);                                                 /**< Battery service instance. */
NRF_BLE_GATT_DEF(m_gatt);                                           /**< GATT module instance. */
NRF_BLE_QWR_DEF(m_qwr);                                             /**< Context for the Queued Write module.*/
BLE_ADVERTISING_DEF(m_advertising);                                 /**< Advertising module instance. */

// set VDD voltage REGOUT0 in UICR
const uint32_t UICR_ADDR_REGOUT0 __attribute__((section(".regout0"))) __attribute__((used))  = 0xFFFFFFF3; // 2.7V
//const uint32_t UICR_ADDR_REGOUT0 __attribute__((section(".regout0"))) __attribute__((used))  = 0xFFFFFFF4; // 3.0V
//const uint32_t UICR_ADDR_REGOUT0 __attribute__((section(".regout0"))) __attribute__((used))  = 0xFFFFFFF5; // 3.3V
 // use P0.18 as reset
const uint32_t UICR_ADDR_PSELRESET0 __attribute__((section(".pselreset0"))) __attribute__((used))  = 0x00000012;
const uint32_t UICR_ADDR_PSELRESET1 __attribute__((section(".pselreset1"))) __attribute__((used))  = 0x00000012;

static uint16_t m_conn_handle              = BLE_CONN_HANDLE_INVALID;                 /**< Handle of the current connection. */
static uint16_t m_ble_vband_max_data_len   = BLE_GATT_ATT_MTU_DEFAULT - 3;            /**< Maximum length of data (in bytes) that can be transmitted to the peer by the Nordic UART service module. */

static sensorsim_cfg_t   m_battery_sim_cfg;                         /**< Battery Level sensor simulator configuration. */
static sensorsim_state_t m_battery_sim_state;                       /**< Battery Level sensor simulator state. */

static bool m_ble_connected_bool = false;
static bool m_ble_was_connected_bool = false;
static bool m_usb_detected = true;

static char m_ble_advertising_name[MAX_BLE_NAME_LENGTH] = {0}; // defined in vband_flash_controller.h
static ble_vband_srv_config_mode_t m_current_vband_mode = BLE_VBAND_SRV_MODE_ENGINEERING;//BLE_VBAND_SRV_MODE_NORMAL;
static uint32_t m_ble_heart_beat = 1; // 32 bit number for keep alive heart beat from tablet app
static uint32_t m_ble_heart_beat_last = 2; // last heart beat value

static uint16_t m_buzzer_block_timeout = 0; // a timeout mechanism to prevent buzzer state being updated for some set time. used in the voltage alarm function, happening every 100ms

static ble_uuid_t m_adv_uuids[] =                                   /**< Universally unique service identifiers. */
{
    {BLE_UUID_BATTERY_SERVICE, BLE_UUID_TYPE_BLE},
    {BLE_UUID_DEVICE_INFORMATION_SERVICE, BLE_UUID_TYPE_BLE},
    {BLE_UUID_VOLTAGE_WRISTBAND_SERVICE, BLE_UUID_TYPE_BLE}
};

// FreeRTOS Timers
static TimerHandle_t m_battery_timer;                               /**< Definition of battery timer. */
static TimerHandle_t m_saadc_timer;                                 /**< Definition of SAADC timer. */

// FreeRTOS Tasks & semaphore (to prevent i2c contention)
static TaskHandle_t saadc_sample_timer_handle;
static TaskHandle_t adxl362_measure_task_handle; 
static TaskHandle_t ccs811_measure_task_handle; 
static TaskHandle_t bme280_measure_task_handle; 
static TaskHandle_t max30105_measure_task_handle; 
static TaskHandle_t m_heart_beat_thread;
xSemaphoreHandle i2c_semaphore = 0;

#if NRF_LOG_ENABLED
static TaskHandle_t m_logger_thread;                                /**< Definition of Logger thread. */
#endif

static void advertising_start(void * p_erase_bonds);
static void update_advertising(void);
void update_vband_config_characteristic(void);

bool        is_new_alarm_threshold = false;
uint32_t    new_alarm_threshold = 0;

/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in]   line_num   Line number of the failing ASSERT call.
 * @param[in]   file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}


/**@brief Function for handling Peer Manager events.
 *
 * @param[in] p_evt  Peer Manager event.
 */
static void pm_evt_handler(pm_evt_t const * p_evt)
{
    bool delete_bonds = false;

    pm_handler_on_pm_evt(p_evt);
    pm_handler_flash_clean(p_evt);

    switch (p_evt->evt_id)
    {
        case PM_EVT_PEERS_DELETE_SUCCEEDED:
            advertising_start(&delete_bonds);
            break;

        default:
            break;
    }
}


/**@brief Function for performing battery measurement and updating the Battery Level characteristic
 *        in Battery Service.
 */
static void battery_level_update(void)
{
    ret_code_t err_code;
    static uint8_t battery_level;
    static uint8_t usb_detect_timeout = USB_DETECT_TIMEOUT_VALUE;
    float battery_voltage;
    float usb_voltage;

#ifdef BOARD_VBAND_V1
    vband_saadc_read_battery_voltage(&battery_voltage);
    if (battery_voltage >= 2.65f)
    {
        battery_level = 100-(2.7f-battery_voltage)/30;
    }
    else if (battery_voltage >= 2.60f)
    {
        battery_level = 70-(2.65f-battery_voltage)/20;
    }
    else if (battery_voltage >= 2.55f)
    {
        battery_level = 50-(2.60f-battery_voltage)/40;
    }
    else
    {
        battery_level = 10-(2.55f-battery_voltage)/10;
    }
#elif defined(BOARD_VWEDGE_V1) || defined(BOARD_VWEDGE_V2)
    vband_saadc_read_battery_voltage(&battery_voltage);
#if NRF_LOG_ENABLED
    //NRF_LOG_INFO("battery_voltage " NRF_LOG_FLOAT_MARKER, NRF_LOG_FLOAT(battery_voltage));
    //NRF_LOG_INFO("battery_level_update %0.2f", battery_voltage);
#endif
    if (battery_voltage >= 0.83f)
    {
        battery_level = 100;
    }
    else if (battery_voltage >= 0.54f)
    {
        battery_level = 100-(0.83f-battery_voltage)*310;
    }
    else
    {
        battery_level = 10-(0.54f-battery_voltage)*230;
    }
    //NRF_LOG_INFO("battery_level=%d", battery_level);
 #if defined(BOARD_VWEDGE_V2)
    if(m_usb_detected)
    {
      if (nrf_gpio_pin_read(USB_DETECT_PIN))
      {
        usb_detect_timeout = USB_DETECT_TIMEOUT_VALUE;
      }
      else
      {
        //NRF_LOG_INFO("USB detect Low, timeout %d", usb_detect_timeout);
        usb_detect_timeout--;
      }
      if(usb_detect_timeout == 0)
      {
        m_usb_detected = false;
        //NRF_LOG_INFO("Charging stopped: " NRF_LOG_FLOAT_MARKER, NRF_LOG_FLOAT(usb_voltage));
        NRF_LOG_INFO("Charging stopped");
        if(m_ble_connected_bool) update_vband_config_characteristic();
      }
    }
    else
    {
      if (nrf_gpio_pin_read(USB_DETECT_PIN))
      {
        usb_detect_timeout--;
      }
      else
      {
        usb_detect_timeout = USB_DETECT_TIMEOUT_VALUE;
      }
      if(usb_detect_timeout == 0)
      {
        m_usb_detected = true;
        //NRF_LOG_INFO("Charging started: " NRF_LOG_FLOAT_MARKER, NRF_LOG_FLOAT(usb_voltage));
        NRF_LOG_INFO("Charging started");
        if(m_ble_connected_bool) update_vband_config_characteristic();
      }
    }
 #else
    vband_saadc_read_usb_voltage(&usb_voltage);
    if(m_usb_detected)
    {
      if (usb_voltage < 1.1f)
      {
        usb_detect_timeout--;
      }
      else
      {
        usb_detect_timeout = USB_DETECT_TIMEOUT_VALUE;
      }
      if(usb_detect_timeout == 0)
      {
        m_usb_detected = false;
        NRF_LOG_INFO("Charging stopped: " NRF_LOG_FLOAT_MARKER, NRF_LOG_FLOAT(usb_voltage));
        if(m_ble_connected_bool) update_vband_config_characteristic();
      }
    }
    else
    {
      //NRF_LOG_INFO("usb_voltage " NRF_LOG_FLOAT_MARKER, NRF_LOG_FLOAT(usb_voltage));
      if (usb_voltage >= 1.1f)
      {
        usb_detect_timeout--;
      }
      else
      {
        usb_detect_timeout = USB_DETECT_TIMEOUT_VALUE;
      }
      if(usb_detect_timeout == 0)
      {
        m_usb_detected = true;
        NRF_LOG_INFO("Charging started: " NRF_LOG_FLOAT_MARKER, NRF_LOG_FLOAT(usb_voltage));
        if(m_ble_connected_bool) update_vband_config_characteristic();
      }
    }
 #endif
#else
    battery_level = (uint8_t)sensorsim_measure(&m_battery_sim_state, &m_battery_sim_cfg);
#endif
    
    err_code = ble_bas_battery_level_update(&m_bas, battery_level, BLE_CONN_HANDLE_ALL);
    if ((err_code != NRF_SUCCESS) &&
        (err_code != NRF_ERROR_INVALID_STATE) &&
        (err_code != NRF_ERROR_RESOURCES) &&
        (err_code != NRF_ERROR_BUSY) &&
        (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
       )
    {
        APP_ERROR_HANDLER(err_code);
    }
}


/**@brief Function for sending Voltage Band data over BLE
 */
static void vband_characteristic_update(ble_vband_char_update_t char_update, uint8_t * p_data, uint16_t * p_length)
{
    ret_code_t err_code;
	static uint32_t nFail = 0;

    if (m_current_vband_mode == BLE_VBAND_SRV_MODE_NORMAL)
    {
        if (char_update == ble_vband_srv_alarm_update)
        {
            *p_length = 4; // don't send waveforms during normal mode
        }
        else if (char_update == ble_vband_srv_accelerometer_update)
        {
            return; // don't send accelerometer data during normal mode
        }
    }

    if(m_conn_handle != BLE_CONN_HANDLE_INVALID)
    {
		err_code = char_update(&m_vband, p_data, p_length, m_conn_handle);

		if(err_code != NRF_SUCCESS) {
		  NRF_LOG_INFO("char update failed error: %d", err_code);
		  if ((err_code != NRF_SUCCESS) &&
			  (err_code != NRF_ERROR_INVALID_STATE) &&
			  (err_code != NRF_ERROR_RESOURCES) &&
			  (err_code != NRF_ERROR_BUSY) &&
			  (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
			)
		  {
			  APP_ERROR_HANDLER(err_code);
		  } else if(err_code == NRF_ERROR_RESOURCES) {
			if(++nFail > 100) {
				// forcibly disconnect
				nFail = 0;
				sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
			}
		  }
		} else { nFail = 0; } //NRF_LOG_INFO("update char %p %u", p_data, *p_length); }
    } 
}


/**@brief Function for handling the Battery measurement timer time-out.
 *
 * @details This function will be called each time the battery level measurement timer expires.
 *
 * @param[in] xTimer Handler to the timer that called this function.
 *                   You may get identifier given to the function xTimerCreate using pvTimerGetTimerID.
 */
static void battery_level_meas_timeout_handler(TimerHandle_t xTimer)
{
    UNUSED_PARAMETER(xTimer);
    battery_level_update();
}

/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */
static void timers_init(void)
{
    // Initialize timer module.
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);

    // Create timers.
    m_battery_timer = xTimerCreate("BATT",
                                   BATTERY_LEVEL_MEAS_INTERVAL,
                                   pdTRUE,
                                   NULL,
                                   battery_level_meas_timeout_handler);

    /* Error checking */
    if ( (NULL == m_battery_timer) )
    {
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }
}


/**@brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */
static void gap_params_init(void)
{
    ret_code_t              err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)&m_ble_advertising_name,
                                          sizeof(m_ble_advertising_name));
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;
//	gap_conn_params.min_conn_interval = 6; //MIN_CONN_INTERVAL;
//	gap_conn_params.max_conn_interval = 16; //MAX_CONN_INTERVAL;
//	gap_conn_params.slave_latency     = SLAVE_LATENCY;
//	gap_conn_params.conn_sup_timeout  = 400;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling events from the GATT library. */
void gatt_evt_handler(nrf_ble_gatt_t * p_gatt, nrf_ble_gatt_evt_t const * p_evt)
{
    if ((m_conn_handle == p_evt->conn_handle) && (p_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED))
    {
        m_ble_vband_max_data_len = p_evt->params.att_mtu_effective - 3;
#if NRF_LOG_ENABLED
        NRF_LOG_INFO("Data len is set to 0x%X(%d)", m_ble_vband_max_data_len, m_ble_vband_max_data_len);
#endif
    }
    NRF_LOG_DEBUG("ATT MTU exchange completed. central 0x%x peripheral 0x%x",
                  p_gatt->att_mtu_desired_central,
                  p_gatt->att_mtu_desired_periph);
}


/**@brief Function for initializing the GATT module. */
static void gatt_init(void)
{
    ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, gatt_evt_handler);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_ble_gatt_att_mtu_periph_set(&m_gatt, NRF_SDH_BLE_GATT_MAX_MTU_SIZE);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling Queued Write Module errors.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void nrf_qwr_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}

/**@brief Function for handling the data from the Voltage Band Service.
 *
 * @details This function will process the data received from the Voltage Band BLE Service
 *
 * @param[in] p_evt       Voltage Band Service event.
 */
/**@snippet [Handling the data received over BLE] */
static void vband_data_handler(ble_vband_srv_evt_t * p_evt)
{
    ret_code_t err_code;
    
    uint8_t buf[BLE_VBAND_CONFIG_DATA_LEN];
    if (p_evt->type == BLE_VBAND_SRV_EVT_CONFIG_UPDATED)
    {
        uint32_t err_code;

        NRF_LOG_DEBUG("Received config update from BLE Voltage Band Service.");
        
        // working buffer copy, parse
        memcpy(&buf[0], p_evt->params.config_data.config_data_buffer, BLE_VBAND_CONFIG_DATA_LEN);
        switch(buf[0])
        {
            case BLE_NAME:
                // write to flash
                memcpy(&m_ble_advertising_name[0], &buf[1], MAX_BLE_NAME_LENGTH);
                write_flash_ble_advertisement_name(&m_ble_advertising_name[0], MAX_BLE_NAME_LENGTH);
                
                // update current advertising data
                update_advertising();
                break;
            case ALARM_THRESHOLD:
				new_alarm_threshold = *((uint32_t*) (buf + 1));
				if(new_alarm_threshold  != -1) {
					// write to flash
					write_flash_alarm_threshold(&new_alarm_threshold);
					nrf_delay_ms(10);
					// update in algorithm, must be thread-safe
					is_new_alarm_threshold = true;
					NRF_LOG_INFO("** New Alarm Level %d", new_alarm_threshold);
					for(int i = 0; i < 10; ++i)
						if(is_new_alarm_threshold)
							vTaskDelay(0);
				} else {
					NRF_LOG_INFO("** Alarm Level Requested");
				}
                break;
            case DEFAULT_MODE:
                // write to flash
                write_flash_active_mode(&buf[1]);

                // update in main app
                m_current_vband_mode = buf[1];
                break;
            case HEART_BEAT:
                // save latest heart beat value
                memcpy(&m_ble_heart_beat, (uint32_t *)&buf[1], sizeof(uint32_t));
                break;
            default:
                NRF_LOG_DEBUG("Invalid Config command written, ignoring");
                break;
        }

        // update characteristic back to show current config
        update_vband_config_characteristic();
    }/* else if(p_evt->type == BLE_VBAND_SRV_EVT_WRITE_EN) {
		if(m_bSendAlarm) {
			m_bSendAlarm = false;
			update_vband_config_characteristic();
		}
	}*/
}

/**@brief Function for initializing services that will be used by the application.
 *
 * @details Initialize the Battery and Device Information services.
 */
static void services_init(void)
{
    ret_code_t             err_code;
    ble_bas_init_t         bas_init;
    ble_dis_init_t         dis_init;
    ble_vband_srv_init_t   vband_init;
    nrf_ble_qwr_init_t qwr_init = {0};

    // Initialize Queued Write Module.
    qwr_init.error_handler = nrf_qwr_error_handler;

    err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init);
    APP_ERROR_CHECK(err_code);

    // Initialize Voltage Band Service.
    memset(&vband_init, 0, sizeof(vband_init));

    vband_init.evt_handler = vband_data_handler;

    err_code = ble_vband_srv_init(&m_vband, &vband_init);
    APP_ERROR_CHECK(err_code);

    // Initialize Battery Service.
    memset(&bas_init, 0, sizeof(bas_init));

    // Here the sec level for the Battery Service can be changed/increased.
    bas_init.bl_rd_sec        = SEC_OPEN;
    bas_init.bl_cccd_wr_sec   = SEC_OPEN;
    bas_init.bl_report_rd_sec = SEC_OPEN;

    bas_init.evt_handler          = NULL;
    bas_init.support_notification = true;
    bas_init.p_report_ref         = NULL;
    bas_init.initial_batt_level   = 100;

    err_code = ble_bas_init(&m_bas, &bas_init);
    APP_ERROR_CHECK(err_code);

    // Initialize Device Information Service.
    memset(&dis_init, 0, sizeof(dis_init));

    ble_srv_ascii_to_utf8(&dis_init.manufact_name_str, (char *)MANUFACTURER_NAME);

    dis_init.dis_char_rd_sec = SEC_OPEN;

    err_code = ble_dis_init(&dis_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the sensor simulators. */
static void sensor_simulator_init(void)
{
    m_battery_sim_cfg.min          = MIN_BATTERY_LEVEL;
    m_battery_sim_cfg.max          = MAX_BATTERY_LEVEL;
    m_battery_sim_cfg.incr         = BATTERY_LEVEL_INCREMENT;
    m_battery_sim_cfg.start_at_max = true;

    sensorsim_init(&m_battery_sim_state, &m_battery_sim_cfg);
}


/**@brief   Function for starting application timers.
 * @details Timers are run after the scheduler has started.
 */
static void application_timers_start(void)
{
    ret_code_t         err_code;

    // Start application timers.
    if (pdPASS != xTimerStart(m_battery_timer, OSTIMER_WAIT_FOR_QUEUE))
    {
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }
}


/**@brief Function for handling the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module which
 *          are passed to the application.
 *          @note All this function does is to disconnect. This could have been done by simply
 *                setting the disconnect_on_fail config parameter, but instead we use the event
 *                handler mechanism to demonstrate its use.
 *
 * @param[in]   p_evt   Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    ret_code_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for handling a Connection Parameters error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module. */
static void conn_params_init(void)
{
    ret_code_t             err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
static void sleep_mode_enter(void)
{
    ret_code_t err_code;

    err_code = bsp_indication_set(BSP_INDICATE_IDLE);
    APP_ERROR_CHECK(err_code);

    // turn off buzzer
    set_buzzer_status(BUZZER_OFF, BUZZER_LOOP_FOREVER);

    // turn off motor
    // not implemented yet

    // turn off led
    set_led_status(LED_OFF);

    // suspend tasks
    vTaskSuspendAll();
//    if(NULL != saadc_sample_timer_handle) vTaskSuspend(saadc_sample_timer_handle);
    //if(NULL != ccs811_measure_task_handle) vTaskSuspend(ccs811_measure_task_handle);
    //if(NULL != bme280_measure_task_handle) vTaskSuspend(bme280_measure_task_handle);
    //if(NULL != max30105_measure_task_handle) vTaskSuspend(max30105_measure_task_handle);
    //if(NULL != adxl362_measure_task_handle) vTaskSuspend(adxl362_measure_task_handle);
    vband_saadc_disable_rtc();

    // turn off analog section
    nrf_gpio_pin_set(VSENSOR_POWER);

    // set capacitive touch sensor to low power mode
    nrf_gpio_pin_clear(CAPSENSOR_MODE);

    // Put any connected sensors into shutdown mode, uninit TWI
    vband_sensor_shutdown(BME280 | CCS811 | MAX30105 | ADXL362);
    
    nrf_gpio_cfg_default(CCS811_WAKEUP_PIN);
    nrf_gpio_cfg_default(CCS811_RESET_PIN);

    // disable DCDC converter, need to use the softdevice functions because memory access is restricted
    sd_power_dcdc_mode_set(NRF_POWER_DCDC_DISABLE);  // for core 2.7V to 1.3V
    sd_power_dcdc0_mode_set(NRF_POWER_DCDC_DISABLE);  // for VDDH to 2.7V

    //vTaskEndScheduler(); // not implemented?

    // wait some time for tasks to finish, VDD to stablize
    nrf_delay_ms(100);

#if defined(BOARD_VWEDGE_V1) || defined(BOARD_VWEDGE_V2)
    // Prepare wakeup buttons.
    err_code = bsp_btn_ble_sleep_mode_prepare();
    APP_ERROR_CHECK(err_code);

    // wait for adxl to indicate sleep state
    adxl362_wait_for_sleep();

    // Prepare activity interrupt
    err_code = adxl362_configure_wakeup();
    APP_ERROR_CHECK(err_code);
#endif

    // clear latched detect for motion interrupt
    nrf_gpio_pin_latch_clear(ADXL362_INT1);
    // Go to system-off mode (this function will not return; wakeup will cause a reset).
#if NRF_LOG_ENABLED
    NRF_LOG_INFO("shut down");
    NRF_LOG_FLUSH();
#endif

    sd_power_system_off();
    
    while(1);
}


/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    uint32_t err_code;

    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
#if NRF_LOG_ENABLED
            NRF_LOG_INFO("Fast advertising.");
#endif
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_ADV_EVT_IDLE:
#if NRF_LOG_ENABLED
            NRF_LOG_INFO("Advertising timeout.");
#endif
            sleep_mode_enter();
            break;

        default:
            break;
    }
}


/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    uint32_t err_code;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            m_ble_connected_bool = true;
#if NRF_LOG_ENABLED
            NRF_LOG_INFO("Connected");
#endif
            //err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING_SLOW);
            err_code = bsp_indication_set(BSP_INDICATE_IDLE);
            APP_ERROR_CHECK(err_code);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);
            APP_ERROR_CHECK(err_code);

            // Put any connected sensors into measurement mode
            vband_sensor_wakeup(BME280 | CCS811 | MAX30105 | ADXL362);

            m_buzzer_block_timeout = 15;
            set_buzzer_status(BUZZER_ON_LONG_BEEP, BUZZER_LOOP_1);

            break;

        case BLE_GAP_EVT_DISCONNECTED:
            // Put any connected sensors into sleep mode while advertising
//            vband_sensor_sleep(BME280 | CCS811 | MAX30105 | ADXL362);

            m_ble_connected_bool = false;
#if NRF_LOG_ENABLED
            uint8_t disconnect_reason = p_ble_evt->evt.gap_evt.params.disconnected.reason;
            if(disconnect_reason  == BLE_HCI_CONNECTION_TIMEOUT) { NRF_LOG_INFO("Disconnected, Conn Timeout"); }
			else { NRF_LOG_INFO("Disconnected, other"); }
#endif
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
//            APP_ERROR_CHECK(err_code);
			ret_code_t err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
            break;

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
            NRF_LOG_DEBUG("PHY update request.");
            ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            NRF_LOG_DEBUG("GATT Client Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            NRF_LOG_DEBUG("GATT Server Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

#if 1
	// increase the notification queue
	ble_cfg_t ble_cfg;
    memset(&ble_cfg, 0, sizeof ble_cfg);
    ble_cfg.conn_cfg.conn_cfg_tag = APP_BLE_CONN_CFG_TAG;
    ble_cfg.conn_cfg.params.gatts_conn_cfg.hvn_tx_queue_size = 3;
    err_code = sd_ble_cfg_set(BLE_CONN_CFG_GATTS, &ble_cfg, ram_start);
    APP_ERROR_CHECK(err_code);
#endif

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}


/**@brief Function for handling events from the BSP module.
 *
 * @param[in]   event   Event generated by button press.
 */
static void bsp_event_handler(bsp_event_t event)
{
    ret_code_t err_code;

    switch (event)
    {
        case BSP_EVENT_SLEEP:
            sleep_mode_enter();
            break;

        case BSP_EVENT_DISCONNECT:
            err_code = sd_ble_gap_disconnect(m_conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            if (err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            }
            break;

        case BSP_EVENT_WHITELIST_OFF:
            
            if (m_conn_handle == BLE_CONN_HANDLE_INVALID)
            {
                err_code = ble_advertising_restart_without_whitelist(&m_advertising);
                if (err_code != NRF_ERROR_INVALID_STATE)
                {
                    APP_ERROR_CHECK(err_code);
                }
            }
            break;

        default:
            break;
    }
}


/**@brief Function for the Peer Manager initialization. */
static void peer_manager_init(void)
{
    ble_gap_sec_params_t sec_param;
    ret_code_t           err_code;

    err_code = pm_init();
    APP_ERROR_CHECK(err_code);

    memset(&sec_param, 0, sizeof(ble_gap_sec_params_t));

    // Security parameters to be used for all security procedures.
    sec_param.bond           = SEC_PARAM_BOND;
    sec_param.mitm           = SEC_PARAM_MITM;
    sec_param.lesc           = SEC_PARAM_LESC;
    sec_param.keypress       = SEC_PARAM_KEYPRESS;
    sec_param.io_caps        = SEC_PARAM_IO_CAPABILITIES;
    sec_param.oob            = SEC_PARAM_OOB;
    sec_param.min_key_size   = SEC_PARAM_MIN_KEY_SIZE;
    sec_param.max_key_size   = SEC_PARAM_MAX_KEY_SIZE;
    sec_param.kdist_own.enc  = 1;
    sec_param.kdist_own.id   = 1;
    sec_param.kdist_peer.enc = 1;
    sec_param.kdist_peer.id  = 1;

    err_code = pm_sec_params_set(&sec_param);
    APP_ERROR_CHECK(err_code);

    err_code = pm_register(pm_evt_handler);
    APP_ERROR_CHECK(err_code);
}


/**@brief Clear bond information from persistent storage. */
static void delete_bonds(void)
{
    ret_code_t err_code;

    NRF_LOG_INFO("Erase bonds!");

    err_code = pm_peers_delete();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the Advertising functionality. */
static void advertising_init(void)
{
    ret_code_t             err_code;
    ble_advertising_init_t init;

    memset(&init, 0, sizeof(init));

    init.advdata.name_type               = BLE_ADVDATA_FULL_NAME;
    init.advdata.include_appearance      = true;
    init.advdata.flags                   = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
    init.advdata.uuids_complete.uuid_cnt = 2;
    init.advdata.uuids_complete.p_uuids  = &m_adv_uuids[0];

    init.srdata.uuids_complete.uuid_cnt = 1;
    init.srdata.uuids_complete.p_uuids  = &m_adv_uuids[2];

    init.config.ble_adv_fast_enabled  = true;
    init.config.ble_adv_fast_interval = APP_ADV_INTERVAL;
    init.config.ble_adv_fast_timeout  = APP_ADV_DURATION;

    init.evt_handler = on_adv_evt;

    err_code = ble_advertising_init(&m_advertising, &init);
    APP_ERROR_CHECK(err_code);

    ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);
}

static uint32_t log_timestamp(void)
{
  return nrf_rtc_counter_get(NRF_RTC1)*2594/84; // convert from 32khz tick to microseconds
}

/**@brief Function for initializing the nrf log module.
 */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(log_timestamp);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}




/**@brief Function for initializing buttons and leds.
 *
 * @param[out] p_erase_bonds  Will be true if the clear bonding button was pressed to wake the application up.
 */
static void buttons_leds_init(bool * p_erase_bonds)
{
    ret_code_t err_code;
    bsp_event_t startup_event;

    err_code = bsp_init(BSP_INIT_LEDS | BSP_INIT_BUTTONS, bsp_event_handler);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_btn_ble_init(NULL, &startup_event);
    APP_ERROR_CHECK(err_code);

    *p_erase_bonds = (startup_event == BSP_EVENT_CLEAR_BONDING_DATA);
}


/**@brief Function for starting advertising. */
static void advertising_start(void * p_erase_bonds)
{
    bool erase_bonds = *(bool*)p_erase_bonds;

    if (erase_bonds)
    {
        delete_bonds();
        // Advertising is started by PM_EVT_PEERS_DELETE_SUCCEEDED event.
    }
    else
    {
        ret_code_t err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
        APP_ERROR_CHECK(err_code);
    }

    // Put any connected sensors into sleep mode while advertising
//    vband_sensor_sleep(BME280 | CCS811 | MAX30105 | ADXL362);
}


#if NRF_LOG_ENABLED
/**@brief Thread for handling the logger.
 *
 * @details This thread is responsible for processing log entries if logs are deferred.
 *          Thread flushes all log entries and suspends. It is resumed by idle task hook.
 *
 * @param[in]   arg   Pointer used for passing some arbitrary information (context) from the
 *                    osThreadCreate() call to the thread.
 */
static void logger_thread(void * arg)
{
    UNUSED_PARAMETER(arg);

    while (1)
    {
        //NRF_LOG_INFO("Idle");
        NRF_LOG_FLUSH();

        vTaskSuspend(NULL); // Suspend myself
    }
}
#endif //NRF_LOG_ENABLED

// update_advertising() is not in SDK example

static void update_advertising()
{
    ret_code_t err_code;
    ble_gap_conn_sec_mode_t sec_mode;
    static bool buffer_index; 
    static ble_gap_adv_data_t new_data;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)&m_ble_advertising_name,
                                          sizeof(m_ble_advertising_name));
    APP_ERROR_CHECK(err_code);   
    
    // We alternate between two data buffer in order to hot-swap
    ble_gap_adv_data_t *old_data  = &m_advertising.adv_data;

    // Copy advertising data
    static uint8_t data[2][BLE_GAP_ADV_SET_DATA_SIZE_MAX];
    new_data.adv_data.p_data      = data[buffer_index];
    new_data.adv_data.len         = old_data->adv_data.len;
    memcpy(new_data.adv_data.p_data, old_data->adv_data.p_data, old_data->adv_data.len);

    // Copy scan response data
    static uint8_t scan_data[2][BLE_GAP_ADV_SET_DATA_SIZE_MAX];
    new_data.scan_rsp_data.p_data = scan_data[buffer_index];
    new_data.scan_rsp_data.len    = old_data->scan_rsp_data.len;
    memcpy(new_data.scan_rsp_data.p_data,
           old_data->scan_rsp_data.p_data,
           old_data->scan_rsp_data.len);

    err_code = ble_advertising_advdata_update(&m_advertising, &new_data, true);
    APP_ERROR_CHECK(err_code);

    buffer_index = !buffer_index;
}

/**@snippet [Handling the data received over BLE] */


/**@brief A function which is hooked to idle task.
 * @note Idle hook must be enabled in FreeRTOS configuration (configUSE_IDLE_HOOK).
 */
void vApplicationIdleHook( void )
{
#if NRF_LOG_ENABLED
     vTaskResume(m_logger_thread);
#endif
    __set_FPSCR(__get_FPSCR() & ~(0x0000009F)); 
    (void) __get_FPSCR();
    sd_nvic_ClearPendingIRQ(FPU_IRQn);
    
    // check if BLE was connected
    if(m_ble_connected_bool)
    {
      if(~m_ble_was_connected_bool)
      {
        m_ble_was_connected_bool = true;
        // BLE connected, make a sound indication
        //set_buzzer_status(BUZZER_ON_LONG_BEEP, BUZZER_LOOP_1);
      }
    }
    else
    {
      m_ble_was_connected_bool = false;
    }

    //sd_app_evt_wait(); //does not lower current consumption
}

/**@brief Function for initializing the clock.
 */
static void clock_init(void)
{
    ret_code_t err_code = nrf_drv_clock_init();
    APP_ERROR_CHECK(err_code);
    nrf_drv_clock_lfclk_request(NULL); // not in SDK exmaple
}

void update_vband_config_characteristic(void)
{
    //ble_vband_srv_config_mode_t op_mode;           /**< Operating Mode for Device. */
    //uint16_t                    alarm_threshold;   /**< Alarm Threshold. */
    static uint8_t p_data[BLE_VBAND_CONFIG_DATA_LEN];
    static uint16_t p_data_length = 6;
    
    memset(&p_data[0],0,BLE_VBAND_CONFIG_DATA_LEN);
    p_data[0] = m_current_vband_mode;

    read_flash_alarm_threshold((uint32_t *)&p_data[1]); // 4 bytes for alarm threshold

    p_data[5] = m_usb_detected;
    vband_characteristic_update(ble_vband_srv_config_update, &p_data[0], &p_data_length);
}

/**@brief Function for checking BLE keep alive heart beat
 */
static void ble_heart_beat_check_task (void * pvParameter)
{
    static ret_code_t err_code;

    UNUSED_PARAMETER(pvParameter);
    while(1)
    {
        if(m_ble_connected_bool)
        {
            NRF_LOG_INFO("heart beat: %lu, last: %lu", m_ble_heart_beat, m_ble_heart_beat_last);
/*            if(m_ble_heart_beat_last == m_ble_heart_beat)
            {
              NRF_LOG_INFO("try to disconnect");
              // if heart beat value stays the same, then we have lost connection
              err_code = sd_ble_gap_disconnect(m_conn_handle,
                                               BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
              if (err_code != NRF_SUCCESS)
              {
                NRF_LOG_INFO("error code: %d", err_code);
                APP_ERROR_CHECK(err_code);
              }
              m_conn_handle = BLE_CONN_HANDLE_INVALID;
              m_ble_heart_beat = 1;
            }
            else 
            {
              m_ble_heart_beat_last = m_ble_heart_beat;
            }
            */
        }
        vTaskDelay(HEART_BEAT_CHECK_INTERVAL);
    }
}

/**@brief Function for pinging the CCS811 for a measurement.
 */
static void ccs811_measure_task (void * pvParameter)
{
    static uint8_t p_data[BLE_VBAND_NORMAL_DATA_LEN];
    static uint16_t p_data_length;

    UNUSED_PARAMETER(pvParameter);
    while(1)
    {
        if(m_ble_connected_bool)
        {
            if(xSemaphoreTake(i2c_semaphore,CCS811_MEASURE_INTERVAL))
            {
                if(get_sensor_data(CCS811, &p_data[0], &p_data_length) == WEDGE_SENSOR_SUCCESS)
                {
                  vband_characteristic_update(ble_vband_srv_gas_sensor_update, &p_data[0], &p_data_length);
                }
                xSemaphoreGive(i2c_semaphore);
            }
        }
        vTaskDelay(CCS811_MEASURE_INTERVAL);
    }
}


/**@brief Function for pinging the BME280 for a measurement.
 */
static void bme280_measure_task (void * pvParameter)
{
    static uint8_t p_data[BLE_VBAND_NORMAL_DATA_LEN];
    static uint16_t p_data_length;

    UNUSED_PARAMETER(pvParameter);
    while(1)
    {
        if(m_ble_connected_bool)
        {
            if(xSemaphoreTake(i2c_semaphore,BME280_MEASURE_INTERVAL))
            {
                if(get_sensor_data(BME280, &p_data[0], &p_data_length) == WEDGE_SENSOR_SUCCESS)
                {
                  vband_characteristic_update(ble_vband_srv_temp_humid_pressure_update, &p_data[0], &p_data_length);
                }
                xSemaphoreGive(i2c_semaphore);
            }
        }
        vTaskDelay(BME280_MEASURE_INTERVAL);
    }
}


/**@brief Function for pinging the MAX30105 for a measurement.
 */
static void max30105_measure_task (void * pvParameter)
{
    static uint8_t p_data[BLE_VBAND_NORMAL_DATA_LEN];
    static uint16_t p_data_length;
    static max30105_state_t current_state = PRESENCE_NOT_DETECTED;
    static uint16_t thread_interval = MAX30105_MEASURE_PROXIMITY_INTERVAL;

    UNUSED_PARAMETER(pvParameter);
    while(1)
    {
        if(m_ble_connected_bool)
        {
            if(xSemaphoreTake(i2c_semaphore,thread_interval))
            {
                if(get_sensor_data(MAX30105, &p_data[0], &p_data_length) == WEDGE_SENSOR_SUCCESS)
                {
                /*if(p_data[0] != current_state)
                {
                    current_state = p_data[0];
                    if(current_state == PRESENCE_DETECTED)
                    {
                        thread_interval = MAX30105_MEASURE_HEART_RATE_INTERVAL;
                    }
                    else
                    {
                        thread_interval = MAX30105_MEASURE_PROXIMITY_INTERVAL;
                    }
                }*/

                  vband_characteristic_update(ble_vband_srv_optical_sensor_update, &p_data[0], &p_data_length);
                }
                xSemaphoreGive(i2c_semaphore);
            }
        }
        vTaskDelay(thread_interval);
    }
}


/**@brief Function for pinging the ADXL362 for a measurement.
 */
static void adxl362_measure_task (void * pvParameter)
{
    static uint8_t p_data[BLE_VBAND_NORMAL_DATA_LEN];
    static uint16_t p_data_length;
    static uint32_t consecutive_unawakes = 0;
    static uint32_t timeout_ms = ADXL362_INACTIVITY_WHILE_CONNECTED_TIMEOUT*1000;

    UNUSED_PARAMETER(pvParameter);
    while(1)
    {
        if(get_sensor_data(ADXL362, &p_data[0], &p_data_length) == WEDGE_SENSOR_SUCCESS)
        {
          if(m_ble_connected_bool)
          {
              vband_characteristic_update(ble_vband_srv_accelerometer_update, &p_data[0], &p_data_length);
              consecutive_unawakes = (p_data[6] == 0) ? (consecutive_unawakes + 1) : 0;
              if((consecutive_unawakes*ADXL362_MEASURE_INTERVAL) >= timeout_ms)
              {
                  NRF_LOG_ERROR("Inactivity while connected timeout.");
                  sleep_mode_enter();
              }
          }
        }
        else consecutive_unawakes = 0; // we only increment this while we're connected, the advertising timeout should go to sleep

        vTaskDelay(ADXL362_MEASURE_INTERVAL);
    }
}


/**@brief Function for initializeing external sensors.
 */
static void external_sensor_init(void)
{
//    BaseType_t xReturned;
#ifdef BOARD_PCA10056
    uint16_t enabled_sensors = ADXL362 | BME280 | MAX30105;
#elif BOARD_VWEDGE_V2
    uint16_t enabled_sensors = ADXL362 | VSENSOR | BME280 | CCS811;
    //uint16_t enabled_sensors = ADXL362 | VSENSOR | BME280;
#else
    uint16_t enabled_sensors = ADXL362 | VSENSOR | BME280 | CCS811 | MAX30105;
    //uint16_t enabled_sensors = ADXL362 | VSENSOR | BME280 | CCS811 | MAX30105 | POZYX;
#endif

    enabled_sensors = vband_sensor_init(enabled_sensors);

    // start tasks for each sensor
    if(enabled_sensors & CCS811)
    {
      if(pdPASS != xTaskCreate(ccs811_measure_task, "CCS811", configMINIMAL_STACK_SIZE + 200, NULL, 1, &ccs811_measure_task_handle))
      {
        NRF_LOG_ERROR("CCS811 task not created.");
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
      }
      //else NRF_LOG_INFO("CCS811 task created");
    }

    if(enabled_sensors & BME280)
    {
      if(pdPASS != xTaskCreate(bme280_measure_task, "BME280", configMINIMAL_STACK_SIZE + 200, NULL, 1, &bme280_measure_task_handle))
      {
        NRF_LOG_ERROR("BME280 task not created.");
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
      }
      //else NRF_LOG_INFO("BME280 task created");
    }

    if(enabled_sensors & MAX30105)
    {
      if(pdPASS != xTaskCreate(max30105_measure_task, "MAX30105", configMINIMAL_STACK_SIZE + 200, NULL, 1, &max30105_measure_task_handle))
      {
        NRF_LOG_ERROR("MAX30105 task not created.");
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
      }
      //else NRF_LOG_INFO("MAX30105 task created");
    }

    if(enabled_sensors & ADXL362)
    {
      if(pdPASS != xTaskCreate(adxl362_measure_task, "ADXL362", configMINIMAL_STACK_SIZE + 200, NULL, 1, &adxl362_measure_task_handle))
      {
        NRF_LOG_ERROR("ADXL362 task not created.");
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
      }
      //else NRF_LOG_INFO("ADXL362 task created");
    }
}


static void saadc_sample_task (void * pvParameter)
{
    static uint8_t * p_data;
    static uint8_t * p_data_length;
    static uint16_t  wakeup_interval = SAADC_WAKEUP_SAMPLE_INTERVAL_DISCONNECTED;

    UNUSED_PARAMETER(pvParameter);
    while(1)
    {
        if(is_new_alarm_threshold)
        {
            set_voltage_alarm_threshold(&new_alarm_threshold);
            is_new_alarm_threshold = false;
        }

        if(xSemaphoreTake(i2c_semaphore,wakeup_interval))
        {
            vband_saadc_sample_electrode_channels();
            xSemaphoreGive(i2c_semaphore);
        }

        // change sampling interval if we're connected
        if (m_ble_connected_bool)
        {
            wakeup_interval = SAADC_WAKEUP_SAMPLE_INTERVAL_CONNECTED;
        }
        else
        {
            wakeup_interval = SAADC_WAKEUP_SAMPLE_INTERVAL_DISCONNECTED;
        }

        vTaskDelay(wakeup_interval);
    }
}

static void saadc_run_voltage_alarm_algorithm(float * adc_ch1, float * adc_ch2, float * adc_ch3, uint16_t len)
{
    static voltage_algorithm_results results;
    static uint8_t p_data[sizeof(results)];
    static uint16_t p_data_length = sizeof(results);
    static bool current_alarm_status = false;

    current_alarm_status = check_for_voltage_detection(&p_data[0], adc_ch1, adc_ch2, adc_ch3, len);

    // control buzzer
    // do not touch buzzer while the timeout counter is positive
    if(m_buzzer_block_timeout)
    {
        m_buzzer_block_timeout--;
    }
    else
    {
        if((current_alarm_status == true) && (m_usb_detected == false))
        {
            set_buzzer_status(BUZZER_ON_ALARM, BUZZER_LOOP_FOREVER); // consecutive states return without re-initializing
        }
        else
        {
            set_buzzer_status(BUZZER_OFF, BUZZER_LOOP_FOREVER);
        }
    }

    // update ble
    if (m_ble_connected_bool)
    {
        p_data_length = sizeof(results);
        vband_characteristic_update(ble_vband_srv_alarm_update, &p_data[0], &p_data_length);
    }
}

static void saadc_init(void)
{
//    BaseType_t xReturned;
    
    saadc_assign_callback_fn(saadc_run_voltage_alarm_algorithm);

    // start tasks for each sensor
    if(pdPASS != xTaskCreate(saadc_sample_task, "SAADC", configMINIMAL_STACK_SIZE + 200, NULL, 1, &saadc_sample_timer_handle))
    {
        NRF_LOG_ERROR("SAADC task not created.");
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }
    else NRF_LOG_INFO("SAADC task created");
}

/**@brief Function for application main entry.
 */
int main(void)
{
    bool erase_bonds = false;
    float battery_voltage = 0;
    NRF_POWER->RESETREAS =  0xffffffff;
    // Initialize semaphores
    i2c_semaphore = xSemaphoreCreateMutex();

    // Initialize modules.
#if NRF_LOG_ENABLED
    log_init();
#endif
    clock_init();

    // initialize GPIO
    nrf_gpio_cfg_output(BUZZER_GPIO);
    nrf_gpio_cfg_output(MOTOR_GPIO);
    nrf_gpio_cfg_output(LED_1);
#ifdef BOARD_VWEDGE_V2
    nrf_gpio_cfg(CCS811_WAKEUP_PIN, NRF_GPIO_PIN_DIR_OUTPUT, NRF_GPIO_PIN_INPUT_DISCONNECT, NRF_GPIO_PIN_PULLUP, NRF_GPIO_PIN_S0D1, NRF_GPIO_PIN_NOSENSE);
    nrf_gpio_cfg(CCS811_RESET_PIN, NRF_GPIO_PIN_DIR_OUTPUT, NRF_GPIO_PIN_INPUT_DISCONNECT, NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_S0D1, NRF_GPIO_PIN_NOSENSE);
#else
    nrf_gpio_cfg_output(CCS811_WAKEUP_PIN);
#endif
    nrf_gpio_pin_set(CCS811_WAKEUP_PIN);
    nrf_gpio_pin_set(CCS811_RESET_PIN);
    nrf_gpio_cfg_input(CAPSENSOR_BUTTON0, NRF_GPIO_PIN_NOPULL);
    nrf_gpio_cfg_input(CAPSENSOR_BUTTON1, NRF_GPIO_PIN_NOPULL);

#ifdef ADXL362_POWER_ON
    nrf_gpio_cfg_output(ADXL362_POWER_ON);
    nrf_gpio_pin_clear(ADXL362_POWER_ON);
#endif

#ifdef LDO_SET_PIN
    nrf_gpio_cfg_output(LDO_SET_PIN);
    nrf_gpio_pin_clear(LDO_SET_PIN);
#endif
#ifdef LDO_1_8_EN_PIN
    nrf_gpio_cfg_output(LDO_1_8_EN_PIN);
    nrf_gpio_pin_clear(LDO_1_8_EN_PIN);
#endif
#ifdef USB_DETECT_PIN
    nrf_gpio_cfg_input(USB_DETECT_PIN, NRF_GPIO_PIN_NOPULL);
#endif

    // enable DCDCs to reduce power consumption
    NRF_POWER->DCDCEN = 1;  // for core 2.7V to 1.3V
    NRF_POWER->DCDCEN0 = 1; // for VDDH to 2.7V

    // Do not start any interrupt that uses system functions before system initialisation.
    // The best solution is to start the OS before any other initalisation.

#if NRF_LOG_ENABLED
    // Start execution.
    if (pdPASS != xTaskCreate(logger_thread, "LOGGER", 256, NULL, 1, &m_logger_thread))
    {
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }
    else NRF_LOG_INFO("LOGGER task created");
#endif

    // Activate deep sleep mode.
    SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;

#if NRF_LOG_ENABLED
    NRF_LOG_INFO("Reboot.");
#endif

    // Initialize modules.
    initialize_flash();

    // Set advertising name from flash
    if (!read_flash_ble_advertisement_name(&m_ble_advertising_name[0]))
    {
        if(sizeof(DEVICE_NAME) <= MAX_BLE_NAME_LENGTH)
        {
          memcpy(&m_ble_advertising_name[0], (const uint8_t *)DEVICE_NAME, sizeof(DEVICE_NAME));
        }
        else
        {
          ASSERT(false); // default device name is too long
        }
    }

    // Set alarm threshold from flash
    uint32_t flash_alarm_threshold = 0;
    if (read_flash_alarm_threshold(&flash_alarm_threshold))
    {
        set_voltage_alarm_threshold(&flash_alarm_threshold);
    }
    // temporary set threshold high
    //flash_alarm_threshold = 30000;
    //set_voltage_alarm_threshold(&flash_alarm_threshold);

    // Set engineering mode from flash
    if (!read_flash_active_mode(&m_current_vband_mode))
    {
        m_current_vband_mode = BLE_VBAND_SRV_MODE_ENGINEERING; // if invalid flash default to normal mode
    }

#if NRF_LOG_ENABLED
    NRF_LOG_FLUSH();
#endif

    // Configure and initialize the BLE stack.
    ble_stack_init();

    timers_init(); // FreeRTOS timers
    buttons_leds_init(&erase_bonds);
    gap_params_init();
    gatt_init();
    services_init();
    advertising_init();
    sensor_simulator_init();
    conn_params_init();
    //peer_manager_init();
    application_timers_start(); // starts FreeRTOS timers (Battery update)

#if NRF_LOG_ENABLED
    NRF_LOG_INFO("going to sensor init");
    NRF_LOG_FLUSH();
#endif
    
    // added for VBAND functions
    external_sensor_init();
    saadc_init();


    //pm_peers_delete();
#if NRF_LOG_ENABLED
    NRF_LOG_INFO("passed initialization");
    NRF_LOG_FLUSH();
#endif

    // check battery level. if battery is too low, go to sleep
#if defined(BOARD_VWEDGE_V1) || defined(BOARD_VWEDGE_V2)
   
    vband_saadc_sample_electrode_channels();
    while(battery_voltage == 0)
    {
      vband_saadc_read_battery_voltage(&battery_voltage);
    }
    //NRF_LOG_INFO("Battery at %1.2fV",battery_voltage*2.0989f);
    NRF_LOG_INFO("Battery at: " NRF_LOG_FLOAT_MARKER, NRF_LOG_FLOAT(battery_voltage));
    if(battery_voltage < 0.51f)
    {
      NRF_LOG_INFO("Battery level low, shutdown");
      //nrf_delay_ms(2000); // need some delay for the accelerometer to assert sleep interrupt
      nrf_sdh_freertos_init(sleep_mode_enter, NULL);
      vTaskStartScheduler();
    }
#endif

    // battery is ok, make a boot up sound
    set_buzzer_status(BUZZER_ON_ALARM, BUZZER_LOOP_2);
    //set_buzzer_status(BUZZER_ON_LONG_BEEP, BUZZER_LOOP_1);
    //set_buzzer_status(BUZZER_ON_WARNING, BUZZER_LOOP_1);

    if(pdPASS != xTaskCreate(ble_heart_beat_check_task, "BLEHB", configMINIMAL_STACK_SIZE + 200, NULL, 1, &m_heart_beat_thread))
    {
      NRF_LOG_ERROR("heart beat checking task not created.");
      APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }

    // Create a FreeRTOS task for the BLE stack.
    // The task will run advertising_start() before entering its loop.

    nrf_sdh_freertos_init(advertising_start, &erase_bonds);

#if NRF_LOG_ENABLED
    NRF_LOG_INFO("Voltage Band FreeRTOS Scheduler started.");
    NRF_LOG_FLUSH();
#endif

    // Start FreeRTOS scheduler.
    vTaskStartScheduler();

    for (;;)
    {
        APP_ERROR_HANDLER(NRF_ERROR_FORBIDDEN);
    }
}


#if defined(configCHECK_FOR_STACK_OVERFLOW) && configCHECK_FOR_STACK_OVERFLOW

//*****************************************************************************
//
// This hook is called by FreeRTOS when an stack overflow error is detected.
//
//*****************************************************************************
void vApplicationStackOverflowHook(xTaskHandle *pxTask, char *pcTaskName) {
    //
    // This function can not return, so loop forever.  Interrupts are disabled
    // on entry to this function, so no processor interrupts will interrupt
    // this loop.
    //
    while(1) {
    }
}

#endif

