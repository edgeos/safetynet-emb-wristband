#ifndef BLE_VBAND_SRV_H__
#define BLE_VBAND_SRV_H__

#include <stdint.h>
#include <stdbool.h>
#include "sdk_config.h"
#include "ble.h"
#include "ble_srv_common.h"
#include "nrf_sdh_ble.h"
#include "ble_link_ctx_manager.h"

#ifdef __cplusplus
extern "C" {
#endif

/**@brief Macro for defining a ble_bas instance.
 *
 * @param   _name  Name of the instance.
 * @hideinitializer
 */
#define BLE_VBAND_SRV_DEF(_name)                          \
    static ble_vband_srv_t _name;                         \
    NRF_SDH_BLE_OBSERVER(_name ## _obs,                   \
                         BLE_VBAND_SRV_BLE_OBSERVER_PRIO, \
                         ble_vband_srv_on_ble_evt,        \
                         &_name)


// 4001c63b-0b4c-4d95-8451-a0d4a5d77036 from https://www.uuidgenerator.net/version4
#define VBAND_SERVICE_UUID_BASE          {0x36, 0x70, 0xD7, 0xA5, 0xD4, 0xA0, 0x51, 0x84, \
                                          0x4D, 0x95, 0x0B, 0x4C, 0x3B, 0xC6, 0x01, 0x40}

#define BLE_UUID_VOLTAGE_WRISTBAND_SERVICE           0x0001
#define VOLTAGE_ALARM_STATE_CHAR_UUID                0x0002
#define VOLTAGE_ALARM_CONFIG_CHAR_UUID               0x0003
#define ACCELEROMETER_DATA_CHAR_UUID                 0x0004
#define TEMP_HUMID_PRESSURE_DATA_CHAR_UUID           0x0005
#define GAS_SENSOR_DATA_CHAR_UUID                    0x0006
#define OPTICAL_SENSOR_DATA_CHAR_UUID                0x0007
#define STREAMING_DATA_CHAR_UUID                     0x0008

/**@brief   Maximum length of data (in bytes) that can be transmitted to the peer by the Nordic UART service module. */
#if defined(NRF_SDH_BLE_GATT_MAX_MTU_SIZE) && (NRF_SDH_BLE_GATT_MAX_MTU_SIZE != 0)
    #define BLE_VBAND_MAX_DATA_LEN (NRF_SDH_BLE_GATT_MAX_MTU_SIZE - 3)
#else
    #define BLE_VBAND_MAX_DATA_LEN (BLE_GATT_MTU_SIZE_DEFAULT - 3)
    #warning NRF_SDH_BLE_GATT_MAX_MTU_SIZE is not defined.
#endif

#define BLE_VBAND_NORMAL_DATA_LEN                    20


/**@brief Voltage Band event type. */
typedef enum
{
    BLE_VBAND_SRV_MODE_NORMAL,
    BLE_VBAND_SRV_MODE_ENGINEERING
} ble_vband_srv_config_mode_t;

/**@brief Voltage Band event type. */
typedef enum
{
    NONE,
    XYZ,
    OPTICAL,
    SAADC_ALL,
    SAADC_CH0,
    SAADC_CH1,
    SAADC_CH2
} ble_vband_stream_data_t;

/**@brief Voltage Band event type. */
typedef enum
{
    BLE_VBAND_SRV_EVT_CONFIG_UPDATED /**< Battery value notification enabled event. */
} ble_vband_srv_evt_type_t;

/**@brief Voltage Band configuration data. */
typedef struct
{
    ble_vband_srv_config_mode_t op_mode;           /**< Operating Mode for Device. */
    uint16_t                    alarm_threshold;   /**< Alarm Threshold. */
    ble_vband_stream_data_t     stream_data;       /**< What data to stream */
} ble_vband_srv_config_t;

/**@brief Voltage Band Service client context structure.
 *
 * @details This structure contains state context related to hosts.
 */
typedef struct
{
    bool is_notification_enabled; /**< Variable to indicate if the peer has enabled notification of the characteristic.*/
} ble_vband_srv_client_context_t;

// Forward declaration of the ble_vband_srv_t type.
typedef struct ble_vband_srv_s ble_vband_srv_t;

/**@brief Voltage Band event. */
typedef struct
{
    ble_vband_srv_evt_type_t          type;        /**< Type of event. */
    ble_vband_srv_t                 * p_vband_srv; /**< A pointer to the instance. */
    uint16_t                          conn_handle; /**< Connection handle. */
    ble_vband_srv_client_context_t  * p_link_ctx;  /**< A pointer to the link context. */
    union
    {
        ble_vband_srv_config_t config_data;        /**< @ref BLE_VBAND_SRV_EVT_CONFIG_UPDATED event data. */
    } params;
} ble_vband_srv_evt_t;

/**@brief Voltage Band Service event handler type. */
typedef void (* ble_vband_srv_handler_t) (ble_vband_srv_evt_t * p_evt);

/**@brief Voltage Band Service update characteristic type. */
typedef uint32_t (* ble_vband_char_update_t) (ble_vband_srv_t * p_vband_srv, uint8_t * p_data, uint16_t * p_length, uint16_t conn_handle);

/**@brief Voltage Band Service init structure. This contains all options and data needed for
 *        initialization of the service.*/
typedef struct
{
    ble_vband_srv_handler_t  evt_handler;                    /**< Event handler to be called for handling events in the Battery Service. */
} ble_vband_srv_init_t;

/**@brief Voltage Band Service structure. This contains various status information for the service. */
struct ble_vband_srv_s
{
    ble_vband_srv_handler_t         evt_handler;                        /**< Event handler to be called for handling events in the Voltage Band Service. */
    uint16_t                        service_handle;                     /**< Handle of Voltage Band Service (as provided by the BLE stack). */
    ble_gatts_char_handles_t        alarm_state_handles;                /**< Handles related to the Voltage Band Alarm State characteristic (as provided by the SoftDevice). */
    ble_gatts_char_handles_t        config_handles;                     /**< Handles related to the Voltage Band Configuration characteristic (as provided by the SoftDevice). */
    ble_gatts_char_handles_t        accel_handles;                      /**< Handles related to the Voltage Band Acceleromoter characteristic (as provided by the SoftDevice). */
    ble_gatts_char_handles_t        temp_humid_pressure_handles;        /**< Handles related to the Voltage Band Temp/Humid/Pressure characteristic (as provided by the SoftDevice). */
    ble_gatts_char_handles_t        gas_sensor_handles;                 /**< Handles related to the Voltage Band Gas Sensor characteristic (as provided by the SoftDevice). */
    ble_gatts_char_handles_t        optical_sensor_handles;             /**< Handles related to the Voltage Band Optical Sensor characteristic (as provided by the SoftDevice). */
    ble_gatts_char_handles_t        streaming_data_handles;             /**< Handles related to the Voltage Band Streaming Data characteristic (as provided by the SoftDevice). */
    blcm_link_ctx_storage_t * const p_link_ctx_storage;                 /**< Pointer to link context storage with handles of all current connections and its context. */
    bool                            is_notification_supported_alarm;    /**< TRUE if notification of Alarm State is supported. */
    bool                            is_notification_supported_accel_d;  /**< TRUE if notification of Accelerometer Data is supported. */
    bool                            is_notification_supported_thp_d;    /**< TRUE if notification of Temp/Humid/Pressure Data is supported. */
    bool                            is_notification_supported_gas_d;    /**< TRUE if notification of Gas Sensor Data is supported. */
    bool                            is_notification_supported_opt_d;    /**< TRUE if notification of Optical Sensor Data is supported. */
    bool                            is_notification_supported_stream_d; /**< TRUE if notification of Streaming Data is supported. */
    uint8_t                         uuid_type;                     /**< UUID type for Voltage Band Service Base UUID. */
};

/**@brief   Function for initializing the Voltage Band Service.
 *
 * @param[out] p_vband_srv      Voltage Band Service structure. This structure must be supplied
 *                              by the application. It is initialized by this function and will
 *                              later be used to identify this particular service instance.
 * @param[in] p_vband_srv_init  Information needed to initialize the service.
 *
 * @retval NRF_SUCCESS If the service was successfully initialized. Otherwise, an error code is returned.
 * @retval NRF_ERROR_NULL If either of the pointers p_vband_srv or p_vband_srv_init is NULL.
 */
uint32_t ble_vband_srv_init(ble_vband_srv_t * p_vband_srv, ble_vband_srv_init_t const * p_vband_srv_init);


/**@brief   Function for handling the Voltage Band Service's BLE events.
 *
 * @details The Voltage Band Service expects the application to call this function each time an
 * event is received from the SoftDevice. This function processes the event if it
 * is relevant and calls the Voltage Band Service event handler of the
 * application if necessary.
 *
 * @param[in] p_ble_evt     Event received from the SoftDevice.
 * @param[in] p_context     Voltage Band Service structure.
 */
void ble_vband_srv_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context);


/**@brief   Function for updating the alarm state characteristic.
 *
 * @details This function sends the alarm state as characteristic notification to the
 *          peer.
 *
 * @param[in]     p_vband_srv       Pointer to the Voltage Band Service structure.
 * @param[in]     p_data            Pointer to data to be sent.
 * @param[in,out] p_length          Pointer Length of the string. Amount of sent bytes.
 * @param[in]     conn_handle       Connection Handle of the destination client.
 *
 * @retval NRF_SUCCESS If the string was sent successfully. Otherwise, an error code is returned.
 */
uint32_t ble_vband_srv_alarm_update(ble_vband_srv_t * p_vband_srv,
                                    uint8_t         * p_data,
                                    uint16_t        * p_length,
                                    uint16_t          conn_handle);


/**@brief   Function for updating the accelerometer characteristic.
 *
 * @details This function sends the accelerometer data as characteristic notification to the
 *          peer.
 *
 * @param[in]     p_vband_srv       Pointer to the Voltage Band Service structure.
 * @param[in]     p_data            Pointer to data to be sent.
 * @param[in,out] p_length          Pointer Length of the string. Amount of sent bytes.
 * @param[in]     conn_handle       Connection Handle of the destination client.
 *
 * @retval NRF_SUCCESS If the string was sent successfully. Otherwise, an error code is returned.
 */
uint32_t ble_vband_srv_accelerometer_update(ble_vband_srv_t * p_vband_srv,
                                            uint8_t         * p_data,
                                            uint16_t        * p_length,
                                            uint16_t          conn_handle);


/**@brief   Function for updating the temp/humid/pressure characteristic.
 *
 * @details This function sends the temp/humid/pressure data as characteristic notification to the
 *          peer.
 *
 * @param[in]     p_vband_srv       Pointer to the Voltage Band Service structure.
 * @param[in]     p_data            Pointer to data to be sent.
 * @param[in,out] p_length          Pointer Length of the string. Amount of sent bytes.
 * @param[in]     conn_handle       Connection Handle of the destination client.
 *
 * @retval NRF_SUCCESS If the string was sent successfully. Otherwise, an error code is returned.
 */
uint32_t ble_vband_srv_temp_humid_pressure_update(ble_vband_srv_t * p_vband_srv,
                                                  uint8_t         * p_data,
                                                  uint16_t        * p_length,
                                                  uint16_t          conn_handle);


/**@brief   Function for updating the gas sensor characteristic.
 *
 * @details This function sends the gas sensor data as characteristic notification to the
 *          peer.
 *
 * @param[in]     p_vband_srv       Pointer to the Voltage Band Service structure.
 * @param[in]     p_data            Pointer to data to be sent.
 * @param[in,out] p_length          Pointer Length of the string. Amount of sent bytes.
 * @param[in]     conn_handle       Connection Handle of the destination client.
 *
 * @retval NRF_SUCCESS If the string was sent successfully. Otherwise, an error code is returned.
 */
uint32_t ble_vband_srv_gas_sensor_update(ble_vband_srv_t * p_vband_srv,
                                         uint8_t         * p_data,
                                         uint16_t        * p_length,
                                         uint16_t          conn_handle);

/**@brief   Function for updating the optical sensor characteristic.
 *
 * @details This function sends the optical sensor data as characteristic notification to the
 *          peer.
 *
 * @param[in]     p_vband_srv       Pointer to the Voltage Band Service structure.
 * @param[in]     p_data            Pointer to data to be sent.
 * @param[in,out] p_length          Pointer Length of the string. Amount of sent bytes.
 * @param[in]     conn_handle       Connection Handle of the destination client.
 *
 * @retval NRF_SUCCESS If the string was sent successfully. Otherwise, an error code is returned.
 */
uint32_t ble_vband_srv_optical_sensor_update(ble_vband_srv_t * p_vband_srv,
                                             uint8_t         * p_data,
                                             uint16_t        * p_length,
                                             uint16_t          conn_handle);

/**@brief   Function for updating the streaming data characteristic.
 *
 * @details This function sends the streaming data as characteristic notification to the
 *          peer.
 *
 * @param[in]     p_vband_srv       Pointer to the Voltage Band Service structure.
 * @param[in]     p_data            Pointer to data to be sent.
 * @param[in,out] p_length          Pointer Length of the string. Amount of sent bytes.
 * @param[in]     conn_handle       Connection Handle of the destination client.
 *
 * @retval NRF_SUCCESS If the string was sent successfully. Otherwise, an error code is returned.
 */
uint32_t ble_vband_srv_streaming_data_update(ble_vband_srv_t * p_vband_srv,
                                             uint8_t         * p_data,
                                             uint16_t        * p_length,
                                             uint16_t          conn_handle);        

#ifdef __cplusplus
}
#endif

#endif /* BLE_VBAND_SRV_H__*/