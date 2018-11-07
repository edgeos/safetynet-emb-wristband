/* This code belongs in ble_cus.c*/
#include "sdk_common.h"
#include "ble_srv_common.h"
#include "ble_vband_srv.h"
#include <string.h>
#include "nrf_gpio.h"
#include "boards.h"
#include "nrf_log.h"


#define OPCODE_LENGTH 1                                                             /**< Length of opcode inside Running Speed and Cadence Measurement packet. */
#define HANDLE_LENGTH 2                                                             /**< Length of handle inside Running Speed and Cadence Measurement packet. */

/**@brief Function for handling the @ref BLE_GATTS_EVT_WRITE event from the SoftDevice.
 *
 * @param[in] p_vband_srv     Voltage Band Service structure.
 * @param[in] p_ble_evt Pointer to the event received from BLE stack.
 */
static void on_write(ble_vband_srv_t * p_vband_srv, ble_evt_t const * p_ble_evt)
{
    ret_code_t                          err_code;
    ble_vband_srv_evt_t                 evt;
    ble_vband_srv_client_context_t    * p_client;
    ble_gatts_evt_write_t const * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;

    err_code = blcm_link_ctx_get(p_vband_srv->p_link_ctx_storage,
                                 p_ble_evt->evt.gatts_evt.conn_handle,
                                 (void *) &p_client);
    if (err_code != NRF_SUCCESS)
    {
        NRF_LOG_ERROR("Link context for 0x%02X connection handle could not be fetched.",
                      p_ble_evt->evt.gatts_evt.conn_handle);
    }

    memset(&evt, 0, sizeof(ble_vband_srv_evt_t));
    evt.p_vband_srv = p_vband_srv;
    evt.conn_handle = p_ble_evt->evt.gatts_evt.conn_handle;
    evt.p_link_ctx  = p_client;

    if ((p_evt_write->handle == p_vband_srv->alarm_state_handles.cccd_handle) &&
        (p_evt_write->len == 2))
    {
        if (ble_srv_is_notification_enabled(p_evt_write->data))
        {
            p_vband_srv->is_notification_supported_alarm = true;
        }
        else
        {
            p_vband_srv->is_notification_supported_alarm = false;
        }
    }
    else if ((p_evt_write->handle == p_vband_srv->accel_handles.cccd_handle) &&
        (p_evt_write->len == 2))
    {
        if (ble_srv_is_notification_enabled(p_evt_write->data))
        {
            p_vband_srv->is_notification_supported_accel_d = true;
        }
        else
        {
            p_vband_srv->is_notification_supported_accel_d = false;
        }
    }
    else if ((p_evt_write->handle == p_vband_srv->temp_humid_pressure_handles.cccd_handle) &&
        (p_evt_write->len == 2))
    {
        if (ble_srv_is_notification_enabled(p_evt_write->data))
        {
            p_vband_srv->is_notification_supported_thp_d = true;
        }
        else
        {
            p_vband_srv->is_notification_supported_thp_d = false;
        }
    }
    else if ((p_evt_write->handle == p_vband_srv->gas_sensor_handles.cccd_handle) &&
        (p_evt_write->len == 2))
    {
        if (ble_srv_is_notification_enabled(p_evt_write->data))
        {
            p_vband_srv->is_notification_supported_gas_d = true;
        }
        else
        {
            p_vband_srv->is_notification_supported_gas_d = false;
        }
    }
    else if ((p_evt_write->handle == p_vband_srv->optical_sensor_handles.cccd_handle) &&
        (p_evt_write->len == 2))
    {
        if (ble_srv_is_notification_enabled(p_evt_write->data))
        {
            p_vband_srv->is_notification_supported_opt_d = true;
        }
        else
        {
            p_vband_srv->is_notification_supported_opt_d = false;
        }
    }
    else if ((p_evt_write->handle == p_vband_srv->streaming_data_handles.cccd_handle) &&
        (p_evt_write->len == 2))
    {
        if (ble_srv_is_notification_enabled(p_evt_write->data))
        {
            p_vband_srv->is_notification_supported_stream_d = true;
        }
        else
        {
            p_vband_srv->is_notification_supported_stream_d = false;
        }
    }
    else if ((p_evt_write->handle == p_vband_srv->config_handles.value_handle) &&
             (p_vband_srv->evt_handler != NULL))
    {
        evt.type                               = BLE_VBAND_SRV_EVT_CONFIG_UPDATED;
        evt.params.config_data.op_mode         = *(p_evt_write->data);
        evt.params.config_data.alarm_threshold = uint16_decode(p_evt_write->data+1);
        p_vband_srv->evt_handler(&evt);
    }
    else
    {
        // Do Nothing. This event is not relevant for this service.
    }
}

static uint32_t update_characteristic(ble_vband_srv_t * p_vband_srv,
                                      uint8_t         * p_data,
                                      uint16_t        * p_length,
                                      uint16_t          conn_handle,
                                      uint16_t          value_handle,
                                      bool              is_notify)
{
    ret_code_t         err_code = NRF_SUCCESS;
    ble_gatts_value_t  gatts_value;

    // Initialize value struct.
    memset(&gatts_value, 0, sizeof(gatts_value));
    gatts_value.len     = *p_length;
    gatts_value.offset  = 0;
    gatts_value.p_value = p_data;

    // Update database.
    err_code = sd_ble_gatts_value_set(BLE_CONN_HANDLE_INVALID,
                                      value_handle,
                                      &gatts_value);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    // Send value if connected and notifying.
    if (is_notify)
    {
        ble_gatts_hvx_params_t hvx_params;
        memset(&hvx_params, 0, sizeof(hvx_params));
        hvx_params.handle = value_handle;
        hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
        hvx_params.offset = gatts_value.offset;
        hvx_params.p_len  = &gatts_value.len;
        hvx_params.p_data = gatts_value.p_value;

        return sd_ble_gatts_hvx(conn_handle, &hvx_params);
    }
    else
    {
        err_code = NRF_ERROR_INVALID_STATE;
    }
    return err_code;
}

uint32_t ble_vband_srv_init(ble_vband_srv_t * p_vband_srv, ble_vband_srv_init_t const * p_vband_srv_init)
{
    ret_code_t            err_code;
    ble_uuid_t            ble_uuid;
    ble_uuid128_t         vband_srv_base_uuid = VBAND_SERVICE_UUID_BASE;
    ble_add_char_params_t add_char_params;

    VERIFY_PARAM_NOT_NULL(p_vband_srv);
    VERIFY_PARAM_NOT_NULL(p_vband_srv_init);

    // Initialize the service structure.
    p_vband_srv->evt_handler = p_vband_srv_init->evt_handler;

    /**@snippet [Adding proprietary Service to the SoftDevice] */
    // Add a custom base UUID.
    err_code = sd_ble_uuid_vs_add(&vband_srv_base_uuid, &p_vband_srv->uuid_type);
    VERIFY_SUCCESS(err_code);

    ble_uuid.type = p_vband_srv->uuid_type;
    ble_uuid.uuid = BLE_UUID_VOLTAGE_WRISTBAND_SERVICE;

    // Add the service.
    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY,
                                        &ble_uuid,
                                        &p_vband_srv->service_handle);
    /**@snippet [Adding proprietary Service to the SoftDevice] */
    VERIFY_SUCCESS(err_code);

    // Add the Alarm State Characteristic.
    memset(&add_char_params, 0, sizeof(add_char_params));
    add_char_params.uuid                     = VOLTAGE_ALARM_STATE_CHAR_UUID;
    add_char_params.uuid_type                = p_vband_srv->uuid_type;
    add_char_params.max_len                  = BLE_VBAND_MAX_DATA_LEN;
    add_char_params.init_len                 = sizeof(uint8_t);
    add_char_params.is_var_len               = true;
    add_char_params.char_props.read          = 1;
    add_char_params.char_props.notify        = 1;

    add_char_params.read_access  = SEC_OPEN;
    add_char_params.write_access = SEC_NO_ACCESS;
    add_char_params.cccd_write_access = SEC_OPEN;

    err_code = characteristic_add(p_vband_srv->service_handle, &add_char_params, &p_vband_srv->alarm_state_handles);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    // Add the Configuration Characteristic.
    memset(&add_char_params, 0, sizeof(add_char_params));
    add_char_params.uuid                     = VOLTAGE_ALARM_CONFIG_CHAR_UUID;
    add_char_params.uuid_type                = p_vband_srv->uuid_type;
    add_char_params.max_len                  = BLE_VBAND_NORMAL_DATA_LEN;
    add_char_params.init_len                 = sizeof(uint8_t);
    add_char_params.is_var_len               = true;
    add_char_params.char_props.read          = 1;
    add_char_params.char_props.notify        = 1;
    add_char_params.char_props.write         = 1;
    add_char_params.char_props.write_wo_resp = 1;

    add_char_params.read_access  = SEC_OPEN;
    add_char_params.write_access = SEC_OPEN;
    add_char_params.cccd_write_access = SEC_OPEN;

    err_code = characteristic_add(p_vband_srv->service_handle, &add_char_params, &p_vband_srv->config_handles);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    // Add the Accelerometer Data Characteristic.
    memset(&add_char_params, 0, sizeof(add_char_params));
    add_char_params.uuid                     = ACCELEROMETER_DATA_CHAR_UUID;
    add_char_params.uuid_type                = p_vband_srv->uuid_type;
    add_char_params.max_len                  = BLE_VBAND_NORMAL_DATA_LEN;
    add_char_params.init_len                 = sizeof(uint8_t);
    add_char_params.is_var_len               = true;
    add_char_params.char_props.read          = 1;
    add_char_params.char_props.notify        = 1;

    add_char_params.read_access  = SEC_OPEN;
    add_char_params.write_access = SEC_NO_ACCESS;
    add_char_params.cccd_write_access = SEC_OPEN;

    err_code = characteristic_add(p_vband_srv->service_handle, &add_char_params, &p_vband_srv->accel_handles);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    // Add the Temp/Humid/Pressure Data Characteristic.
    memset(&add_char_params, 0, sizeof(add_char_params));
    add_char_params.uuid                     = TEMP_HUMID_PRESSURE_DATA_CHAR_UUID;
    add_char_params.uuid_type                = p_vband_srv->uuid_type;
    add_char_params.max_len                  = BLE_VBAND_NORMAL_DATA_LEN;
    add_char_params.init_len                 = sizeof(uint8_t);
    add_char_params.is_var_len               = true;
    add_char_params.char_props.read          = 1;
    add_char_params.char_props.notify        = 1;

    add_char_params.read_access  = SEC_OPEN;
    add_char_params.write_access = SEC_NO_ACCESS;
    add_char_params.cccd_write_access = SEC_OPEN;

    err_code = characteristic_add(p_vband_srv->service_handle, &add_char_params, &p_vband_srv->temp_humid_pressure_handles);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    // Add the Gas Sensor Data Characteristic.
    memset(&add_char_params, 0, sizeof(add_char_params));
    add_char_params.uuid                     = GAS_SENSOR_DATA_CHAR_UUID;
    add_char_params.uuid_type                = p_vband_srv->uuid_type;
    add_char_params.max_len                  = BLE_VBAND_NORMAL_DATA_LEN;
    add_char_params.init_len                 = sizeof(uint8_t);
    add_char_params.is_var_len               = true;
    add_char_params.char_props.read          = 1;
    add_char_params.char_props.notify        = 1;

    add_char_params.read_access  = SEC_OPEN;
    add_char_params.write_access = SEC_NO_ACCESS;
    add_char_params.cccd_write_access = SEC_OPEN;

    err_code = characteristic_add(p_vband_srv->service_handle, &add_char_params, &p_vband_srv->gas_sensor_handles);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    // Add the Optical Sensor Data Characteristic.
    memset(&add_char_params, 0, sizeof(add_char_params));
    add_char_params.uuid                     = OPTICAL_SENSOR_DATA_CHAR_UUID;
    add_char_params.uuid_type                = p_vband_srv->uuid_type;
    add_char_params.max_len                  = BLE_VBAND_NORMAL_DATA_LEN;
    add_char_params.init_len                 = sizeof(uint8_t);
    add_char_params.is_var_len               = true;
    add_char_params.char_props.read          = 1;
    add_char_params.char_props.notify        = 1;

    add_char_params.read_access  = SEC_OPEN;
    add_char_params.write_access = SEC_NO_ACCESS;
    add_char_params.cccd_write_access = SEC_OPEN;

    err_code = characteristic_add(p_vband_srv->service_handle, &add_char_params, &p_vband_srv->optical_sensor_handles);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    // Add the Gas Sensor Data Characteristic.
    memset(&add_char_params, 0, sizeof(add_char_params));
    add_char_params.uuid                     = STREAMING_DATA_CHAR_UUID;
    add_char_params.uuid_type                = p_vband_srv->uuid_type;
    add_char_params.max_len                  = BLE_VBAND_MAX_DATA_LEN;
    add_char_params.init_len                 = sizeof(uint8_t);
    add_char_params.is_var_len               = true;
    add_char_params.char_props.read          = 1;
    add_char_params.char_props.notify        = 1;

    add_char_params.read_access  = SEC_OPEN;
    add_char_params.write_access = SEC_NO_ACCESS;
    add_char_params.cccd_write_access = SEC_OPEN;

    err_code = characteristic_add(p_vband_srv->service_handle, &add_char_params, &p_vband_srv->streaming_data_handles);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
}


void ble_vband_srv_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context)
{
    if ((p_context == NULL) || (p_ble_evt == NULL))
    {
        return;
    }

    ble_vband_srv_t * p_vband_srv = (ble_vband_srv_t *)p_context;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GATTS_EVT_WRITE:
            on_write(p_vband_srv, p_ble_evt);
            break;

        default:
            // No implementation needed.
            break;
    }
}


uint32_t ble_vband_srv_alarm_update(ble_vband_srv_t * p_vband_srv,
                                    uint8_t         * p_data,
                                    uint16_t        * p_length,
                                    uint16_t          conn_handle)
{
    if (p_vband_srv == NULL)
    {
        return NRF_ERROR_NULL;
    }

    if (*p_length > BLE_VBAND_MAX_DATA_LEN)
    {
        return NRF_ERROR_INVALID_PARAM;
    }

    // generic characteristic update call
    return update_characteristic(p_vband_srv, p_data, p_length, conn_handle, p_vband_srv->alarm_state_handles.value_handle, p_vband_srv->is_notification_supported_alarm);
}


uint32_t ble_vband_srv_config_update(ble_vband_srv_t * p_vband_srv,
                                     uint8_t         * p_data,
                                     uint16_t        * p_length,
                                     uint16_t          conn_handle)
{
    if (p_vband_srv == NULL)
    {
        return NRF_ERROR_NULL;
    }

    if (*p_length > BLE_VBAND_MAX_DATA_LEN)
    {
        return NRF_ERROR_INVALID_PARAM;
    }

    // generic characteristic update call
    return update_characteristic(p_vband_srv, p_data, p_length, conn_handle, p_vband_srv->config_handles.value_handle, false);
}


uint32_t ble_vband_srv_accelerometer_update(ble_vband_srv_t * p_vband_srv,
                                            uint8_t         * p_data,
                                            uint16_t        * p_length,
                                            uint16_t          conn_handle)
{
    if (p_vband_srv == NULL)
    {
        return NRF_ERROR_NULL;
    }

    if (*p_length > BLE_VBAND_MAX_DATA_LEN)
    {
        return NRF_ERROR_INVALID_PARAM;
    }

    // generic characteristic update call
    return update_characteristic(p_vband_srv, p_data, p_length, conn_handle, p_vband_srv->accel_handles.value_handle, p_vband_srv->is_notification_supported_accel_d);
}


uint32_t ble_vband_srv_temp_humid_pressure_update(ble_vband_srv_t * p_vband_srv,
                                                  uint8_t         * p_data,
                                                  uint16_t        * p_length,
                                                  uint16_t          conn_handle)
{
    if (p_vband_srv == NULL)
    {
        return NRF_ERROR_NULL;
    }

    if (*p_length > BLE_VBAND_MAX_DATA_LEN)
    {
        return NRF_ERROR_INVALID_PARAM;
    }

    // generic characteristic update call
    return update_characteristic(p_vband_srv, p_data, p_length, conn_handle, p_vband_srv->temp_humid_pressure_handles.value_handle, p_vband_srv->is_notification_supported_thp_d);
}


uint32_t ble_vband_srv_gas_sensor_update(ble_vband_srv_t * p_vband_srv,
                                         uint8_t         * p_data,
                                         uint16_t        * p_length,
                                         uint16_t          conn_handle)
{
    if (p_vband_srv == NULL)
    {
        return NRF_ERROR_NULL;
    }

    if (*p_length > BLE_VBAND_MAX_DATA_LEN)
    {
        return NRF_ERROR_INVALID_PARAM;
    }

    // generic characteristic update call
    return update_characteristic(p_vband_srv, p_data, p_length, conn_handle, p_vband_srv->gas_sensor_handles.value_handle, p_vband_srv->is_notification_supported_gas_d);
}


uint32_t ble_vband_srv_optical_sensor_update(ble_vband_srv_t * p_vband_srv,
                                             uint8_t         * p_data,
                                             uint16_t        * p_length,
                                             uint16_t          conn_handle)
{
    if (p_vband_srv == NULL)
    {
        return NRF_ERROR_NULL;
    }

    if (*p_length > BLE_VBAND_MAX_DATA_LEN)
    {
        return NRF_ERROR_INVALID_PARAM;
    }

    // generic characteristic update call
    return update_characteristic(p_vband_srv, p_data, p_length, conn_handle, p_vband_srv->optical_sensor_handles.value_handle, p_vband_srv->is_notification_supported_opt_d);
}


uint32_t ble_vband_srv_streaming_data_update(ble_vband_srv_t * p_vband_srv,
                                             uint8_t         * p_data,
                                             uint16_t        * p_length,
                                             uint16_t          conn_handle)
{
    if (p_vband_srv == NULL)
    {
        return NRF_ERROR_NULL;
    }

    if (*p_length > BLE_VBAND_MAX_DATA_LEN)
    {
        return NRF_ERROR_INVALID_PARAM;
    }

    // generic characteristic update call
    return update_characteristic(p_vband_srv, p_data, p_length, conn_handle, p_vband_srv->streaming_data_handles.value_handle, p_vband_srv->is_notification_supported_stream_d);
}
                                                     
