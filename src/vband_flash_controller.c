#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "nrf_delay.h"
#include "nrf_fstorage.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "nrf_fstorage_sd.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "vband_flash_controller.h"

static void fstorage_evt_handler(nrf_fstorage_evt_t * p_evt);

NRF_FSTORAGE_DEF(nrf_fstorage_t fstorage) =
{
    /* Set a handler for fstorage events. */
    .evt_handler = fstorage_evt_handler,

    /* These below are the boundaries of the flash space assigned to this instance of fstorage.
     * You must set these manually, even at runtime, before nrf_fstorage_init() is called.
     * The function nrf5_flash_end_addr_get() can be used to retrieve the last address on the
     * last page of flash available to write data. */
    .start_addr = 0xfe000,
    .end_addr   = 0xfffff,
};
nrf_fstorage_api_t * p_fs_api;

static ble_name_param_t m_ble_name = {0};
static alarm_threshold_param_t m_alarm_threshold = {0};
static active_mode_param_t m_active_mode = {0};

static void fstorage_evt_handler(nrf_fstorage_evt_t * p_evt)
{
    if (p_evt->result != NRF_SUCCESS)
    {
        NRF_LOG_INFO("--> Event received: ERROR while executing an fstorage operation.");
        return;
    }

    switch (p_evt->id)
    {
        case NRF_FSTORAGE_EVT_WRITE_RESULT:
        {
            NRF_LOG_INFO("--> Event received: wrote %d bytes at address 0x%x.",
                         p_evt->len, p_evt->addr);
        } break;

        case NRF_FSTORAGE_EVT_ERASE_RESULT:
        {
            NRF_LOG_INFO("--> Event received: erased %d page from address 0x%x.",
                         p_evt->len, p_evt->addr);
        } break;

        default:
            break;
    }
}

static void wait_for_flash_ready(nrf_fstorage_t const * p_fstorage)
{
    /* While fstorage is busy, sleep and wait for an event. */
    while (nrf_fstorage_is_busy(p_fstorage)){}
}

/**@brief   Helper function to obtain the last address on the last page of the on-chip flash that
 *          can be used to write user data.
 */
static uint32_t nrf5_flash_end_addr_get()
{
    uint32_t const bootloader_addr = NRF_UICR->NRFFW[0];
    uint32_t const page_sz         = NRF_FICR->CODEPAGESIZE;
    uint32_t const code_sz         = NRF_FICR->CODESIZE;

    return (bootloader_addr != 0xFFFFFFFF ?
            bootloader_addr : (code_sz * page_sz));
}

static void print_flash_info(nrf_fstorage_t * p_fstorage)
{
    NRF_LOG_INFO("========| flash info |========");
    NRF_LOG_INFO("erase unit: \t%d bytes",      p_fstorage->p_flash_info->erase_unit);
    NRF_LOG_INFO("program unit: \t%d bytes",    p_fstorage->p_flash_info->program_unit);
    NRF_LOG_INFO("==============================");
}

void initialize_flash(void)
{
    ret_code_t rc;

    // initialize flash
    p_fs_api = &nrf_fstorage_sd;
    rc = nrf_fstorage_init(&fstorage, p_fs_api, NULL);
    APP_ERROR_CHECK(rc);

    print_flash_info(&fstorage);

    /* It is possible to set the start and end addresses of an fstorage instance at runtime.
     * They can be set multiple times, should it be needed. The helper function below can
     * be used to determine the last address on the last page of flash memory available to
     * store data. */
    (void) nrf5_flash_end_addr_get();

    rc = nrf_fstorage_read(&fstorage, BLE_NAME_ADDRESS, &m_ble_name, sizeof(m_ble_name));
    APP_ERROR_CHECK(rc);
    wait_for_flash_ready(&fstorage);

    rc = nrf_fstorage_read(&fstorage, ALARM_THRESHOLD_ADDRESS, &m_alarm_threshold, sizeof(m_alarm_threshold));
    APP_ERROR_CHECK(rc);
    wait_for_flash_ready(&fstorage);

    rc = nrf_fstorage_read(&fstorage, ACTIVE_MODE_ADDRESS, &m_active_mode, sizeof(m_active_mode));
    APP_ERROR_CHECK(rc);
    wait_for_flash_ready(&fstorage);
}

bool read_flash_ble_advertisement_name(uint8_t * buf)
{
    memcpy(buf, &m_ble_name.name, MAX_BLE_NAME_LENGTH);
    return (m_ble_name.magic_number == FLASHWRITE_BLOCK_VALID) ? true : false;
}

bool read_flash_alarm_threshold(float * buf)
{
    memcpy(buf, &m_alarm_threshold.threshold, sizeof(float));
    return (m_alarm_threshold.magic_number == FLASHWRITE_BLOCK_VALID) ? true : false;
}

bool read_flash_active_mode(uint8_t * buf)
{
    *buf = m_active_mode.mode;
    return (m_active_mode.magic_number == FLASHWRITE_BLOCK_VALID) ? true : false;
}

void write_flash_ble_advertisement_name(uint8_t * buf, uint8_t len)
{
    ret_code_t rc;

    memcpy(&m_ble_name.name, buf, len);

    // add spaces at end
    uint8_t pad_len = MAX_BLE_NAME_LENGTH - len;
    if (pad_len > 0)
    {
        memset(&m_ble_name.name[len], 32, pad_len); // 32 = space decimal code for ASCII
    }

    m_ble_name.magic_number = FLASHWRITE_BLOCK_VALID;
    m_ble_name.config = BLE_NAME;

    // write to flash, need to erase first
    /*rc = nrf_fstorage_erase(&fstorage, BLE_NAME_ADDRESS, sizeof(m_ble_name), NULL);
    APP_ERROR_CHECK(rc);
    nrf_delay_ms(1);*/
    rc = nrf_fstorage_write(&fstorage, BLE_NAME_ADDRESS, &m_ble_name, sizeof(m_ble_name), NULL);
    APP_ERROR_CHECK(rc);
    //wait_for_flash_ready(&fstorage);
}

void write_flash_alarm_threshold(float * buf)
{
    ret_code_t rc;

    memcpy(&m_alarm_threshold.threshold, buf, sizeof(float));
    m_alarm_threshold.magic_number = FLASHWRITE_BLOCK_VALID;
    m_alarm_threshold.config = ALARM_THRESHOLD;

    // write to flash, need to erase first
    /*rc = nrf_fstorage_erase(&fstorage, ALARM_THRESHOLD_ADDRESS, sizeof(m_alarm_threshold), NULL);
    APP_ERROR_CHECK(rc);
    nrf_delay_ms(1);*/
    rc = nrf_fstorage_write(&fstorage, ALARM_THRESHOLD_ADDRESS, &m_alarm_threshold, sizeof(m_alarm_threshold), NULL);
    APP_ERROR_CHECK(rc);
    //wait_for_flash_ready(&fstorage);
}

void write_flash_active_mode(uint8_t * buf)
{
    ret_code_t rc;

    m_active_mode.mode = *buf;
    m_active_mode.magic_number = FLASHWRITE_BLOCK_VALID;
    m_active_mode.config = DEFAULT_MODE;

    // write to flash, need to erase first
    /*rc = nrf_fstorage_erase(&fstorage, ACTIVE_MODE_ADDRESS, sizeof(m_active_mode), NULL);
    APP_ERROR_CHECK(rc);
    nrf_delay_ms(1);*/
    rc = nrf_fstorage_write(&fstorage, ACTIVE_MODE_ADDRESS, &m_active_mode, sizeof(m_active_mode), NULL);
    APP_ERROR_CHECK(rc);
    //wait_for_flash_ready(&fstorage);
}