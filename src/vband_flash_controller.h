#ifndef VBAND_FLASH_CONTROLLER_H__
#define VBAND_FLASH_CONTROLLER_H__

#include <stdint.h>
#include "nrf.h"

#ifdef __cplusplus
extern "C" {
#endif

#define BLE_NAME_ADDRESS                (0x3e000)
#define ALARM_THRESHOLD_ADDRESS         (0x3e100)
#define ACTIVE_MODE_ADDRESS             (0x3e200)

#define FLASHWRITE_BLOCK_VALID          (0xA55A5AA5)
#define FLASHWRITE_BLOCK_INVALID        (0xA55A0000)
#define FLASHWRITE_BLOCK_NOT_INIT       (0xFFFFFFFF)

#define MAX_BLE_NAME_LENGTH  16

#pragma pack(1)

/**@brief Coonfiguration Parameter Type */
typedef enum
{
    BLE_NAME        = 1,
    ALARM_THRESHOLD = 2,
    DEFAULT_MODE    = 3
} config_param_type_t;

typedef struct
{
    uint32_t magic_number;
    config_param_type_t config;
    uint8_t name[MAX_BLE_NAME_LENGTH];
    uint8_t pad[3]; // flash writes must be in size (in bytes) that is multiple of 4
} ble_name_param_t;

typedef struct
{
    uint32_t magic_number;
    config_param_type_t config;
    float threshold;
    uint8_t pad[3]; // flash writes must be in size (in bytes) that is multiple of 4
} alarm_threshold_param_t;

typedef struct
{
    uint32_t magic_number;
    config_param_type_t config;
    uint8_t mode;
    uint8_t pad[2]; // flash writes must be in size (in bytes) that is multiple of 4
} active_mode_param_t;

void initialize_flash(void);

bool read_flash_ble_advertisement_name(uint8_t * buf);

bool read_flash_alarm_threshold(float * buf);

bool read_flash_active_mode(uint8_t * buf);

void write_flash_ble_advertisement_name(uint8_t * buf, uint8_t len);

void write_flash_alarm_threshold(float * buf);

void write_flash_active_mode(uint8_t * buf);

#ifdef __cplusplus
}
#endif

#endif /* VBAND_FLASH_CONTROLLER_H__ */