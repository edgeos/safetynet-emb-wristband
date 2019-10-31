// CCS811 Air quality sensor

#include "nrf_drv_twi.h"

#ifndef CCS811_H
#define CCS811_H

#define CCS811_ADDR                  0x5A // if ADDR is gnd
//#define CCS811_ADDR                  0x5B // if ADDR is VDD
#define CCS811_REG_STATUS            0x00
#define CCS811_REG_MEAS_MODE         0x01
#define CCS811_REG_ALG_RESULTS_DATA  0x02
#define CCS811_REG_RAW_DATA          0x03
#define CCS811_REG_ENV_DATA          0x05
#define CCS811_REG_NTC               0x06
#define CCS811_REG_THRESHOLDS        0x10
#define CCS811_REG_BASELINE          0x11
#define CCS811_REG_HW_ID             0x20
#define CCS811_REG_HW_VERSION        0x21
#define CCS811_REG_ERROR_ID          0xE0
#define CCS811_REG_SW_RESET          0xFF
#define CCS811_REG_APP_ERASE         0xF1
#define CCS811_REG_APP_DATA          0xF2
#define CCS811_REG_APP_VERIFY        0xF3
#define CCS811_REG_APP_START         0xF4

#define CCS811_HW_ID                 0x81
#define CCS811_HW_VERSION            0x10

/*!
 * @brief Type definitions
 */
typedef int8_t (*ccs811_com_fptr_t)(uint8_t dev_id, uint8_t reg_addr,
		uint8_t *data, uint16_t len);

/*!
 * @brief ccs811 device structure
 */
struct ccs811_dev {
	/*! Chip Id */
	uint8_t chip_id;
	/*! Device Id */
	uint8_t dev_id;
	/*! Read function pointer */
	ccs811_com_fptr_t read;
	/*! Write function pointer */
	ccs811_com_fptr_t write;
};

/**@brief I2C enable on - WAKEUP pin control */
void ccs811_enable(void);

/**@brief I2C enable off - WAKEUP pin control*/
void ccs811_disable(void);

/**@brief reset if error found */
void ccs811_reset(struct ccs811_dev *dev);

/**@brief full init sequence */
bool ccs811_init(struct ccs811_dev *dev);

/**@brief APP_START with mode */
bool ccs811_start_mode(struct ccs811_dev *dev, uint8_t mode);

/**@brief full calibration sequence */
void ccs811_calibrate(struct ccs811_dev *dev, int16_t temperature_c, uint8_t humidity_pct);

/**@brief read measurement, return true if new data */
bool ccs811_measure(struct ccs811_dev *dev, uint16_t *eCO2, uint16_t *TVOC, uint8_t* pError);

/**@brief shutdown */
bool ccs811_idle(struct ccs811_dev *dev);

#endif // CCS811_H
