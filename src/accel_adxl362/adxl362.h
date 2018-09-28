#include <stdint.h>
#include "nrf_drv_spi.h"

#define REG_DEVICE_ID_1    0x00
#define REG_DEVICE_ID_2    0x01
#define REG_PART_ID        0x02
#define REG_XDATA_L        0x0E
#define REG_YDATA_L        0x10
#define REG_ZDATA_L        0x12
#define REG_THRESH_ACT_L   0x20
#define REG_THRESH_ACT_H   0x21
#define REG_TIME_ACT       0x22
#define REG_THRESH_INACT_L 0x23
#define REG_THRESH_INACT_H 0x24
#define REG_TIME_INACT_L   0x25
#define REG_TIME_INACT_H   0x26
#define REG_ACT_INACT_CTL  0x27
#define REG_FIFO_CONTROL   0x28
#define REG_FIFO_SAMPLES   0x29
#define REG_INTMAP1        0x2A
#define REG_INTMAP2        0x2B
#define REG_FILTER_CTL     0x2C
#define REG_POWER_CTL      0x2D
#define REG_SELF_TEST      0x2E

#define DEVICE_ID_1       0xAD
#define DEVICE_ID_2       0x1D
#define PART_ID           0xF2

#define CMD_READ          0x0B
#define CMD_WRITE         0x0A

/**@brief chip enable on */
void adxl362_enable(void);

/**@brief chip enable off */
void adxl362_disable(void);

/**@brief full init sequence */
bool adxl362_init(void);

/**@brief Function for reading to the ADXL362 registers over SPI. */
void adxl362_readRegister(uint8_t address, uint8_t bytes);

/**@brief Function for writing to the ADXL362 registers over SPI. */
void adxl362_writeRegister(uint8_t address, uint8_t bytes);

/**@brief Function for verfiying the presence of the ADXL362 . */
bool adxl362_verifyChip(void);

/**@brief Function for configuring the movement detections settings of the ADXL362 . */
void adxl362_configTrigger(void);

/**@brief Function for configuring interrupt handlers for the ADXL362 . */
void adxl362_configIntPins(void);
 
void adxl362_readAccelerometerData(int16_t *x_val, int16_t *y_val, int16_t *z_val);



