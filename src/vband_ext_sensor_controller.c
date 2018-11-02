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
#include "boards.h"
#include "app_util_platform.h"
#include "app_error.h"
#include "nrf_drv_twi.h"
#include "nrf_delay.h"
#include "ccs811.h"
#include "bme280.h"
#include "ADXL362.h"
#include "MAX30105.h"
#include "vband_ext_sensor_controller.h"
#include "heart_rate_detector.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

// prox threshold for MAX30105
#define PROX_THRESHOLD                      10

#ifndef SCL_PIN
#define SCL_PIN             NRF_GPIO_PIN_MAP(0,26)    // SCL signal pin
#define SDA_PIN             NRF_GPIO_PIN_MAP(1,15)    // SDA signal pin
#define WAKEUP_PIN          NRF_GPIO_PIN_MAP(1,8)     // Wakeup signal pin
#define INT_PIN             NRF_GPIO_PIN_MAP(1,7)     // Data ready (active low)
#endif

static bool simulate_on = false;

struct bme280_dev i2c_dev_bme280;
struct ccs811_dev i2c_dev_ccs811;
struct max30105_dev i2c_dev_max301015;

// to track which sensors are enabled / connected
static uint16_t enabled_sensors = 0;

/* TWI instance. */
static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(0);
/* Busy bool for I2C */
static volatile bool m_xfer_done1 = false;
/* I2C enable bool */
static volatile bool twi_enabled = false;

// track Optical sensor state
static max30105_state_t max301015_state = PRESENCE_NOT_DETECTED;
static uint32_t hr_ticks = 0;

/**
 * @brief TWI events handler.
 */
static void twi_handler(nrf_drv_twi_evt_t const * p_event, void * p_context)
{
    switch (p_event->type)
    {
        case NRF_DRV_TWI_EVT_DONE:
            if (p_event->xfer_desc.type == NRF_DRV_TWI_XFER_RX)
            {
                // do nothing for now
            }
            m_xfer_done1 = true;
            break;
        default:
            break;
    }
}

static void twi_wait(void)
{
    uint16_t ctr = 0;
    uint16_t limit_us = 500*64;
    do
    {
        //nrf_delay_us(1);
        ctr++;
    }while((m_xfer_done1 == false) & (ctr < limit_us));
}


/**
 * @brief TWI initialization.
 */
static void twi_init(void)
{
    ret_code_t err_code;

    const nrf_drv_twi_config_t twi_config = {
       .scl                = SCL_PIN,
       .sda                = SDA_PIN,
       .frequency          = NRF_TWI_FREQ_400K,
       .interrupt_priority = APP_IRQ_PRIORITY_LOW,
       .clear_bus_init     = true//,
       //.hold_bus_uninit    = true 
    };

    err_code = nrf_drv_twi_init(&m_twi, &twi_config, twi_handler, NULL);
    APP_ERROR_CHECK(err_code);

    nrf_drv_twi_enable(&m_twi);

    twi_enabled = true;
}

static int8_t twi_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
    ret_code_t err_code;

    /* Prepare for the read with a register write */
    uint8_t reg_read[2] = {reg_addr, 0};
    m_xfer_done1 = false; 
    err_code = nrf_drv_twi_tx(&m_twi, dev_id, reg_read, 1, false);
    APP_ERROR_CHECK(err_code);
    twi_wait();

    /* Now read the register */
    m_xfer_done1 = false; 
    err_code = nrf_drv_twi_rx(&m_twi, dev_id, data, len);
    APP_ERROR_CHECK(err_code);
    twi_wait();

    return err_code;
}

static int8_t twi_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
    ret_code_t err_code;
    
    /* Build tx buffer */
    uint8_t reg_write[20];
    reg_write[0] = reg_addr;
    if(data) memcpy(&reg_write[1],data,len);

    /* Transfer tx buffer */
    m_xfer_done1 = false;
    err_code = nrf_drv_twi_tx(&m_twi, dev_id, reg_write, 1+len, false);
    APP_ERROR_CHECK(err_code);
    twi_wait();

    return err_code;
}

static bool scan_for_twi_device(uint8_t dev_address)
{
    ret_code_t err_code;

    uint8_t sample_data;
    err_code = nrf_drv_twi_rx(&m_twi, dev_address, &sample_data, sizeof(sample_data));
    twi_wait();
    nrf_delay_ms(10);
    return (err_code == NRF_SUCCESS) ? true:false;
}

static void print_sensor_data(struct bme280_data *comp_data)
{
#ifdef BME280_FLOAT_ENABLE
        NRF_LOG_INFO("%0.2f, %0.2f, %0.2f\r\n",comp_data->temperature, comp_data->pressure, comp_data->humidity);
#else
        //NRF_LOG_INFO("\n")
        NRF_LOG_INFO("Temp:  %ld, Humid:  %ld, Pressure:  %ld  \r",comp_data->temperature, comp_data->humidity, comp_data->pressure);
#endif
}

static void sensor_bme280_init(void)
{
    i2c_dev_bme280.dev_id = BME280_I2C_ADDR_SEC;
    i2c_dev_bme280.intf = BME280_I2C_INTF;
    i2c_dev_bme280.read = twi_read;
    i2c_dev_bme280.write = twi_write;
    i2c_dev_bme280.delay_ms = nrf_delay_ms;
    if(scan_for_twi_device(BME280_I2C_ADDR_SEC))
    {
        if (bme280_init(&i2c_dev_bme280) == BME280_OK)
        {
            enabled_sensors |= BME280;
        }
    }
    else
    {
        NRF_LOG_WARNING("BME280 not found! Skipping...");
    }
}

static int8_t bme280_stream_sensor_data_forced_mode(struct bme280_dev *dev, uint8_t * p_data, uint16_t * p_data_length)
{
    static uint8_t data_buffer[20];
    static uint16_t data_buffer_length;

    int8_t rslt;
    uint8_t settings_sel;
    static struct bme280_data comp_data = {0};
    static bool prepare_measurement = true; // because of the delay for the measurement to complete we toggle what we do

    if (!simulate_on)
    {
        /* Recommended mode of operation: Humidity Sensing (p17 of datasheet) */
        dev->settings.osr_h = BME280_OVERSAMPLING_1X;
        dev->settings.osr_p = BME280_NO_OVERSAMPLING;
        dev->settings.osr_t = BME280_OVERSAMPLING_1X;
        dev->settings.filter = BME280_FILTER_COEFF_OFF;
        settings_sel = BME280_OSR_TEMP_SEL | BME280_OSR_HUM_SEL;
        rslt = bme280_set_sensor_settings(settings_sel, dev);

        /* Ping for sensor data */
        rslt = bme280_set_sensor_mode(BME280_FORCED_MODE, dev);
        nrf_delay_ms(20); // measurement time
        rslt = bme280_get_sensor_data(BME280_ALL, &comp_data, dev);
    }
    else
    {
        comp_data.temperature++;
        comp_data.humidity++;
        comp_data.pressure++;
    }
    print_sensor_data(&comp_data);

    // package for BLE
    memcpy(&data_buffer[0], &comp_data.temperature, sizeof(comp_data.temperature));
    memcpy(&data_buffer[2], &comp_data.humidity, sizeof(comp_data.humidity));
    memcpy(&data_buffer[4], &comp_data.pressure, sizeof(comp_data.pressure));
    data_buffer_length = 6;

    // assign pointers
    memcpy(p_data, &data_buffer[0], data_buffer_length);
    *p_data_length = data_buffer_length;

    return rslt;
}

static void max30105_setup_proximity_sensing(void)
{
    MAX30105_softReset(); // reset the MAX30105
    MAX30105_shutDown();  // shut down while configuring
    MAX30105_enableIntr(INTR_PROX);  // enable proximity interrupt
    MAX30105_setProx(0x40, PROX_THRESHOLD);    // set proximity pulse amplitude and threshold
    MAX30105_setSingleLED(SMP_AVE_1, false, FIFO_A_FULL_F, ADC_RGE_01, SMP_RT_100, LED_PW_18BIT, MAX30105_DEFAULT_LED_PA);  // configure single LED mode to initiate proximity detection
    MAX30105_wakeUp();        // exit shutdown to start sensing
}

static void max30105_setup_heart_rate_sensing(void)
{
    MAX30105_softReset(); // reset the MAX30105
    MAX30105_shutDown();  // shut down while configuring
    MAX30105_enableIntr(INTR_DATA_RDY | INTR_PROX);  // enable proximity interrupt
    MAX30105_setProx(0x40, PROX_THRESHOLD);    // set proximity pulse amplitude and threshold
    MAX30105_setDualLED(SMP_AVE_4, true, FIFO_A_FULL_F, ADC_RGE_01, SMP_RT_200, LED_PW_18BIT, MAX30105_HR_SENSE_RED_PA, MAX30105_HR_SENSE_IR_PA);
    MAX30105_clearFIFO(); // clear FIFO before sampling start
    MAX30105_wakeUp();        // exit shutdown to start sensing
}

static void sensor_max30105_init(void) 
{
    i2c_dev_max301015.dev_id = MAX30105_I2C_ADDR;
    i2c_dev_max301015.read = twi_read;
    i2c_dev_max301015.write = twi_write;
    if (scan_for_twi_device(MAX30105_I2C_ADDR)) 
    {
        if (MAX30105_init(&i2c_dev_max301015) == MAX30105_NO_ERROR) 
        {
            enabled_sensors |= MAX30105;
            max30105_setup_proximity_sensing();
        }
    } 
    else 
    {
        NRF_LOG_WARNING("MAX30105 not found! Skipping...");
    }
}

static max30105_state_t get_max30105_presence(void)
{
    static max30105_state_t prox_detected = PRESENCE_NOT_DETECTED;
    if (MAX30105_getIntr1() & INTR_PROX) // if the proximity interrupt occurs
    { 
        if (!prox_detected) 
        {
            NRF_LOG_INFO("Proximity detected!");
        }
        MAX30105_writeReg(REG_MODE_CONFIG, MODE_1LED); // go back into proximity detection
        prox_detected = PRESENCE_DETECTED;
    } 
    else 
    {
        if (prox_detected) 
        {
          NRF_LOG_INFO("Proximity not detected!");
        }
        prox_detected = PRESENCE_NOT_DETECTED;
    }
    return prox_detected;
}

static void get_max30105_heart_rate(uint8_t *heart_rate)
{
    uint32_t redLED = 0;
    uint32_t irLED = 0;
    if (MAX30105_getIntr1() & INTR_DATA_RDY) // if the proximity interrupt occurs
    {
        MAX30105_readFIFO(&redLED, &irLED);
        update_beat_timer(++hr_ticks, 20);
        get_hr_bpm(heart_rate, &irLED);
        NRF_LOG_INFO("red LED:  %ld, IR LED:  %ld, Avg BPM:  %d, Time:   %ld \r", redLED, irLED, *heart_rate, hr_ticks*20);
    }
}

static void get_max30105_status(uint8_t * p_data, uint16_t * p_data_length)
{
    static uint8_t data_buffer[20];
    static uint16_t data_buffer_length;
    static uint8_t heart_rate = 0;

    if (!simulate_on)
    {
        max301015_state = get_max30105_presence();
        /*
        if(max301015_state == PRESENCE_NOT_DETECTED)
        {
            max301015_state = get_max30105_presence();
            if(max301015_state == PRESENCE_DETECTED)
            {
                max30105_setup_heart_rate_sensing();
                hr_ticks = 0;
                reset_beat_timer();
            }
        }
        else
        {
           get_max30105_heart_rate(&heart_rate);
           //max301015_state = get_max30105_presence();
           if(max301015_state == PRESENCE_NOT_DETECTED)
           {
                max30105_setup_proximity_sensing();
           }
        }*/
    }
    else
    {
        max301015_state = !max301015_state;
    }

    // package for BLE
    memcpy(&data_buffer[0], &max301015_state, sizeof(max301015_state));
    memcpy(&data_buffer[1], &heart_rate, sizeof(heart_rate));
    data_buffer_length = sizeof(max301015_state) + sizeof(heart_rate);

    // assign pointers
    memcpy(p_data, &data_buffer[0], data_buffer_length);
    *p_data_length = data_buffer_length;
}

static void sensor_ccs811_init(void)
{
    i2c_dev_ccs811.dev_id = CCS811_ADDR;
    i2c_dev_ccs811.read = twi_read;
    i2c_dev_ccs811.write = twi_write;
    if (scan_for_twi_device(CCS811_ADDR))
    {
        if(ccs811_init(&i2c_dev_ccs811) == true)
        {
            enabled_sensors |= CCS811;
            nrf_delay_ms(5);
            ccs811_start_mode(&i2c_dev_ccs811, 0x10);
        }
    }
    else
    {
        NRF_LOG_WARNING("CCS811 not found! Skipping...");
    }
}

static void get_ccs811_measurement(uint8_t * p_data, uint16_t * p_data_length)
{
    static uint8_t data_buffer[20];
    static uint16_t data_buffer_length;
    
    static uint16_t eCO2 = 0;
    static uint16_t TVOC = 0;
    if (!simulate_on)
    {
        ccs811_measure(&i2c_dev_ccs811, &eCO2, &TVOC);
    }
    {
        eCO2++;
        TVOC++;
    }
    NRF_LOG_INFO("CO2:  %d  , TVOC:  %d  \r", eCO2, TVOC);

    // package for BLE
    memcpy(&data_buffer[0], &eCO2, sizeof(eCO2));
    memcpy(&data_buffer[2], &TVOC, sizeof(TVOC));
    data_buffer_length = 4;

    // assign pointers
    memcpy(p_data, &data_buffer[0], data_buffer_length);
    *p_data_length = data_buffer_length;
}

static void sensor_adxl362_init(void)
{
    if (ADXL362_Init() == 1) 
    {
        enabled_sensors |= ADXL362;
        ADXL362_SetPowerMode(1);
        ADXL362_SetRange(ADXL362_RANGE_2G);
        ADXL362_SetOutputRate(ADXL362_ODR_12_5_HZ);
    }
    else
    {
        NRF_LOG_WARNING("ADXL362 not found! Skipping...");
    }
}

static void get_adxl362_measurement(uint8_t * p_data, uint16_t * p_data_length)
{
    static uint8_t data_buffer[20];
    static uint16_t data_buffer_length;

    static int16_t xdata = 0;
    static int16_t ydata = 0;
    static int16_t zdata = 0;
    if (!simulate_on)
    {
        ADXL362_GetXyz(&xdata,&ydata,&zdata);
    }
    else
    {
        xdata++;
        ydata++;
        zdata++;
    }

    NRF_LOG_INFO("x:  %d  , y:  %d  , z:  %d\r", xdata, ydata, zdata);

    // package for BLE
    memcpy(&data_buffer[0], &xdata, sizeof(xdata));
    memcpy(&data_buffer[2], &ydata, sizeof(ydata));
    memcpy(&data_buffer[4], &zdata, sizeof(zdata));
    data_buffer_length = 6;

    // assign pointers
    memcpy(p_data, &data_buffer[0], data_buffer_length);
    *p_data_length = data_buffer_length;
}


void vband_sensor_init(sensor_type_t use_sensors)
{
    int8_t rslt_bme280 = BME280_OK;
    int8_t rslt_max30105 = BME280_OK;
    
    if (use_sensors & SIMULATE)
    {
        simulate_on = true;
    }
    else
    {
        twi_init();
        if(use_sensors & MAX30105)
        {
            sensor_max30105_init();
        }
        if(use_sensors & CCS811)
        {     
            sensor_ccs811_init();
        }
        if(use_sensors & BME280)
        {  
            sensor_bme280_init();
        }   
        if(use_sensors & ADXL362)
        {
            sensor_adxl362_init();
        }
    }
}

int8_t get_sensor_data(sensor_type_t get_sensor, uint8_t * p_data, uint16_t * p_data_length)
{
    int8_t ret_code = 0;

    // check if sensor is enabled
    if (!simulate_on)
    {
        if (!(enabled_sensors & get_sensor)) return 1;
    }

    // query by sensor type
    switch(get_sensor) 
    {
        case ADXL362:
            get_adxl362_measurement(p_data, p_data_length);
            break;
        case BME280:
            bme280_stream_sensor_data_forced_mode(&i2c_dev_bme280, p_data, p_data_length);
            break;
        case CCS811:
            get_ccs811_measurement(p_data, p_data_length);
            break;
        case MAX30105:
            get_max30105_status(p_data, p_data_length);
            break;
        default:
            break;
    }
    return ret_code;
}