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
//#include "nrf_drv_twi.h"
#include "nrfx_twim.h"
#include "nrf_delay.h"
#include "ccs811.h"
#include "bme280.h"
#include "ADXL362.h"
#include "ADXL362_to_nRF_spi.h"
#include "MAX30105.h"
#include "Pozyx.h"
#include "vband_ext_sensor_controller.h"
#include "vband_common.h"
#include "heart_rate_detector.h"
#include "nrf_drv_gpiote.h"
#include "nrf_gpio.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#define TWI_BUFFER_SIZE                     32

// prox threshold for MAX30105
#define PROX_THRESHOLD                      5

#ifndef SCL_PIN
#define SCL_PIN             NRF_GPIO_PIN_MAP(0,14)    // SCL signal pin
#define SDA_PIN             NRF_GPIO_PIN_MAP(0,16)    // SDA signal pin
#define WAKEUP_PIN          NRF_GPIO_PIN_MAP(1,12)     // Wakeup signal pin
#define INT_PIN             NRF_GPIO_PIN_MAP(1,13)     // Data ready (active low)
#endif

static bool simulate_on = false;
static bool nack_error = false;

struct bme280_dev i2c_dev_bme280;
struct ccs811_dev i2c_dev_ccs811;
struct max30105_dev i2c_dev_max301015;
struct Pozyx_dev i2c_dev_Pozyx;

// to track which sensors are enabled / connected
static uint16_t enabled_sensors = 0;

/* TWI instance. */
static const nrfx_twim_t m_twi = NRFX_TWIM_INSTANCE(0);
/* Busy bool for I2C */
static volatile bool m_xfer_done1 = false;
/* I2C enable bool */
static volatile bool twi_enabled = false;

// track Optical sensor state
static max30105_state_t max301015_state = PRESENCE_NOT_DETECTED;
static uint32_t hr_ticks = 0;

static uint8_t twi_buffer[TWI_BUFFER_SIZE];

// wrapper for sd_app_evt_wait(), adding clearing FPU pending interrupt
void sd_clear_FPU_int_and_sleep(void)
{
  clear_FPU_interrupts();
  sd_app_evt_wait();
}

/**
 * @brief TWI events handler.
 */
static void twi_handler(nrfx_twim_evt_t const * p_event, void * p_context)
{
    switch (p_event->type)
    {
        case NRFX_TWIM_EVT_DONE:
            if (p_event->xfer_desc.type == NRFX_TWIM_XFER_RX)
            {
                // do nothing for now
            }
            m_xfer_done1 = true;
//            NRF_LOG_INFO("TWI Done");
            break;
        case NRFX_TWIM_EVT_ADDRESS_NACK:
//            NRF_LOG_WARNING("TWI no address ACK");
            nack_error = true;
            break;
        case NRFX_TWIM_EVT_DATA_NACK:
//            NRF_LOG_WARNING("TWI no data ACK");
            nack_error = true;
            break;
        default:
            break;
    }
}

static void twi_wait(void)
{
    uint16_t ctr = 0;
    uint16_t limit_us = 1600; // system clock is 64MHz, so 64 instructions = 1us
    ret_code_t err_code;
    while(m_xfer_done1 == false && nrfx_twim_is_busy(&m_twi) && (ctr < limit_us))
    {
        nrf_delay_us(1);
        // about 50 us per byte transfer, x32 bytes = 1600 us max
        ctr++;
        //sd_clear_FPU_int_and_sleep();//sd_app_evt_wait();
//        APP_ERROR_CHECK(err_code);
    }
    if(ctr >= limit_us)
    {
      // escaped a stuck TWI transaction? reset TWI instance
      NRF_LOG_WARNING("TWI stuck detected");
      twi_uninit();
      twi_init();
    }
}


/**
 * @brief TWI initialization.
 */
static void twi_init(void)
{
    ret_code_t err_code;

    const nrfx_twim_config_t twi_config = {
       .scl                = SCL_PIN,
       .sda                = SDA_PIN,
       .frequency          = NRF_TWIM_FREQ_400K, // NRF_TWIM_FREQ_400K NRF_TWIM_FREQ_250K NRF_TWIM_FREQ_100K 
       .interrupt_priority = APP_IRQ_PRIORITY_LOW,
       //.clear_bus_init     = true//,
       .hold_bus_uninit    = true
    };

    // using freq_100k seems to be more stable than 250k or 400k on board A10 some kind of i2c pull up issue.

    // set SCL pin to drive S0S1, see if it fixes SCL stuck low issue
    //nrf_gpio_cfg_input(SDA_PIN, NRF_GPIO_PIN_PULLUP);
    //nrf_gpio_cfg_input(SCL_PIN, NRF_GPIO_PIN_PULLUP);
    
    err_code = nrfx_twim_init(&m_twi, &twi_config, twi_handler, NULL);
    APP_ERROR_CHECK(err_code);

    nrfx_twim_enable(&m_twi);
    twi_enabled = true;
}

static void twi_uninit(void)
{
    nrfx_twim_uninit(&m_twi);
    twi_enabled = false;
}

static void twi_enable(void)
{
    if(!twi_enabled)
    {
      nrfx_twim_enable(&m_twi);
      twi_enabled = true;
    }
}

static void twi_disable(void)
{
    if(twi_enabled)
    {
      nrfx_twim_disable(&m_twi);
      twi_enabled = false;
    }
}

static int8_t twi_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
    ret_code_t err_code;
    uint16_t retry = 3;

    if(twi_enabled)
    {
      //NRF_LOG_INFO("TWI read");
      twi_wait(); // wait until previous TWI operation is done before modifying twi_buffer

      /* Prepare for the read with a register address write 1 byte */
      twi_buffer[0] = reg_addr;
      //twi_buffer[1] = 0;
      do
      {
        m_xfer_done1 = false;
        nack_error =  false;
        do
        {
          err_code = nrfx_twim_tx(&m_twi, dev_id, twi_buffer, 1, true);
        } while(err_code == NRF_ERROR_BUSY);
        twi_wait();
        if(retry-- == 0) break;
      } while(nack_error); // try again if nack error
      APP_ERROR_CHECK(err_code);

      //nrf_delay_ms(1);

      /* Now read the register */
      retry = 3;
      do
      {
        m_xfer_done1 = false;
        nack_error =  false;
        do
        {
          err_code = nrfx_twim_rx(&m_twi, dev_id, data, len);
        } while(err_code == NRF_ERROR_BUSY);
        twi_wait();
        if(retry-- == 0) break;
      } while(nack_error); // try again if nack error
      twi_wait(); // wait until data received and copied to the data variable
      APP_ERROR_CHECK(err_code);
    }
    else
    {
      err_code = 1;
    }
    return err_code;
}

static int8_t twi_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
    ret_code_t err_code;
    uint16_t retry = 3;

    if(twi_enabled)
    {
      //NRF_LOG_INFO("TWI write");
      /* Build tx buffer */
      ASSERT(len <= TWI_BUFFER_SIZE-1); // catch buffer issue

      twi_wait(); // wait until previous TWI operation is done before modifying twi_buffer
      twi_buffer[0] = reg_addr;
      if(data) memcpy(&twi_buffer[1],data,len);

      /* Transfer tx buffer */
      do
      {
        m_xfer_done1 = false;
        nack_error =  false;
        do
        {
          err_code = nrfx_twim_tx(&m_twi, dev_id, twi_buffer, 1+len, false);
        } while(err_code == NRF_ERROR_BUSY);
        twi_wait();
        if(retry-- == 0) break;
      } while(nack_error);
      APP_ERROR_CHECK(err_code);

    }
    else
    {
      err_code = 1;
    }
    //NRF_LOG_FLUSH();
    return err_code;
}

static bool scan_for_twi_device(uint8_t dev_address)
{
    ret_code_t err_code;

    if(twi_enabled)
    {
      //NRF_LOG_INFO("TWI scan");
      //uint8_t sample_data;
      twi_buffer[0] = 0;
      do
      {
        m_xfer_done1 = false;
        nack_error = false;
        err_code = nrfx_twim_tx(&m_twi, dev_address, twi_buffer, 1, false);
      } while(err_code == NRF_ERROR_BUSY);
      twi_wait();
      return (nack_error == false) ? true:false;
    }
    else
    {
      return false;
    }
}

static void print_sensor_data(struct bme280_data *comp_data)
{
#ifdef BME280_FLOAT_ENABLE
        NRF_LOG_INFO("%0.2f, %0.2f, %0.2f\r\n",comp_data->temperature, comp_data->pressure, comp_data->humidity);
#else
        //NRF_LOG_INFO("\n")
        NRF_LOG_INFO("Temp:  %ld, Humid:  %ld, Pressure:  %ld",comp_data->temperature, comp_data->humidity, comp_data->pressure);
        //NRF_LOG_INFO("Temperature (C): " NRF_LOG_FLOAT_MARKER, NRF_LOG_FLOAT(comp_data->temperature*0.01f));
        //NRF_LOG_INFO("Pressure (hPa): " NRF_LOG_FLOAT_MARKER, NRF_LOG_FLOAT(comp_data->pressure/256.0f));
        //NRF_LOG_INFO("Humidity (%%RH): " NRF_LOG_FLOAT_MARKER, NRF_LOG_FLOAT(comp_data->humidity/1024.0f));
#endif
}

static void sensor_bme280_init(void)
{
    i2c_dev_bme280.dev_id = BME280_I2C_ADDR_PRIM;
    i2c_dev_bme280.intf = BME280_I2C_INTF;
    i2c_dev_bme280.read = twi_read;
    i2c_dev_bme280.write = twi_write;
    i2c_dev_bme280.delay_ms = nrf_delay_ms;
    if(scan_for_twi_device(BME280_I2C_ADDR_PRIM))
    {
        if (bme280_init(&i2c_dev_bme280) == BME280_OK)
        {
            enabled_sensors |= BME280;
            NRF_LOG_INFO("BME280 initialized");
            NRF_LOG_FLUSH();
        }
        else
        {
            NRF_LOG_WARNING("BME280 init failed");
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
		// Set BME280 operating mode
        dev->settings.osr_h = BME280_OVERSAMPLING_1X;
        dev->settings.osr_p = BME280_OVERSAMPLING_4X;
        dev->settings.osr_t = BME280_OVERSAMPLING_1X;
        dev->settings.filter = BME280_FILTER_COEFF_4;
        settings_sel = BME280_OSR_PRESS_SEL | BME280_OSR_TEMP_SEL | BME280_OSR_HUM_SEL | BME280_FILTER_SEL;
        rslt = bme280_set_sensor_settings(settings_sel, dev);

        /* Ping for sensor data */
        rslt = bme280_set_sensor_mode(BME280_FORCED_MODE, dev);
        nrf_delay_ms(20); // measurement time
        rslt = bme280_get_sensor_data(BME280_ALL, &comp_data, dev);
        if(rslt == BME280_E_NULL_PTR)
        {
          NRF_LOG_WARNING("BME280 NULL pointer");
        }
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
    memcpy(&data_buffer[4], &comp_data.humidity, sizeof(comp_data.humidity));
    memcpy(&data_buffer[8], &comp_data.pressure, sizeof(comp_data.pressure));
    data_buffer_length = 12;

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
    //MAX30105_setSingleLED(SMP_AVE_1, false, FIFO_A_FULL_F, ADC_RGE_01, SMP_RT_50, LED_PW_15BIT, 1);  // configure single LED mode to initiate proximity detection
    //MAX30105_setDualLED(SMP_AVE_1, false, FIFO_A_FULL_F, ADC_RGE_01, SMP_RT_50, LED_PW_15BIT, 0, 0);
    MAX30105_setMultiLED(SMP_AVE_1, false, FIFO_A_FULL_F, ADC_RGE_00, SMP_RT_50, LED_PW_15BIT, 0, 0, 0, 0x40, SLOT_IR_LED2, SLOT_DISABLED, SLOT_DISABLED, SLOT_DISABLED);
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
    //MAX30105_softReset();
    //MAX30105_wakeUp();
    if (scan_for_twi_device(MAX30105_I2C_ADDR))
    {
        if (MAX30105_init(&i2c_dev_max301015) == MAX30105_NO_ERROR) 
        {
            enabled_sensors |= MAX30105;
            max30105_setup_proximity_sensing();
            NRF_LOG_INFO("MAX30105 initialized");
        }
        else
        {
          NRF_LOG_WARNING("MAX30105 init failed");
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
        MAX30105_writeReg(REG_MODE_CONFIG, MODE_MULTI); // go back into proximity detection
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
    nrf_gpio_pin_clear(CCS811_WAKEUP_PIN);
    nrf_delay_ms(1);
    if (scan_for_twi_device(CCS811_ADDR))
    {
        nrf_gpio_pin_set(CCS811_WAKEUP_PIN);
        NRF_LOG_INFO("going to init CCS811");
        if(ccs811_init(&i2c_dev_ccs811) == true)
        {
            enabled_sensors |= CCS811;
            //nrf_delay_ms(5);
            //ccs811_reset(&i2c_dev_ccs811);
            nrf_delay_ms(5);
            ccs811_start_mode(&i2c_dev_ccs811, 0x00); // start in idle
            NRF_LOG_INFO("CCS811 initialized");
        }
        else
        {
          NRF_LOG_WARNING("CCS811 HW_ID does not match!");
          nrf_gpio_pin_clear(CCS811_RESET_PIN);
        }
    }
    else
    {
        NRF_LOG_WARNING("CCS811 not found! Skipping...");
        nrf_gpio_pin_set(CCS811_WAKEUP_PIN);
        nrf_gpio_pin_clear(CCS811_RESET_PIN);
    }

//#ifdef BOARD_VWEDGE_V2

//#else
    nrf_gpio_pin_set(CCS811_WAKEUP_PIN);
//#endif
}

static void get_ccs811_measurement(uint8_t * p_data, uint16_t * p_data_length)
{
    static uint8_t data_buffer[20];
    static uint16_t data_buffer_length;
    
    static uint16_t eCO2 = 0;
    static uint16_t TVOC = 0;
	static uint8_t nError = 0;
    if (!simulate_on)
    {
        ccs811_measure(&i2c_dev_ccs811, &eCO2, &TVOC, &nError);
    }
    else
    {
        eCO2++;
        TVOC++;
    }
	/* error bitfields
		0 WRITE_REG_INVALID The CCS811 received an I²C write request addressed to this station but with invalid register address ID
		1 READ_REG_INVALID The CCS811 received an I²C read request to a mailbox ID that is invalid
		2 MEASMODE_INVALID The CCS811 received an I²C request to write an unsupported mode to MEAS_MODE
		3 MAX_RESISTANCE The sensor resistance measurement has reached or exceeded the maximum range
		4 HEATER_FAULT The Heater current in the CCS811 is not in range
		5 HEATER_SUPPLY The Heat
		*/

    NRF_LOG_INFO("CO2: %d, TVOC: %d, Error %d", eCO2, TVOC, nError);

    // package for BLE
    memcpy(&data_buffer[0], &eCO2, sizeof(eCO2));
    memcpy(&data_buffer[2], &TVOC, sizeof(TVOC));
	data_buffer[4] = nError;
    data_buffer_length = 5;

    // assign pointers
    memcpy(p_data, &data_buffer[0], data_buffer_length);
    *p_data_length = data_buffer_length;
}

static void sensor_adxl362_init(void)
{
#if NRF_LOG_ENABLED
//        NRF_LOG_INFO("going to call ADXL362_Init");
#endif
    if (ADXL362_Init())
    {
        enabled_sensors |= ADXL362;

        // set up for burst write configuration
        unsigned char buffer[16] = {
          ADXL362_WRITE_REG,        // write command
          ADXL362_REG_THRESH_ACT_L, // first reg address is activity threshold L
          0x5A, // 0x20 activity threshold L
          0x00, // 0x21 activity threshold H
          0x04, // 0x22 activity time value
          0x58, // 0x23 inactivity threshold L
          0x02, // 0x24 inactivity threshold H
          0x12, // 0x25 inactivity time value L
          0x00, // 0x26 inactivity time value H (0x1C2) 450 ticks @12.5Hz = 36 seconds
          0x3F, // 0x27 control - loop mode, referenced detection
          0x02, // 0x28 FIFO control, stream mode
          0x02, // 0x29 FIFO samples, watermark lDevel
          0xC0, // 0x2A map INT1 to AWAKE bit, active low
          0x04, // 0x2B map INT2 to FIFO watermark bit, active high
          //0x10, // 0x2C 2g range, quarter bandwidth, 12.5Hz output
          0x11, // 0x2C 2g range, quarter bandwidth, 25Hz output
          //0x2E // 0x2D ultra low noise, wake up mode, auto sleep, measurement enabled
          0x0E // 0x2D wake up mode, auto sleep, measurement enabled
        };
#if NRF_LOG_ENABLED
//        NRF_LOG_INFO("going to write ADXL config settings");
#endif
        SPI_Write(ADXL362_SLAVE_ID, buffer, 16);
#if NRF_LOG_ENABLED
//        NRF_LOG_INFO("going to uninit SPI");
#endif
        SPI_uninit();
#if NRF_LOG_ENABLED
        NRF_LOG_INFO("ADXL362 initialized");
#endif
/*
        ADXL362_SetRange(ADXL362_RANGE_2G);
        ADXL362_SetOutputRate(ADXL362_ODR_12_5_HZ);
        ADXL362_SetupDetectionMode(ADXL362_MODE_LOOP);
        ADXL362_SetupInactivityDetection(1, 100, 125);//7500); // referenced mode, 7500 = 10 minutes
        ADXL362_SetupActivityDetection(1, 20, 13); // referenced mode, 13 = 1.04 sec
        ADXL362_SetRegisterValue(ADXL362_INTMAP1_INT_LOW | ADXL362_INTMAP1_AWAKE, ADXL362_REG_INTMAP1, 1);
        ADXL362_SetRegisterValue(ADXL362_INTMAP2_INT_LOW | ADXL362_INTMAP2_AWAKE, ADXL362_REG_INTMAP2, 1);
        ADXL362_SetPowerMode(1);
*/
    }
    else
    {
#if NRF_LOG_ENABLED
        NRF_LOG_WARNING("ADXL362 not found! Skipping...");
#endif
    }
}

static void get_adxl362_measurement(uint8_t * p_data, uint16_t * p_data_length)
{
    static uint8_t data_buffer[20];
    static uint16_t data_buffer_length;
    static uint8_t status_reg = 0;
    static int16_t xdata = 0;
    static int16_t ydata = 0;
    static int16_t zdata = 0;
    static bool last_awake_status = false;
    static bool awake_status = false;
    unsigned char status   = 0;

    if (!simulate_on)
    {
        status = SPI_Init(ADXL362_LSB_FIRST, ADXL362_SPI_CLOCK, ADXL362_CLK_POL, ADXL362_CLK_EDGE);
        ADXL362_GetRegisterValue(&status_reg, ADXL362_REG_STATUS, 1);
        last_awake_status = awake_status;
        awake_status = status_reg & ADXL362_STATUS_AWAKE;
        ADXL362_GetXyz(&xdata, &ydata, &zdata);
        SPI_uninit();
    }
    else
    {
        awake_status = true;
        xdata++;
        ydata++;
        zdata++;
    }
    if(awake_status != last_awake_status)
    {
      if(awake_status)
      {
        NRF_LOG_INFO("x:  %d  , y:  %d  , z:  %d, awake", xdata, ydata, zdata);
      } else {
        NRF_LOG_INFO("x:  %d  , y:  %d  , z:  %d, sleep", xdata, ydata, zdata);
      }
    }

    // package for BLE
    memcpy(&data_buffer[0], &xdata, sizeof(xdata));
    memcpy(&data_buffer[2], &ydata, sizeof(ydata));
    memcpy(&data_buffer[4], &zdata, sizeof(zdata));
    memcpy(&data_buffer[6], &awake_status, sizeof(awake_status));
    data_buffer_length = 8;

    // assign pointers
    memcpy(p_data, &data_buffer[0], data_buffer_length);
    *p_data_length = data_buffer_length;
}

static void sensor_pozyx_init(void) 
{
    i2c_dev_Pozyx.read = twi_read;
    i2c_dev_Pozyx.write = twi_write;

    if (scan_for_twi_device(POZYX_I2C_ADDRESS))
    {
        if (Pozyx_init(&i2c_dev_Pozyx) == POZYX_SUCCESS) 
        {
            enabled_sensors |= POZYX;
            
            NRF_LOG_INFO("Pozyx initialized");
        }
        else
        {
          NRF_LOG_WARNING("Pozyx init failed");
        }
    } 
    else 
    {
        NRF_LOG_WARNING("Pozyx not found! Skipping...");
        if (Pozyx_init(&i2c_dev_Pozyx) == POZYX_SUCCESS) 
        {
            enabled_sensors |= POZYX;
            
            NRF_LOG_INFO("Pozyx initialized");
        }
        else
        {
          NRF_LOG_WARNING("Pozyx init failed");
        }
    }
}

uint16_t vband_sensor_init(sensor_type_t use_sensors)
{
    int8_t rslt_bme280 = BME280_OK;
    int8_t rslt_max30105 = BME280_OK;
    
    if (use_sensors & SIMULATE)
    {
        simulate_on = true;
        return use_sensors;
    }
    else
    {
#ifdef LDO_1_8_EN_PIN
        // turn off 1.8V LDO
        //nrf_gpio_pin_clear(LDO_1_8_EN_PIN);// needed for VWedge2.0 with the 2.7V discharge problem
        nrf_gpio_cfg_output(SCL_PIN);// needed for VWedge2.0 with the 2.7V discharge problem
        nrf_gpio_pin_clear(SCL_PIN);// needed for VWedge2.0 with the 2.7V discharge problem
        nrf_gpio_cfg_output(SDA_PIN);// needed for VWedge2.0 with the 2.7V discharge problem
        nrf_gpio_pin_clear(SDA_PIN);// needed for VWedge2.0 with the 2.7V discharge problem
#endif
        if(use_sensors & ADXL362)
        {
            sensor_adxl362_init();
        }
#ifdef LDO_1_8_EN_PIN
        // turn on 1.8V LDO
        nrf_gpio_pin_set(LDO_1_8_EN_PIN);
        nrf_delay_ms(10);
#endif
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
        if(use_sensors & VSENSOR)
        {
            // set GPIO pin as output
            nrf_gpio_cfg_output(VSENSOR_POWER);
            // turn on analog section
            nrf_gpio_pin_clear(VSENSOR_POWER);
        }
        if(use_sensors & POZYX)
        {
            sensor_pozyx_init();
        }
        // set GPIO pin as output for capacitive sensor mode
        nrf_gpio_cfg_output(CAPSENSOR_MODE);
        // set to low power mode
        nrf_gpio_pin_clear(CAPSENSOR_MODE);
    }
    return enabled_sensors;
}

sensor_return_t get_sensor_data(sensor_type_t get_sensor, uint8_t * p_data, uint16_t * p_data_length)
{
    sensor_return_t ret_code = WEDGE_SENSOR_SUCCESS;

    // check if sensor is enabled
    if (!simulate_on)
    {
        if (!(enabled_sensors & get_sensor)) return WEDGE_SENSOR_NOT_ENABLED;
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

uint32_t adxl362_configure_wakeup(void)
{
    ret_code_t err_code;

    // check if sensor is enabled
    if (simulate_on)
    {
        return NRF_SUCCESS;
    }

    if (!(enabled_sensors & ADXL362)) return NRF_SUCCESS;

    if (!nrfx_gpiote_is_init())
    {
        err_code = nrfx_gpiote_init();
        APP_ERROR_CHECK(err_code);		
    }
    // set to sense interrupt 1 from ADXL362 for motion switch status (awake is low, inactivity is high)
    nrf_gpio_cfg_sense_input(ADXL362_INT1, NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_SENSE_LOW);
    
    return NRF_SUCCESS;
}

uint32_t adxl362_wait_for_sleep(void)
{
    ret_code_t err_code;
//    uint32_t latch_value = 0;
    uint32_t pin_value = 0;

    // check if sensor is enabled
    if (simulate_on)
    {
        return NRF_SUCCESS;
    }

    if (!(enabled_sensors & ADXL362)) return NRF_SUCCESS;

    if (!nrfx_gpiote_is_init())
    {
        err_code = nrfx_gpiote_init();
        APP_ERROR_CHECK(err_code);		
    }
    // clear latched detect for motion interrupt
//    nrf_gpio_pin_latch_clear(ADXL362_INT1);

    // set to sense interrupt 1 from ADXL362 for motion switch status (awake is low, inactivity is high)
    nrf_gpio_cfg_sense_input(ADXL362_INT1, NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_SENSE_HIGH);

    do
    {
      sd_clear_FPU_int_and_sleep();
//      latch_value = nrf_gpio_pin_latch_get(ADXL362_INT1);
      pin_value = nrf_gpio_pin_read(ADXL362_INT1);
    } while(!pin_value);

    return NRF_SUCCESS;
}

void vband_sensor_wakeup(sensor_type_t use_sensors)
{
    if (simulate_on)
    {
        return;
    }
    else
    {
        if(!twi_enabled)
            twi_enable();
        if((use_sensors & MAX30105) & (enabled_sensors & MAX30105))
        {
            MAX30105_wakeUp();
        }
        if((use_sensors & CCS811) & (enabled_sensors & CCS811))
        {   
            ccs811_start_mode(&i2c_dev_ccs811, 0x30); // go to mode 3 (every 60 seconds)
        }
    }
}

void vband_sensor_sleep(sensor_type_t use_sensors)
{
    if (simulate_on)
    {
        return;
    }
    else
    {
      if(twi_enabled)
      {
          if((use_sensors & MAX30105) & (enabled_sensors & MAX30105))
          {
              MAX30105_shutDown();
          }
          if((use_sensors & CCS811) & (enabled_sensors & CCS811))
          {   
              ccs811_start_mode(&i2c_dev_ccs811, 0x00); // go to IDLE mode
          }
          if((use_sensors & BME280) & (enabled_sensors & BME280))
          {  
              // we should already be here but just in case
              bme280_set_sensor_mode(BME280_SLEEP_MODE, &i2c_dev_bme280);
          }
          // turn off twi
          twi_disable();
      }
    }
}

void vband_sensor_shutdown(sensor_type_t use_sensors)
{
    if (simulate_on)
    {
        return;
    }
    else
    {
      if(twi_enabled)
      {
          if((use_sensors & MAX30105) & (enabled_sensors & MAX30105))
          {
              MAX30105_shutDown();
          }
          if((use_sensors & CCS811) & (enabled_sensors & CCS811))
          {   
              //ccs811_start_mode(&i2c_dev_ccs811, 0x00); // go to IDLE mode
              ccs811_reset(&i2c_dev_ccs811); // saves 12uA sleep current in boot mode
          }
          if((use_sensors & BME280) & (enabled_sensors & BME280))
          {  
              // we should already be here but just in case
              bme280_set_sensor_mode(BME280_SLEEP_MODE, &i2c_dev_bme280);
          }
          // turn off twi
          twi_disable();
          twi_uninit();

#ifdef LDO_1_8_EN_PIN
          // turn off 1.8V LDO
          nrf_gpio_pin_clear(LDO_1_8_EN_PIN);
#endif

      }
    }
}