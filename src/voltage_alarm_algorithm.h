#ifndef VOLTAGE_ALARM_ALGORITHM_H__
#define VOLTAGE_ALARM_ALGORITHM_H__

#include <stdint.h>
#include "nrf.h"
//#include "nrf_drv_saadc.h"
//#include "nrf_drv_rtc.h"

#ifdef __cplusplus
extern "C" {
#endif

// MUST MATCH vband_saadc_controler.c defines
//#define RTC_FREQUENCY              4096 // must be a factor of 32768
//#define RTC_FREQUENCY              1024 // must be a factor of 32768
// no need for define RTC_FREQUENCY here
//#define NUM_MEASUREMENTS           2048
#define NUM_MEASUREMENTS           512

#define TOGGLE_PIN                  NRF_GPIO_PIN_MAP(0,30)      // toggle test signal pin
//#define FFT_TEST_IN_SAMPLES_LEN     NUM_MEASUREMENTS*2 // for cfft
#define FFT_TEST_IN_SAMPLES_LEN     NUM_MEASUREMENTS // for rfft
#define FFT_TEST_OUT_SAMPLES_LEN    NUM_MEASUREMENTS
#define FFT_LEN                     NUM_MEASUREMENTS
#define FFT_BLE_SAMPLES_LEN         64 // 80 is too long, 79 is ok
//#define SAMPLE_RATE                 4096.0f
#define SAMPLE_RATE                 1024.0f // effective sample rate after decimation
#define NUM_TEST_FREQS              2
#define NUM_CONSEC_TO_ALARM         1  /** consecutive states to trigger alarm state */
#define NUM_CONSEC_TO_UNALARM       5 /** consecutive states to trigger non-alarm state */
#define LOG_MULT_FACTOR             40 // number to multiply the log result
#define LOG_OFFSET_FACTOR           60 // number to add after the log result
//#define FFT_SEND_RAW_SAMPLES // if defined, then BLE sends raw samples from ch1

#pragma pack(1)
typedef struct voltage_algorithm_results {
        bool overall_alarm;
	bool ch1_alarm;
	bool ch2_alarm;
	bool ch3_alarm;
        uint8_t num_fft_bins;
        uint8_t fft_bin_size;
        uint8_t ch1_fft_results[FFT_BLE_SAMPLES_LEN];
        uint8_t ch2_fft_results[FFT_BLE_SAMPLES_LEN];
        uint8_t ch3_fft_results[FFT_BLE_SAMPLES_LEN];
} voltage_algorithm_results;

bool check_for_voltage_detection(uint8_t *results_buf, 
                                 float * adc_ch1, 
                                 float * adc_ch2, 
                                 float * adc_ch3, 
                                 uint16_t len);

void set_voltage_alarm_threshold(uint32_t * threshold);

#ifdef __cplusplus
}
#endif

#endif /* VBAND_SAADC_CONTROLLER_H__ */