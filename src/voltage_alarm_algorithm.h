#ifndef VOLTAGE_ALARM_ALGORITHM_H__
#define VOLTAGE_ALARM_ALGORITHM_H__

#include <stdint.h>
#include "nrf.h"
#include "nrf_drv_saadc.h"
#include "nrf_drv_rtc.h"

#ifdef __cplusplus
extern "C" {
#endif

#define TOGGLE_PIN                  NRF_GPIO_PIN_MAP(0,30)      // Wakeup signal pin
#define FFT_TEST_IN_SAMPLES_LEN     256
#define FFT_TEST_OUT_SAMPLES_LEN    FFT_TEST_IN_SAMPLES_LEN/2
#define SAMPLE_RATE                 1024.0f
#define NUM_TEST_FREQS              2
#define NUM_CONSEC_TO_ALARM         2 /** consecutive states to trigger alarm state */
#define NUM_CONSEC_TO_UNALARM       5 /** consecutive states to trigger non-alarm state */

#pragma pack(1)
typedef struct voltage_algorithm_results {
        bool overall_alarm;
	bool ch1_alarm;
	bool ch2_alarm;
	bool ch3_alarm;
        uint8_t num_fft_bins;
        uint8_t fft_bin_size;
        uint8_t ch1_fft_results[FFT_TEST_OUT_SAMPLES_LEN/2];
        uint8_t ch2_fft_results[FFT_TEST_OUT_SAMPLES_LEN/2];
        uint8_t ch3_fft_results[FFT_TEST_OUT_SAMPLES_LEN/2];
} voltage_algorithm_results;

bool check_for_voltage_detection(uint8_t *results_buf, 
                                 float * adc_ch1, 
                                 float * adc_ch2, 
                                 float * adc_ch3, 
                                 uint16_t len);

#ifdef __cplusplus
}
#endif

#endif /* VBAND_SAADC_CONTROLLER_H__ */