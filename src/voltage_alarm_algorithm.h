#ifndef VOLTAGE_ALARM_ALGORITHM_H__
#define VOLTAGE_ALARM_ALGORITHM_H__

#include <stdint.h>
#include "nrf.h"
#include "nrf_drv_saadc.h"
#include "nrf_drv_rtc.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct voltage_algorithm_results {
	bool ch1_results;
	bool ch2_results;
	bool ch3_results;
} voltage_algorithm_results;

void check_for_voltage_detection(struct voltage_algorithm_results *results, 
                                 float * adc_ch1, 
                                 float * adc_ch2, 
                                 float * adc_ch3, 
                                 uint16_t len);

#ifdef __cplusplus
}
#endif

#endif /* VBAND_SAADC_CONTROLLER_H__ */