#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "app_util_platform.h"
#include "nrf_delay.h"
#include "nrf_drv_gpiote.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
/*lint -save -e689 */ /* Apparent end of comment ignored */
#include "arm_const_structs.h"
/*lint -restore */
#include "voltage_alarm_algorithm.h"

static float freqs_alert[NUM_TEST_FREQS]   = {50.0f, 60.0f};//, 50.0f};   // US and Europe Voltage Frequency
static float mag_threshold[NUM_TEST_FREQS] = {30.0f, 30.0f};//, 30.0f}; // thresholds for the above freqs

// for FFT-based algorithm
static float m_fft_temp_input_f32[FFT_TEST_IN_SAMPLES_LEN];
static float m_fft_temp_input1_f32[FFT_TEST_IN_SAMPLES_LEN];
static float m_fft_temp_output_f32[FFT_TEST_OUT_SAMPLES_LEN];
static float m_fft_temp_f32[FFT_TEST_OUT_SAMPLES_LEN];
static float m_pos_freq_fft_mag[FFT_TEST_OUT_SAMPLES_LEN/2];
static float sine_wave[FFT_TEST_IN_SAMPLES_LEN];
static float sine_wave_real[FFT_TEST_IN_SAMPLES_LEN/2];

static void clear_FPU_interrupts(void)
{
    __set_FPSCR(__get_FPSCR() & ~(0x0000009F)); 
    (void) __get_FPSCR();
    NVIC_ClearPendingIRQ(FPU_IRQn);
}

/**
 * @brief Function for generating sine wave samples for FFT calculations.
 *
 * This function fill up output array with generated sine wave data with proper sampling frequency.
 * Must be executed before fft_process function.
 *
 * @param[in] p_input     Input array to fill with sine wave generated data.
 * @param[in] size        Input array size.
 * @param[in] sample_freq Sine wave sampling frequency.
 * @param[in] sine_freq   Sine wave frequency.
 * @param[in] add_noise   Flag for enable or disble adding noise for generated data.
 */
static void fft_generate_samples(float32_t * p_input,
                                 uint16_t    size,
                                 float32_t   sampling_freq,
                                 float32_t   sine_freq,
                                 bool        add_noise)
{
    uint32_t i;

    /* Remember that sample is represented as two next values in array. */
    uint32_t sample_idx = 0;

    if (2 > size)
    {
        return;
    }

    for (i = 0; i < (size - 1UL); i += 2) 
    {
        sample_idx = i / 2;
        // Real part.
        p_input[(uint16_t)i] = sin(sine_freq * (2.f * PI) * sample_idx / sampling_freq);
        // Img part.
        p_input[(uint16_t)i + 1] = 0;
    }
}

/**
 * @brief Function for centering FFT around midpoint of array
 * @param[in] p_input_output        Pointer to input data array with complex number samples in time domain.
 * @param[in] output_size           Processed data array size.
 */
static void fft_shift(float32_t *                   p_input_output,
                      uint16_t                      output_size)
{
    uint16_t mid_i = output_size/2;

    // copy to temp array before shifting
    memcpy(&m_fft_temp_f32[0], p_input_output, sizeof(m_fft_temp_f32));

    // copy 2nd half to front
    memcpy(p_input_output,&m_fft_temp_f32[mid_i], mid_i*sizeof(m_fft_temp_f32[0]));

    // copy 1st half to back
    memcpy(p_input_output+mid_i,&m_fft_temp_f32[0], mid_i*sizeof(m_fft_temp_f32[0]));
}


/**
 * @brief Function for processing generated sine wave samples.
 * @param[in] p_input        Pointer to input data array with complex number samples in time domain.
 * @param[in] p_input_struct Pointer to cfft instance structure describing input data.
 * @param[out] p_output      Pointer to processed data (bins) array in frequency domain.
 * @param[in] output_size    Processed data array size.
 */
static void fft_process(float32_t *                   p_input,
                        const arm_cfft_instance_f32 * p_input_struct,
                        float32_t *                   p_output,
                        uint16_t                      output_size)
{   
    // Use CFFT module to process the data.
    arm_cfft_f32(p_input_struct, p_input, 0, 1);
    // Calculate the magnitude at each bin using Complex Magnitude Module function.
    arm_cmplx_mag_f32(p_input, p_output, output_size);
    // FFT shift - put DC component at center of output array
    //fft_shift(p_output, output_size);
}


static void add_imag_data(float * input, float * output, uint16_t len)
{
    uint32_t i;

    /* Remember that sample is represented as two next values in array. */
    uint32_t sample_idx = 0;

    if (2 > len)
    {
        return;
    }

    for (i = 0; i < (len - 1UL); i += 2) 
    {
        sample_idx = i / 2;
        // Real part.
        output[(uint16_t)i] = *(input + sample_idx);
        // Complex part
        output[(uint16_t)i + 1] = 0;
    }
}

static bool run_fft_single_channel(float * input, uint16_t len)
{
    uint8_t nearest_ind = 0;
    uint16_t mid_i = len/2;
    float hz_per_index = SAMPLE_RATE/len;

    //NRF_LOG_INFO("CH VAL 0: " NRF_LOG_FLOAT_MARKER, NRF_LOG_FLOAT(*input));
    //NRF_LOG_INFO("CH VAL 0: " NRF_LOG_FLOAT_MARKER, NRF_LOG_FLOAT(*(input+len-1)));
    add_imag_data(input, &m_fft_temp_input_f32[0], 2*len); // 2*len to account for complex pairs
    fft_process(m_fft_temp_input_f32, &arm_cfft_sR_f32_len128, m_fft_temp_output_f32, FFT_TEST_OUT_SAMPLES_LEN);

    // if using Sin wave
    //fft_process(input, &arm_cfft_sR_f32_len128, m_fft_temp_output_f32, FFT_TEST_OUT_SAMPLES_LEN);

    // copy postive side only
    memcpy(&m_pos_freq_fft_mag[0], &m_fft_temp_output_f32[0], sizeof(m_pos_freq_fft_mag)); 
    //memcpy(&m_pos_freq_fft_mag[0], &m_fft_temp_output_f32[mid_i], sizeof(m_pos_freq_fft_mag)); 

    // check for threshold hit
    float mag_sq;
    uint8_t num__freqs = sizeof(freqs_alert)/sizeof(freqs_alert[0]);
    for (uint8_t i = 0; i < num__freqs; i++)
    {
        // compute the closest index
        nearest_ind = (uint8_t) round(freqs_alert[i] / hz_per_index) - 1;
        nearest_ind = (nearest_ind <= 0) ? 0: nearest_ind;
        //NRF_LOG_INFO("FFT MAG: " NRF_LOG_FLOAT_MARKER, NRF_LOG_FLOAT(m_pos_freq_fft_mag[nearest_ind]));
        if (m_pos_freq_fft_mag[nearest_ind] >= mag_threshold[i])
        {
            return true;
        }
    }
    return false;
}

static void fft_algorithm(struct voltage_algorithm_results *results, float * adc_ch1, float * adc_ch2, float * adc_ch3, uint16_t len)
{
    uint16_t i;

    results->ch1_alarm = run_fft_single_channel(adc_ch1, len);
    for(i = 0; i < len/2; i++)
    {
        results->ch1_fft_results[i] = (uint8_t) m_pos_freq_fft_mag[i];
    }

    results->ch2_alarm = run_fft_single_channel(adc_ch2, len);
    for(i = 0; i < len/2; i++)
    {
        results->ch2_fft_results[i] = (uint8_t) m_pos_freq_fft_mag[i];
    }

    results->ch3_alarm = run_fft_single_channel(adc_ch3, len);
    for(i = 0; i < len/2; i++)
    {
        results->ch3_fft_results[i] = (uint8_t) m_pos_freq_fft_mag[i];
    }
    
    clear_FPU_interrupts();
}

static bool run_goertzel_single_channel(float * input, uint16_t len)
{
    // constants
    static bool g_init = false;
    static uint16_t N = FFT_TEST_OUT_SAMPLES_LEN;
    static uint16_t k[NUM_TEST_FREQS];
    static float sample_rate = SAMPLE_RATE;
    static float w[NUM_TEST_FREQS];
    static float cosine[NUM_TEST_FREQS];
    static float sine[NUM_TEST_FREQS];
    static float coeff[NUM_TEST_FREQS];

    // working vars
    uint16_t i, j;
    float Q0, Q1, Q2, mag, mag_sq, mag_sq_thresh;

    //NRF_LOG_INFO("GOERTZEL MAG VAL: " NRF_LOG_FLOAT_MARKER, NRF_LOG_FLOAT(*input));

    // initialize constants
    if(!g_init)
    {
        for(i = 0; i < NUM_TEST_FREQS; i++)
        {
            k[i]      = (uint16_t)(0.5 + ((N*freqs_alert[i]) / sample_rate));
            w[i]      = (2.f * PI / N)*k[i];
            cosine[i] = cos(w[i]);
            sine[i]   = sin(w[i]);
            coeff[i]  = 2*cosine[i];
        }
        g_init = true;
    }

    // run algorithm, this is optimized and does not output phase
    // https://www.embedded.com/design/configurable-systems/4024443/The-Goertzel-Algorithm
    for(i = 0; i < NUM_TEST_FREQS; i++)
    {
        Q1 = Q2 = 0;
        for(j = 0; j < len; j++)
        {
            Q0 = (coeff[i] * Q1) - Q2 + *(input + j);
            Q2 = Q1;
            Q1 = Q0;
        }
        mag_sq = powf(Q1, 2) + powf(Q2, 2) - (Q1*Q2*coeff[i]);
        mag    = sqrtf(mag_sq);
        //NRF_LOG_INFO("GOERTZEL MAG: " NRF_LOG_FLOAT_MARKER, NRF_LOG_FLOAT(mag));
        if (mag >= mag_threshold[i])
        {
            return true;
        }
     }
     return false;
}

static void goertzel_algorithm(struct voltage_algorithm_results *results, float * adc_ch1, float * adc_ch2, float * adc_ch3, uint16_t len)
{
    results->ch1_alarm = run_goertzel_single_channel(adc_ch1, len);
    results->ch2_alarm = run_goertzel_single_channel(adc_ch2, len);
    results->ch3_alarm = run_goertzel_single_channel(adc_ch3, len);

    clear_FPU_interrupts();
}

static bool check_for_alarm_state_change(bool ch1_alarm, bool ch2_alarm, bool ch3_alarm)
{
    static bool current_alarm_state = false;
    static bool current_alarm_buf[NUM_CONSEC_TO_ALARM] = {false};
    static bool current_unalarm_buf[NUM_CONSEC_TO_UNALARM] = {true};
    static uint8_t idx1 = 0;
    static uint8_t idx2 = 0;

    if (ch1_alarm | ch2_alarm | ch3_alarm)
    {
        current_alarm_buf[idx1] = true;
        current_unalarm_buf[idx2] = false;
    }
    else
    {
        current_alarm_buf[idx1] = false;
        current_unalarm_buf[idx2] = true;
    }

    idx1 = (idx1 + 1) % NUM_CONSEC_TO_ALARM;
    idx2 = (idx2 + 1) % NUM_CONSEC_TO_UNALARM;

    uint8_t i;
    if (current_alarm_state == false)
    {
        current_alarm_state = true;
        for (i = 0; i < NUM_CONSEC_TO_ALARM; i++)
        {
            if (current_alarm_buf[i] == false)
            {
                current_alarm_state = false;
                break;
            }
        }
    }
    else
    {
        current_alarm_state = false;
        for (i = 0; i < NUM_CONSEC_TO_UNALARM; i++)
        {
            if (current_unalarm_buf[i] == false)
            {
                current_alarm_state = true;
                break;
            }
        }
    }
    return current_alarm_state;
}

bool check_for_voltage_detection(uint8_t *results_buf, float * adc_ch1, float * adc_ch2, float * adc_ch3, uint16_t len)
{
    static voltage_algorithm_results results;

    // configure toggle pin for latency testing
    static bool pin_cfg = false;
    if(!nrf_drv_gpiote_is_init())
    {
        APP_ERROR_CHECK(nrf_drv_gpiote_init());
    }
    if(!pin_cfg)
    {
        nrf_drv_gpiote_out_config_t out_config = GPIOTE_CONFIG_OUT_SIMPLE(false); // false = init low = WAKEUP
        APP_ERROR_CHECK(nrf_drv_gpiote_out_init(TOGGLE_PIN, &out_config));
        nrf_drv_gpiote_out_set(TOGGLE_PIN);
        pin_cfg = true;
    }
  
    // for running example Sin wave
    if(0)
    {
        fft_generate_samples(&sine_wave[0], FFT_TEST_IN_SAMPLES_LEN, SAMPLE_RATE, 60.0, 0);
        for(uint16_t i = 0; i < FFT_TEST_IN_SAMPLES_LEN; i += 2)
        {
            sine_wave_real[i/2] = sine_wave[i];
        }    
    }

    results.num_fft_bins = FFT_TEST_OUT_SAMPLES_LEN/2;
    results.fft_bin_size = SAMPLE_RATE/len;

    // FFT algorithm
    nrf_drv_gpiote_out_clear(TOGGLE_PIN);
    if(1)
    {
        fft_algorithm(&results, adc_ch1, adc_ch2, adc_ch3, len);
    }
    nrf_drv_gpiote_out_set(TOGGLE_PIN);

    // Goertzel
    nrf_drv_gpiote_out_clear(TOGGLE_PIN);
    if(0)
    {
        goertzel_algorithm(&results, adc_ch1, adc_ch2, adc_ch3, len);
    }
    nrf_drv_gpiote_out_set(TOGGLE_PIN);

    // check for consecutive states to trigger alarm change
    results.overall_alarm = check_for_alarm_state_change(results.ch1_alarm, results.ch2_alarm, results.ch3_alarm);

    // copy to p_data, structure is memory aligned (pack(1))
    memcpy(results_buf, (uint8_t *) &results.overall_alarm, sizeof(results));

    // return alarm state
    return results.overall_alarm;
}                              