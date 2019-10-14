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
#include "vband_common.h"

static float freqs_alert[NUM_TEST_FREQS]   = {50.0f, 60.0f};//, 50.0f};   // US and Europe Voltage Frequency
float mag_threshold[NUM_TEST_FREQS] = {300.0f, 300.0f};//, 30.0f};   // thresholds for the above freqs

// for FFT-based algorithm
static float m_fft_temp_input_f32[FFT_TEST_IN_SAMPLES_LEN];
//static float m_fft_temp_input1_f32[FFT_TEST_IN_SAMPLES_LEN];
static float m_fft_temp_output_f32[FFT_TEST_OUT_SAMPLES_LEN];
static float m_fft_temp_f32[FFT_TEST_OUT_SAMPLES_LEN];
static float m_pos_freq_fft_mag[FFT_TEST_OUT_SAMPLES_LEN/2];
static float sine_wave[FFT_TEST_IN_SAMPLES_LEN];
static float sine_wave_real[FFT_TEST_IN_SAMPLES_LEN/2];

static bool running_algorithm = false;

// instance for doing Real FFT
static arm_rfft_fast_instance_f32 fftInstance;
static bool rfft_initialized = false;

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
    uint16_t i;

    /* Remember that sample is represented as two next values in array. */
    uint16_t sample_idx = 0;

    if (2 > size)
    {
        return;
    }

    for (i = 0; i < (size - 1UL); i += 2) 
    {
        //sample_idx = i / 2;
        // Real part.
        p_input[i] = sin(sine_freq * (2.f * PI) * sample_idx / sampling_freq);
        // Img part.
        p_input[i + 1] = 0;
        sample_idx++;
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

/**
 * @brief do FFT using arm_rfft_fast_f32
 * @param[in] p_input        Pointer to input data array with real number samples in time domain.
 * @param[out] p_output      Pointer to processed data (bins) array in frequency domain.
 */
static void rfft_process(float32_t *                   p_input,
                         float32_t *                   p_output,
                         float32_t *                   p_magnitude)
{   
    // only do it once
    if(!rfft_initialized) {
      arm_status status = arm_rfft_fast_init_f32(&fftInstance, FFT_LEN);
      ASSERT(status == ARM_MATH_SUCCESS);
      rfft_initialized = true;
    }
    arm_rfft_fast_f32(&fftInstance, p_input, p_output, 0);
    // throw away the Nyquist frequency at p_output[1]
    p_output[1] = 0;
    arm_cmplx_mag_f32(p_output, p_magnitude, FFT_LEN/2);
}

// prepare the complex array for FFT, array is {real[0], img[0], real[1], img[1], real[2],...} of length = FFTlen*2
static void add_imag_data(float * input, float * output, uint16_t len)
{
    uint16_t i;

    ASSERT(len <= 8192); // FFT length larger than 4096 not supported, also make sure we don't overflow on uint16_t loop counter

    /* Remember that sample is represented as two next values in array. */
    uint16_t sample_idx = 0;

    if (2 > len)
    {
        return;
    }

    for (i = 0; i < (len - 1UL); i += 2) 
    {
        //sample_idx = i / 2;
        // Real part.
        output[i] = *(input + sample_idx);
        // Complex part
        output[i + 1] = 0;
        sample_idx++;
    }
}

static bool run_fft_single_channel(float * input, uint16_t len)
{
    uint8_t nearest_ind = 0;
    uint16_t mid_i = len/2;
    float hz_per_index = SAMPLE_RATE/len;
    const static arm_cfft_instance_f32 *S;
    float mean;
    
    // subtract out the mean
    arm_mean_f32(input, len, &mean);
    mean = -1 * mean;
    arm_offset_f32(input, mean, m_fft_temp_input_f32, len);
    //NRF_LOG_INFO("CH VAL 0: " NRF_LOG_FLOAT_MARKER, NRF_LOG_FLOAT(*input));
    //add_imag_data(input, &m_fft_temp_input_f32[0], 2*len); // 2*len to account for complex pairs
    /*
    switch(len) // select the constant twiddle factors based on FFT length
    {
      case 16:
        S = &arm_cfft_sR_f32_len16;
        break;
      case 32:
        S = &arm_cfft_sR_f32_len32;
        break;
      case 64:
        S = &arm_cfft_sR_f32_len64;
        break;
      case 128:
        S = &arm_cfft_sR_f32_len128;
        break;
      case 256:
        S = &arm_cfft_sR_f32_len256;
        break;
      case 512:
        S = &arm_cfft_sR_f32_len512;
        break;
      case 1024:
        S = &arm_cfft_sR_f32_len1024;
        break;
      case 2048:
        S = &arm_cfft_sR_f32_len2048;
        break;
      case 4096:
        S = &arm_cfft_sR_f32_len4096;
        break;
      default:
        ASSERT(false); // other FFT sizes not supported
    }
    */
    //fft_process(m_fft_temp_input_f32, &S, m_fft_temp_output_f32, FFT_TEST_OUT_SAMPLES_LEN);
    rfft_process(m_fft_temp_input_f32, m_fft_temp_output_f32, m_pos_freq_fft_mag);

    // copy postive side only
    //memcpy(&m_pos_freq_fft_mag[0], &m_fft_temp_output_f32[0], sizeof(m_pos_freq_fft_mag));
    // normalize the FFT magnitude to an 8 bit number (0 to 255)
//    arm_scale_f32(m_pos_freq_fft_mag, (255.0f / FFT_TEST_OUT_SAMPLES_LEN), m_pos_freq_fft_mag, FFT_TEST_OUT_SAMPLES_LEN);
//    for (uint8_t i = 0; i < FFT_TEST_OUT_SAMPLES_LEN/2; i++)
//    {
//      m_pos_freq_fft_mag[i] = m_pos_freq_fft_mag[i] / (FFT_TEST_OUT_SAMPLES_LEN * 255);
//    }

    // check for threshold hit
    //float mag_sq;
    //uint8_t num__freqs = sizeof(freqs_alert)/sizeof(freqs_alert[0]);
    for (uint8_t i = 0; i < NUM_TEST_FREQS; i++)
    {
        // compute the closest index
        nearest_ind = (uint8_t) round(freqs_alert[i] / hz_per_index);
        nearest_ind = (nearest_ind <= 0) ? 0: nearest_ind;
        //NRF_LOG_INFO("bin size: %dHz", hz_per_index);
        //NRF_LOG_INFO("FFT index: %d", nearest_ind);
        //NRF_LOG_INFO("FFT MAG: " NRF_LOG_FLOAT_MARKER, NRF_LOG_FLOAT(m_pos_freq_fft_mag[nearest_ind]));
        if ((log(m_pos_freq_fft_mag[nearest_ind])*LOG_MULT_FACTOR+LOG_OFFSET_FACTOR) >= mag_threshold[i])
        {
//            NRF_LOG_INFO("threshold %d: " NRF_LOG_FLOAT_MARKER, i, NRF_LOG_FLOAT(mag_threshold[i]));
//            NRF_LOG_INFO("exceed threshold " NRF_LOG_FLOAT_MARKER " index: %d", NRF_LOG_FLOAT(log(m_pos_freq_fft_mag[nearest_ind])*LOG_MULT_FACTOR+LOG_OFFSET_FACTOR), nearest_ind);
            return true;
        }
        else
        {
            //NRF_LOG_INFO("didn't exceed threshold " NRF_LOG_FLOAT_MARKER " index: %d", NRF_LOG_FLOAT(log(m_pos_freq_fft_mag[nearest_ind])*LOG_MULT_FACTOR+LOG_OFFSET_FACTOR), nearest_ind);
        }
    }
    return false;
}

static void fft_algorithm(struct voltage_algorithm_results *results, float * adc_ch1, float * adc_ch2, float * adc_ch3, uint16_t len)
{
    uint16_t i;
    ASSERT(FFT_TEST_OUT_SAMPLES_LEN/2 >= FFT_BLE_SAMPLES_LEN); // trying to send more data points than we have in the fft pos result array is not allowed

#ifdef FFT_SEND_RAW_SAMPLES
    //  send raw ADC samples
    for(i = 0; i < FFT_BLE_SAMPLES_LEN*3; i++)
    {

        if(i < FFT_BLE_SAMPLES_LEN) {
          results->ch1_fft_results[i] = (uint8_t) (adc_ch1[i]*255);
        } else {
            if(i < FFT_BLE_SAMPLES_LEN*2) {
              results->ch2_fft_results[i-FFT_BLE_SAMPLES_LEN] = (uint8_t) (adc_ch1[i]*255);
            } else {
              results->ch3_fft_results[i-FFT_BLE_SAMPLES_LEN*2] = (uint8_t) (adc_ch1[i]*255);
            }
        }

    }
#else

    results->ch1_alarm = run_fft_single_channel(adc_ch1, len);
    for(i = 0; i < FFT_BLE_SAMPLES_LEN; i++)
    {
        m_pos_freq_fft_mag[i] = (log(m_pos_freq_fft_mag[i])*LOG_MULT_FACTOR+LOG_OFFSET_FACTOR);
        if(m_pos_freq_fft_mag[i] > 255) results->ch1_fft_results[i] = (uint8_t) 255;
        else
        {
          if(m_pos_freq_fft_mag[i] > 0) results->ch1_fft_results[i] = (uint8_t) m_pos_freq_fft_mag[i];
          else results->ch1_fft_results[i] = 0;
        }

//        if(i == 30) {
//          NRF_LOG_INFO("ch1 uint8_t: %d = float: " NRF_LOG_FLOAT_MARKER, results->ch1_fft_results[i], NRF_LOG_FLOAT(m_pos_freq_fft_mag[i]));
//        }
    }

    results->ch2_alarm = run_fft_single_channel(adc_ch2, len);
    for(i = 0; i < FFT_BLE_SAMPLES_LEN; i++)
    {
        m_pos_freq_fft_mag[i] = (log(m_pos_freq_fft_mag[i])*LOG_MULT_FACTOR+LOG_OFFSET_FACTOR);
        if(m_pos_freq_fft_mag[i] > 255) results->ch2_fft_results[i] = (uint8_t) 255;
        else
        {
          if(m_pos_freq_fft_mag[i] > 0) results->ch2_fft_results[i] = (uint8_t) m_pos_freq_fft_mag[i];
          else results->ch2_fft_results[i] = 0;
        }
        //else results->ch2_fft_results[i] = (uint8_t) m_pos_freq_fft_mag[i];
    }

/*    for(i = 0; i < FFT_BLE_SAMPLES_LEN*3; i++)
    {

        if(i < FFT_BLE_SAMPLES_LEN) {
          if(m_pos_freq_fft_mag[i] > 255) results->ch1_fft_results[i] = 255;
          else results->ch1_fft_results[i] = (uint8_t) m_pos_freq_fft_mag[i];

          if(m_pos_freq_fft_mag[i] > 0.001) {
//            NRF_LOG_INFO("uint8_t: %d = float: " NRF_LOG_FLOAT_MARKER " at bin %d", results->ch1_fft_results[i], NRF_LOG_FLOAT(m_pos_freq_fft_mag[i]), i);
          }
        } else {
            if(i < FFT_BLE_SAMPLES_LEN*2) {
              if(m_pos_freq_fft_mag[i] > 255) results->ch2_fft_results[i-FFT_BLE_SAMPLES_LEN] = 255;
              else results->ch2_fft_results[i-FFT_BLE_SAMPLES_LEN] = (uint8_t) m_pos_freq_fft_mag[i];

              if(m_pos_freq_fft_mag[i] > 0.001) {
//                NRF_LOG_INFO("uint8_t: %d = float: " NRF_LOG_FLOAT_MARKER " at bin %d", results->ch2_fft_results[i-FFT_BLE_SAMPLES_LEN], NRF_LOG_FLOAT(m_pos_freq_fft_mag[i]), i);
              }
            } else {
              if(m_pos_freq_fft_mag[i] > 255) results->ch3_fft_results[i-FFT_BLE_SAMPLES_LEN*2] = 255;
              else results->ch3_fft_results[i-FFT_BLE_SAMPLES_LEN*2] = (uint8_t) m_pos_freq_fft_mag[i];
              
              if(m_pos_freq_fft_mag[i] > 0.001) {
//                NRF_LOG_INFO("uint8_t: %d = float: " NRF_LOG_FLOAT_MARKER " at bin %d", results->ch3_fft_results[i-FFT_BLE_SAMPLES_LEN*2], NRF_LOG_FLOAT(m_pos_freq_fft_mag[i]), i);
              }
            }
        }

    }*/

    //NRF_LOG_INFO("begin ch3 fft");
    results->ch3_alarm = run_fft_single_channel(adc_ch3, len);
    //NRF_LOG_INFO("end ch3 fft");
    for(i = 0; i < FFT_BLE_SAMPLES_LEN; i++)
    {
        //results->ch3_fft_results[i] = (uint8_t) m_pos_freq_fft_mag[i];
        m_pos_freq_fft_mag[i] = (log(m_pos_freq_fft_mag[i])*LOG_MULT_FACTOR+LOG_OFFSET_FACTOR);
        if(m_pos_freq_fft_mag[i] > 255) results->ch3_fft_results[i] = (uint8_t) 255;
        else
        {
          if(m_pos_freq_fft_mag[i] > 0) results->ch3_fft_results[i] = (uint8_t) m_pos_freq_fft_mag[i];
          else results->ch3_fft_results[i] = 0;
        }
        //else results->ch3_fft_results[i] = (uint8_t) m_pos_freq_fft_mag[i];
    }
#endif

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
    //static bool current_alarm_buf[NUM_CONSEC_TO_ALARM] = {false};
    //static bool current_unalarm_buf[NUM_CONSEC_TO_UNALARM] = {true};
    //static uint8_t idx1 = 0;
    //static uint8_t idx2 = 0;
    static uint8_t alarm_on_count;
    static uint8_t alarm_off_count;

    if (ch1_alarm | ch2_alarm | ch3_alarm)
    {
        //current_alarm_buf[idx1] = true;
        //current_unalarm_buf[idx2] = false;
        alarm_on_count++;
        alarm_off_count = 0;
    }
    else
    {
        //current_alarm_buf[idx1] = false;
        //current_unalarm_buf[idx2] = true;
        alarm_off_count++;
        alarm_on_count = 0;
    }

    uint8_t i;
    if (current_alarm_state == false && alarm_on_count >= NUM_CONSEC_TO_ALARM)
    {
        //idx1 = (idx1 + 1) % NUM_CONSEC_TO_ALARM;
        current_alarm_state = true;
//        for (i = 0; i < NUM_CONSEC_TO_ALARM; i++)
//        {
//            if (current_alarm_buf[i] == false)
//            {
                //current_alarm_state = false;
                //break;
            //}
        //}
    }
    if (current_alarm_state == true && alarm_off_count >= NUM_CONSEC_TO_UNALARM)
//    else
    {
        //idx2 = (idx2 + 1) % NUM_CONSEC_TO_UNALARM;
        current_alarm_state = false;
        //for (i = 0; i < NUM_CONSEC_TO_UNALARM; i++)
        //{
          //  if (current_unalarm_buf[i] == false)
            //{
                //NRF_LOG_INFO("unalarm false: %d", i);
                //current_alarm_state = true;
                //break;
            //}
        //}
    }
    return current_alarm_state;
}

bool check_for_voltage_detection(uint8_t *results_buf, float * adc_ch1, float * adc_ch2, float * adc_ch3, uint16_t len)
{
    static voltage_algorithm_results results;
    
    running_algorithm = true;

    // configure toggle pin for latency testing
    /*
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
*/
    // for running example Sin wave
    if(0)
    {
        fft_generate_samples(&sine_wave[0], FFT_TEST_IN_SAMPLES_LEN, SAMPLE_RATE, 60.0, 0);
        uint16_t i2 = 0;
        for(uint16_t i = 0; i < FFT_TEST_IN_SAMPLES_LEN; i += 2)
        {
            sine_wave_real[i2] = sine_wave[i];
            i2++;
        }    
    }

    results.num_fft_bins = FFT_BLE_SAMPLES_LEN;
    results.fft_bin_size = SAMPLE_RATE/len;

    // FFT algorithm
    //nrf_drv_gpiote_out_clear(TOGGLE_PIN);
    if(1)
    {
        fft_algorithm(&results, adc_ch1, adc_ch2, adc_ch3, len);
        //fft_algorithm(&results, adc_ch1, sine_wave_real, adc_ch3, len);
    }
    //nrf_drv_gpiote_out_set(TOGGLE_PIN);

    // Goertzel
    //nrf_drv_gpiote_out_clear(TOGGLE_PIN);
    if(0)
    {
        goertzel_algorithm(&results, adc_ch1, adc_ch2, adc_ch3, len);
    }
    //nrf_drv_gpiote_out_set(TOGGLE_PIN);

    // check for consecutive states to trigger alarm change
    results.overall_alarm = check_for_alarm_state_change(results.ch1_alarm, results.ch2_alarm, results.ch3_alarm);

    // copy to p_data, structure is memory aligned (pack(1))
    memcpy(results_buf, (uint8_t *) &results.overall_alarm, sizeof(results));

    running_algorithm = false;

    // return alarm state
    return results.overall_alarm;
}         

void set_voltage_alarm_threshold(uint32_t * threshold)
{
    while (running_algorithm) {nrf_delay_us(1);}
    for(uint8_t i = 0; i < NUM_TEST_FREQS; i++)
    {
        mag_threshold[i] = *threshold*1.0f;
    }
}                     