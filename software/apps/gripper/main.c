// Breadboard example app
//
// Read from multiple analog sensors and control an RGB LED

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <math.h>

#include "app_timer.h"
#include "nrf_delay.h"
#include "nrfx_saadc.h"
#include "nrf_twi_mngr.h"

#include "pca9685.h"
#include "app_timer.h"

#include "microbit_v2.h"


// Analog inputs
// Breakout pins 1 and 2
// These are GPIO pin numbers that can be used in ADC configurations
// AIN1 is breakout pin 1. AIN2 is breakout pin 2.
#define ANALOG_FSR_IN NRF_SAADC_INPUT_AIN2
#define ANALOG_EMG_IN NRF_SAADC_INPUT_AIN1

// ADC channel configurations
// These are ADC channel numbers that can be used in ADC calls
#define ADC_FSR_CHANNEL 0
#define ADC_EMG_CHANNEL 1

// Global variables
APP_TIMER_DEF(sample_timer);
float volts_fsr = 0.0;
float volts_emg = 0.0;

float fsr_threshold = 250.0;
float emg_threshold = 2.0;

// Function prototypes
static void adc_init(void);
static float adc_sample_blocking(uint8_t channel);

static void sample_timer_callback(void* _unused) {
  volts_emg = adc_sample_blocking(ADC_EMG_CHANNEL);
  printf("EMG Voltage: %f\n", volts_emg);
  volts_fsr = 100*adc_sample_blocking(ADC_FSR_CHANNEL);
  printf("100x FSR Voltage: %f\n", volts_fsr);
    if(volts_emg > emg_threshold){
      // flexing
      if(volts_fsr > fsr_threshold){ // If grasping, stop moving
        printf("Grip but stop\n");
        send_servo(0);
        nrf_delay_ms(100);
      }else{
        printf("Grip!\n");
        send_servo(40);
        nrf_delay_ms(100);
        // send_servo(0);
        // nrf_delay_ms(100);
      }
    }else{
      // Not flexing
      printf("Release\n");
      send_servo(48);
      nrf_delay_ms(100);
      // send_servo(0);
      // nrf_delay_ms(100);
    }
}

static void saadc_event_callback(nrfx_saadc_evt_t const* _unused) {
  // don't care about saadc events
  // ignore this function
}


static void adc_init(void) {
  // Initialize the SAADC
  nrfx_saadc_config_t saadc_config = {
    .resolution = NRF_SAADC_RESOLUTION_12BIT,
    .oversample = NRF_SAADC_OVERSAMPLE_DISABLED,
    .interrupt_priority = 4,
    .low_power_mode = false,
  };
  ret_code_t error_code = nrfx_saadc_init(&saadc_config, saadc_event_callback);
  APP_ERROR_CHECK(error_code);

  // Initialize FSRerature sensor channel
  nrf_saadc_channel_config_t FSR_channel_config = NRFX_SAADC_DEFAULT_CHANNEL_CONFIG_SE(ANALOG_FSR_IN);
  error_code = nrfx_saadc_channel_init(ADC_FSR_CHANNEL, &FSR_channel_config);
  APP_ERROR_CHECK(error_code);

  // Initialize EMG sensor channel
  nrf_saadc_channel_config_t EMG_channel_config = NRFX_SAADC_DEFAULT_CHANNEL_CONFIG_SE(ANALOG_EMG_IN);
  error_code = nrfx_saadc_channel_init(ADC_EMG_CHANNEL, &EMG_channel_config);
  APP_ERROR_CHECK(error_code);
}

static float adc_sample_blocking(uint8_t channel) {
  // read ADC counts (0-4095)
  // this function blocks until the sample is ready
  int16_t adc_counts = 0;
  ret_code_t error_code = nrfx_saadc_sample_convert(channel, &adc_counts);
  APP_ERROR_CHECK(error_code);
  // return voltage measurement
  return (adc_counts/4096.0*3.6); 
}

// Global variables
NRF_TWI_MNGR_DEF(twi_mngr_instance, 1, 0);

int main(void) {
  printf("\n\nBoard started!\n");

  nrf_drv_twi_config_t i2c_config = NRF_DRV_TWI_DEFAULT_CONFIG;
  i2c_config.scl = EDGE_P19;
  i2c_config.sda = EDGE_P20;
  i2c_config.frequency = NRF_TWIM_FREQ_100K;
  i2c_config.interrupt_priority = 0;
  printf("Calling Init\n");
  nrf_twi_mngr_init(&twi_mngr_instance, &i2c_config);
  printf("Hello?\n");
  nrf_delay_ms(1000);
  // Initialize the LSM303AGR accelerometer/magnetometer sensor
  lsm303agr_init(&twi_mngr_instance);

  // initialize ADC
  adc_init();

  // initialize app timers
  app_timer_init();
  app_timer_create(&sample_timer, APP_TIMER_MODE_REPEATED, sample_timer_callback);

  // start timer
  // change the rate to whatever you want
  app_timer_start(sample_timer, 10000, NULL);

  send_servo(48);
  nrf_delay_ms(1000);

  send_servo(0);
  nrf_delay_ms(1000);
  // loop forever
  while (1) {
    nrf_delay_ms(1000);
  }
}

