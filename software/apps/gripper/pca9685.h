// LSM303AGR accelerometer and magnetometer

#pragma once

#include "nrf_twi_mngr.h"

// Address of PCA9735 board
static const uint8_t SERVO_ADDRESS = 0x40;


// Register definitions for PCA9735 board
typedef enum {
  MODE1 = 0X00,
  MODE2 = 0X01,
  SUBADR1 = 0X02,
  SUBADR2 = 0X03,
  SUBADR3 = 0X04,
  ALLCALLADR = 0X05,
  LED0_ON_L = 0X06,
  LED0_ON_H = 0X07,
  LED0_OFF_L = 0X08,
  LED0_OFF_H = 0X09,
  PRE_SCALE = 0XFE,
  TestMode = 0xFF,
} servo_reg_t;
// there are more registers, but I think we only care about LED0 (only 1 motor)

// Function prototypes

// Initialize and configure the LSM303AGR accelerometer/magnetometer
//
// i2c - pointer to already initialized and enabled twim instance
void pca9685_init(const nrf_twi_mngr_t* i2c);

void send_servo(float duty_cycle);

