// LSM303AGR driver for Microbit_v2
//
// Initializes sensor and communicates over I2C
// Capable of reading temperature, acceleration, and magnetic field strength

#include <stdbool.h>
#include <stdint.h>

#include "pca9685.h"
#include "nrf_delay.h"
#include <math.h>

// Pointer to an initialized I2C instance to use for transactions
static const nrf_twi_mngr_t* i2c_manager = NULL;

// Helper function to perform a 1-byte I2C read of a given register
//
// i2c_addr - address of the device to read from
// reg_addr - address of the register within the device to read
//
// returns 8-bit read value
static uint8_t i2c_reg_read(uint8_t i2c_addr, uint8_t reg_addr) {
  uint8_t rx_buf = 0;
  nrf_twi_mngr_transfer_t const read_transfer[] = {
    //TODO: implement me
    NRF_TWI_MNGR_WRITE(i2c_addr, &reg_addr, 1, NRF_TWI_MNGR_NO_STOP),
    NRF_TWI_MNGR_READ(i2c_addr, &rx_buf, 1, 0),
  };
  // printf("Read Transfer: %x\n", read_transfer);
  nrf_twi_mngr_perform(i2c_manager, NULL, read_transfer, 2, NULL);

  return rx_buf;
}

static float compute_phi(lsm303agr_measurement_t acc){
  float ax = acc.x_axis;
  float ay = acc.y_axis;
  float az = acc.z_axis;
  float sum = (ax*ax) + (ay*ay);
  float sqt = sqrt(sum);
  float res = atan((sqt)/az);
  return res*180./3.14159265358979;
}

// Helper function to perform a 1-byte I2C write of a given register
//
// i2c_addr - address of the device to write to
// reg_addr - address of the register within the device to write
static void i2c_reg_write(uint8_t i2c_addr, uint8_t reg_addr, uint8_t data) {
  //Note: there should only be a single two-byte transfer to be performed
  uint8_t bytes[2] = {reg_addr, data};
  // printf("Bytes: %x %x \n", bytes[0], bytes[1]);
  nrf_twi_mngr_transfer_t const write_transfer[] = {
    NRF_TWI_MNGR_WRITE(i2c_addr, bytes, 2, 0),
  };
  nrf_twi_mngr_perform(i2c_manager, NULL, write_transfer, 1, NULL);
}

// Initialize and configure the LSM303AGR accelerometer/magnetometer
//
// i2c - pointer to already initialized and enabled twim instance
void lsm303agr_init(const nrf_twi_mngr_t* i2c) {
  // printf("We are here\n");
  i2c_manager = i2c;
  // printf("stupid board\n");
  // ---Initialize Servo Board---
  uint8_t mode1 = i2c_reg_read(SERVO_ADDRESS, MODE1);
  // printf("Mode1 is %d\n", mode1);

  // put it to sleep
  i2c_reg_write(SERVO_ADDRESS, MODE1, 0x10);
  printf("Brd tarted\n");
  // write to prescaler
  float freq = 50;
  uint8_t prescaler = (uint8_t) roundf(25000000.0f/(4096 *freq))-1;
  // printf("Write prescaler: %d\n", prescaler);
  i2c_reg_write(SERVO_ADDRESS, PRE_SCALE, prescaler);
  // wake it up
  i2c_reg_write(SERVO_ADDRESS, MODE1, 0x80);
  // ???? Totem pole
  i2c_reg_write(SERVO_ADDRESS, MODE2, 0x04);
}


void send_servo(float duty_cycle){
  // Servo pulse width range is 900-1200 us
  // Pulse cycle is 20 ms
  //Duty cycle = PW * freq

  // Duty cycle is the percent of time that servo is on.
  // Must be between 0 and 100.
  // EG: Input 10, duty cycle is 10%
  // printf("\nSend duty cycle %f\n", duty_cycle);
  // uint8_t pre = i2c_reg_read(SERVO_ADDRESS, PRE_SCALE);
  // printf("Prescaler is %d\n", pre);
  // uint8_t mode1 = i2c_reg_read(SERVO_ADDRESS, MODE1);
  // printf("Mode1 is %d\n", mode1);
  // uint8_t mode2 = i2c_reg_read(SERVO_ADDRESS, MODE2);
  // printf("Mode2 is %d\n", mode2);
  // Do bitshifting
  // we want angle from 0 to 180. Corresponds to 0-4096
  //for (uint8_t pwmnum=0; pwmnum < 16; pwmnum++) {
    uint16_t on = duty_cycle * (4096./100.);
    // printf("On %d %x\n", on, on);
    uint16_t off = 4096 - on;
    // printf("Off %d %x\n", off, off);
    uint8_t on_l = on&0xFF;
    uint8_t on_h = (on>>8)&0xFF;
    uint8_t off_l = off&0xFF;
    uint8_t off_h = (off>>8)&0xFF;
    // printf("ON: %x %x\n", on_l, on_h);
    // printf("OFF: %x %x\n", off_l, off_h);
    i2c_reg_write(SERVO_ADDRESS, LED0_ON_L, on_l);
    i2c_reg_write(SERVO_ADDRESS, LED0_ON_H, on_h);
    i2c_reg_write(SERVO_ADDRESS, LED0_OFF_L, off_l);
    i2c_reg_write(SERVO_ADDRESS, LED0_OFF_H, off_h);
}



