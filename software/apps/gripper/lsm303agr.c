// LSM303AGR driver for Microbit_v2
//
// Initializes sensor and communicates over I2C
// Capable of reading temperature, acceleration, and magnetic field strength

#include <stdbool.h>
#include <stdint.h>

#include "lsm303agr.h"
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
  printf("We are here\n");
  i2c_manager = i2c;
  printf("stupid board\n");
  // ---Initialize Servo Board---
  uint8_t mode1 = i2c_reg_read(SERVO_ADDRESS, MODE1);
  printf("Mode1 is %d\n", mode1);

  // put it to sleep
  i2c_reg_write(SERVO_ADDRESS, MODE1, 0x10);
  printf("Brd tarted\n");
  // write to prescaler
  float freq = 500;
  uint8_t prescaler = (uint8_t) roundf(25000000.0f/(4096 *freq))-1;
  printf("Write prescaler: %d\n", prescaler);
  i2c_reg_write(SERVO_ADDRESS, PRE_SCALE, prescaler);
  // wake it up
  i2c_reg_write(SERVO_ADDRESS, MODE1, 0x80);
  // ???? Totem pole
  i2c_reg_write(SERVO_ADDRESS, MODE2, 0x04);
}

// Read the internal temperature sensor
//
// Return measurement as floating point value in degrees C
float lsm303agr_read_temperature(void) {
  //TODO: implement me
  uint8_t res_L = i2c_reg_read(LSM303AGR_ACC_ADDRESS, LSM303AGR_ACC_TEMP_L);
  uint8_t res_H = i2c_reg_read(LSM303AGR_ACC_ADDRESS, LSM303AGR_ACC_TEMP_H);
  // printf("ResL: %x\n", res_L);
  // printf("ResH: %x\n", res_H);
  uint16_t res = (res_H<<8) | res_L; 
  // printf("res: %x\n", res);
  float temp = ((float)((int16_t)res))/256. + 25.0;
  return temp;
}

lsm303agr_measurement_t lsm303agr_read_accelerometer(void) {
  //TODO: implement me
  uint8_t x_L = i2c_reg_read(LSM303AGR_ACC_ADDRESS, LSM303AGR_ACC_OUT_X_L);
  uint8_t x_H = i2c_reg_read(LSM303AGR_ACC_ADDRESS, LSM303AGR_ACC_OUT_X_H);
  uint8_t y_L = i2c_reg_read(LSM303AGR_ACC_ADDRESS, LSM303AGR_ACC_OUT_Y_L);
  uint8_t y_H = i2c_reg_read(LSM303AGR_ACC_ADDRESS, LSM303AGR_ACC_OUT_Y_H);
  uint8_t z_L = i2c_reg_read(LSM303AGR_ACC_ADDRESS, LSM303AGR_ACC_OUT_Z_L);
  uint8_t z_H = i2c_reg_read(LSM303AGR_ACC_ADDRESS, LSM303AGR_ACC_OUT_Z_H);
  // printf("X: %x, %x\n", x_L, x_H);
  // printf("Y: %x, %x\n", y_L, y_H);
  // printf("X: %x, %x\n", z_L, z_H);
  uint16_t res_x = (x_H<<8) | x_L;
  uint16_t res_y = (y_H<<8) | y_L;
  uint16_t res_z = (z_H<<8) | z_L;
  int16_t new_x = ((int16_t)res_x) >> 6;
  int16_t new_y = ((int16_t)res_y) >> 6;
  int16_t new_z = ((int16_t)res_z) >> 6;
  float scaling = 3.9;
  float acc_x = scaling*new_x/1000.;
  float acc_y = scaling*new_y/1000.;
  float acc_z = scaling*new_z/1000.;
  printf("Acc: (%f, %f, %f)\n", acc_x, acc_y, acc_z);
  lsm303agr_measurement_t measurement = {acc_x, acc_y, acc_z};
  float phi = compute_phi(measurement);
  printf("Phi: %f\n", phi);
  return measurement;
}

lsm303agr_measurement_t lsm303agr_read_magnetometer(void) {
  //TODO: implement me
  uint8_t x_L = i2c_reg_read(LSM303AGR_MAG_ADDRESS, LSM303AGR_MAG_OUT_X_L_REG);
  uint8_t x_H = i2c_reg_read(LSM303AGR_MAG_ADDRESS, LSM303AGR_MAG_OUT_X_H_REG);
  uint8_t y_L = i2c_reg_read(LSM303AGR_MAG_ADDRESS, LSM303AGR_MAG_OUT_Y_L_REG);
  uint8_t y_H = i2c_reg_read(LSM303AGR_MAG_ADDRESS, LSM303AGR_MAG_OUT_Y_H_REG);
  uint8_t z_L = i2c_reg_read(LSM303AGR_MAG_ADDRESS, LSM303AGR_MAG_OUT_Z_L_REG);
  uint8_t z_H = i2c_reg_read(LSM303AGR_MAG_ADDRESS, LSM303AGR_MAG_OUT_Z_H_REG);
  uint16_t res_x = (x_H<<8) | x_L;
  uint16_t res_y = (y_H<<8) | y_L;
  uint16_t res_z = (z_H<<8) | z_L;
  float scaling = 1.5;
  float mag_x = scaling*((int16_t)res_x)/1000.;
  float mag_y = scaling*((int16_t)res_y)/1000.;
  float mag_z = scaling*((int16_t)res_z)/1000.;
  printf("Mag: (%f, %f, %f)\n", mag_x, mag_y, mag_z);
  lsm303agr_measurement_t measurement = {mag_x, mag_y, mag_z};
  return measurement;
}


void send_servo(uint32_t angle){
  printf("Send servo %ld (0x%lx)\n", angle, angle);
  uint8_t pre = i2c_reg_read(SERVO_ADDRESS, PRE_SCALE);
  printf("Prescaler is %d\n", pre);
  uint8_t mode1 = i2c_reg_read(SERVO_ADDRESS, MODE1);
  printf("Mode1 is %d\n", mode1);
  uint8_t mode2 = i2c_reg_read(SERVO_ADDRESS, MODE2);
  printf("Mode2 is %d\n", mode2);
  // Do bitshifting
  // uint8_t byte1 = val&0xFF;
  // uint8_t byte2 = (val>>8)&0xFF;
  // uint8_t byte3 = (val>>16)&0xFF;
  // uint8_t byte4 = (val>>24)&0xFF;
  // printf("Bytes: %x %x %x %x", byte1, byte2, byte3, byte4);
  // i2c_reg_write(SERVO_ADDRESS, LED0_ON_L, byte1);
  // i2c_reg_write(SERVO_ADDRESS, LED0_ON_H, byte2);
  // i2c_reg_write(SERVO_ADDRESS, LED0_OFF_L, byte3);
  // i2c_reg_write(SERVO_ADDRESS, LED0_OFF_H, byte4);
}



