/*
 * i2c_dev.h
 *
 *  Created on: May 8, 2024
 *      Author: ivanm
 */

#ifndef INC_I2C_DEV_H_
#define INC_I2C_DEV_H_

// custom defines

// addresses
#define MPU6050_ADR 0xD0
#define I2C_TIMEOUT 1000
#define WHO_AM_I_REG 0x75
#define PWR_MGMT_1_REG 0x6B
#define SMPLRT_DIV_REG 0x19
#define ACCEL_CONFIG_REG 0x1C
#define ACCEL_XOUT_H_REG 0x3B
#define TEMP_OUT_H_REG 0x41
#define GYRO_CONFIG_REG 0x1B
#define LOW_PASS_FILTER_REG 0x1A
#define GYRO_XOUT_H_REG 0x43

// values
#define SAMPLING_RATE_1KHZ 0b00000111
#define GYRO_500DS 0b00001000
#define ACCEL_4G 0b00001000
#define ACCEL_8G 0b00010000

// functions
void MPU6050_init(I2C_HandleTypeDef *hi2c, uint8_t sampling_rate,  uint8_t guro_sens, uint8_t accel_sens);
void MPU6050_getGyroValues(int16_t *angles);
void MPU6050_getAccelValues(int16_t *acceleration);

#endif /* INC_I2C_DEV_H_ */
