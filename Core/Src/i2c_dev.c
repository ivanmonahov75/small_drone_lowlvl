// custom i2c lib for mpu6050 and bmp180

// inculdes
#include "main.h"
#include "i2c_dev.h"

// remembered values
I2C_HandleTypeDef *MPU6050_hi2c;
int32_t gyro_offset[3] = { -21, -4, 0 }; // TODO: add auto calibration

// just init
void MPU6050_init(I2C_HandleTypeDef *hi2c, uint8_t sampling_rate,
		uint8_t guro_sens, uint8_t accel_sens) {
	// set i2c channel
	MPU6050_hi2c = hi2c;
	uint8_t temp = 0;
	// check if device is working correctly
	HAL_I2C_Mem_Read(MPU6050_hi2c, MPU6050_ADR, 0x75, 1, &temp, 1, I2C_TIMEOUT);
	// if return value is correct proceed
	if (temp == 104) {
		// wake device up
		temp = 0b00000000;
		HAL_I2C_Mem_Write(MPU6050_hi2c, MPU6050_ADR, PWR_MGMT_1_REG, 1, &temp,
				1,
				I2C_TIMEOUT);

		// set correct sample rate (in this case 1 KHz)
		HAL_I2C_Mem_Write(MPU6050_hi2c, MPU6050_ADR, SMPLRT_DIV_REG, 1,
				&sampling_rate, 1,
				I2C_TIMEOUT);

		// set accel register value (4g)
		HAL_I2C_Mem_Write(MPU6050_hi2c, MPU6050_ADR, ACCEL_CONFIG_REG, 1,
				&guro_sens, 1,
				I2C_TIMEOUT);

		// set gyro register value (500 degree/sec)
		HAL_I2C_Mem_Write(MPU6050_hi2c, MPU6050_ADR, GYRO_CONFIG_REG, 1,
				&accel_sens, 1,
				I2C_TIMEOUT);
		// set up low pass filter (pre-installed)
		temp = 0x05;
		HAL_I2C_Mem_Write(MPU6050_hi2c, MPU6050_ADR, LOW_PASS_FILTER_REG, 1, &temp, 1, I2C_TIMEOUT);

	} else {
		// endless loop
		while (1) {
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, 0);
			HAL_Delay(200);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, 1);
			HAL_Delay(800);
		}
	}

}

// get gyro samples, copies them in angles
void MPU6050_getGyroValues(int16_t *angles) {
	// define and write
	uint8_t Rec_Data[6];
	HAL_I2C_Mem_Read(MPU6050_hi2c, MPU6050_ADR, GYRO_XOUT_H_REG, 1, Rec_Data, 6,
	I2C_TIMEOUT);

	// transform to raw
	angles[0] = (int16_t) (Rec_Data[0] << 8 | Rec_Data[1]); // X values
	angles[1] = (int16_t) (Rec_Data[2] << 8 | Rec_Data[3]); // Y values
	angles[2] = (int16_t) (Rec_Data[4] << 8 | Rec_Data[5]); // Z values
}

// same for accel
void MPU6050_getAccelValues(int16_t *acceleration) {
	// define and write
	uint8_t Rec_Data[6];
	HAL_I2C_Mem_Read(MPU6050_hi2c, MPU6050_ADR, ACCEL_XOUT_H_REG, 1, Rec_Data,
			6,
			I2C_TIMEOUT);

	// transform to raw
	acceleration[0] = (int16_t) (Rec_Data[0] << 8 | Rec_Data[1]); // X values
	acceleration[1] = (int16_t) (Rec_Data[2] << 8 | Rec_Data[3]); // Y values
	acceleration[2] = (int16_t) (Rec_Data[4] << 8 | Rec_Data[5]); // Z values
}

// updates values of angles in given array if floats
void MPU6050_updateAngles(float *angles, uint32_t tm) {
	float temp = 0;
	int16_t Rec_data_raw[3];
	MPU6050_getGyroValues(Rec_data_raw);

	for (uint8_t i = 0; i++; i < 3) {

		temp = (float) (HAL_GetTick() - tm) * (Rec_data_raw[i] - gyro_offset[i])
				/ 65500;
		if (fabs(temp) > 0.001) {
			angles[i] += temp;

		}

	}

}
