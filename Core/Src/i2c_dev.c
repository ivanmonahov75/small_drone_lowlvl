// custom i2c lib for mpu6050 and bmp180

// inculdes
#include "main.h"
#include "i2c_dev.h"
#include <math.h>
// remembered values
I2C_HandleTypeDef *local_hi2c;
const static float accel_offset[3] = { +0.055, -0.03, -0.1 }; // accel offset is measured manually

// MPU6050
// just init
void MPU6050_init(I2C_HandleTypeDef *hi2c, uint8_t sampling_rate, uint8_t guro_sens, uint8_t accel_sens) {
	// set i2c channel
	local_hi2c = hi2c;
	uint8_t temp = 0;
	// check if device is working correctly
	HAL_I2C_Mem_Read(local_hi2c, MPU6050_ADR, 0x75, 1, &temp, 1, I2C_TIMEOUT);
	// if return value is correct proceed
	if (temp == 104) {
		// wake device up
		temp = 0b00000000;
		HAL_I2C_Mem_Write(local_hi2c, MPU6050_ADR, PWR_MGMT_1_REG, 1, &temp, 1,
		I2C_TIMEOUT);

		// set correct sample rate (in this case 1 KHz)
		HAL_I2C_Mem_Write(local_hi2c, MPU6050_ADR, SMPLRT_DIV_REG, 1, &sampling_rate, 1,
		I2C_TIMEOUT);

		// set accel register value (4g)
		HAL_I2C_Mem_Write(local_hi2c, MPU6050_ADR, ACCEL_CONFIG_REG, 1, &guro_sens, 1,
		I2C_TIMEOUT);

		// set gyro register value (500 degree/sec)
		HAL_I2C_Mem_Write(local_hi2c, MPU6050_ADR, GYRO_CONFIG_REG, 1, &accel_sens, 1,
		I2C_TIMEOUT);
		// set up low pass filter (pre-installed)
		temp = 0x05;
		HAL_I2C_Mem_Write(local_hi2c, MPU6050_ADR, LOW_PASS_FILTER_REG, 1, &temp, 1, I2C_TIMEOUT);

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

// sets up gyro offset
float gyro_offset[3] = { 0, 0, 0 }; // inital offset
void MPU6050_calibrate(uint16_t sample_amount) {
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, 1); 	// turn on led
	uint16_t calibration; // temp value
	int16_t Rec_data_raw[3];

	for (calibration = 0; calibration < sample_amount; calibration++) {
		MPU6050_getGyroValues(Rec_data_raw); // get gyro values
		for (uint8_t i = 0; i < 3; i++) {
			gyro_offset[i] += Rec_data_raw[i]; // add them to total sum
			HAL_Delay(1);
		}
	}
	for (uint8_t i = 0; i < 3; i++) {
		gyro_offset[i] = (float) gyro_offset[i]; // / sample_amount; // normalize values
	}
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, 0);  	// turn off led

}

// get gyro samples, copies them in angles
void MPU6050_getGyroValues(int16_t *angles) {
	// define and write
	uint8_t Rec_Data[6];
	HAL_I2C_Mem_Read(local_hi2c, MPU6050_ADR, GYRO_XOUT_H_REG, 1, Rec_Data, 6,
	I2C_TIMEOUT);

	// transform to raw
	angles[0] = (int16_t) (Rec_Data[0] << 8 | Rec_Data[1]); // X values
	angles[1] = (int16_t) (Rec_Data[2] << 8 | Rec_Data[3]); // Y values
	angles[2] = (int16_t) (Rec_Data[4] << 8 | Rec_Data[5]); // Z values

	angles[0] = angles[0] - gyro_offset[0];
	angles[1] = angles[1] - gyro_offset[1];
	angles[2] = angles[2] - gyro_offset[2];

}

void MPU6050_getGyroRates(float *angles) {
	int16_t Raw_data[3];
	MPU6050_getGyroValues(Raw_data);
	angles[0] = Raw_data[0] / 65.5;
	angles[1] = Raw_data[1] / 65.5;
	angles[2] = Raw_data[2] / 65.5;
}

// same for accel
void MPU6050_getAccelValues(int16_t *acceleration) {
	// define and write
	uint8_t Rec_Data[6];
	HAL_I2C_Mem_Read(local_hi2c, MPU6050_ADR, ACCEL_XOUT_H_REG, 1, Rec_Data, 6,
	I2C_TIMEOUT);

	// transform to raw
	acceleration[0] = (int16_t) (Rec_Data[0] << 8 | Rec_Data[1]); // X values
	acceleration[1] = (int16_t) (Rec_Data[2] << 8 | Rec_Data[3]); // Y values
	acceleration[2] = (int16_t) (Rec_Data[4] << 8 | Rec_Data[5]); // Z values
}

// gets angles from accel data and copies them into float array
void MPU6050_getAccelAngles(float *angles) {
	// init values
	int16_t dataRaw[3];
	float angles_accel_raw[3];

	// get values
	MPU6050_getAccelValues(dataRaw);

	// from raw to G`s
	for (uint8_t i = 0; i < 3; i++) {
		angles_accel_raw[i] = (float) dataRaw[i] / 8192 - accel_offset[i];
	}

	// from G`s to angles (only x and y axis angles)
	angles[0] = atan(angles_accel_raw[1] / sqrt(angles_accel_raw[0] * angles_accel_raw[0] + angles_accel_raw[2] * angles_accel_raw[2])) * M_1_PI * 180;
	angles[1] = -atan(angles_accel_raw[0] / sqrt(angles_accel_raw[1] * angles_accel_raw[1] + angles_accel_raw[2] * angles_accel_raw[2])) * M_1_PI * 180;
}
// updates values of angles in given array if floats
float uncert[2] = { 02.0 * 2.0, 2.0 * 2.0 }; // our uncertanity level
float kalman_state[2] = { 0.0, 0.0 }; // our angle

void MPU6050_getAnglesKalman(float *input_angle, uint32_t tik) {

	float time_delta = (float) (HAL_GetTick() - tik); // get time  delta
	time_delta = 4 * 0.001;
	float angles_rates[3];
	MPU6050_getGyroRates(angles_rates); // get gyro

	float accel_angles[2];
	MPU6050_getAccelAngles(accel_angles); // get accel

	for (uint8_t i = 0; i < 2; i++) {
		input_angle[1-i] = input_angle[1-i] + angles_rates[i] * time_delta;
		uncert[i] += time_delta * time_delta * 4 * 4;
		float gain = uncert[i] * 1 / (uncert[i] + 3 * 3);
		input_angle[1-i] += gain * (accel_angles[i] - input_angle[1-i]);
		uncert[i] = uncert[i] * (1 - gain);


	}
	input_angle[2] = angles_rates[2];
}

// BMP180
// functions to make life easier
uint8_t BMP180_ReadReg(uint8_t reg) {
	HAL_I2C_Master_Transmit(local_hi2c, BMP180_ADR, &reg, 1, I2C_TIMEOUT);
	uint8_t result;
	HAL_I2C_Master_Receive(local_hi2c, BMP180_ADR, &result, 1, I2C_TIMEOUT);
	return result;
}

void BMP180_WriteReg(uint8_t reg, uint8_t cmd) {
	uint8_t arr[2] = { reg, cmd };
	HAL_I2C_Master_Transmit(local_hi2c, BMP180_ADR, arr, 2, I2C_TIMEOUT);
}

// init, includies setting up I2C and getting calibration data

// define values for MSB and LSB regs
BMP180_epp bmp180_epp;
uint8_t oss;
// define register places
const uint8_t BMP180_eep_msb[11] = { 0xaa, 0xac, 0xae, 0xb0, 0xb2, 0xb4, 0xb6, 0xb8, 0xba, 0xbc, 0xbe };
const uint8_t BMP180_eep_lsb[11] = { 0xab, 0xad, 0xaf, 0xb1, 0xb3, 0xb5, 0xb7, 0xb9, 0xbb, 0xbd, 0xbf };

void BMP180_initCalibrate(I2C_HandleTypeDef *hi2c){
	// setup i2c channel (in case we use different channels for MPU and BMP)
	local_hi2c = hi2c;

	// read all msb/lsb regs and write them to struct
	bmp180_epp.BMP180_AC1 = (BMP180_ReadReg(BMP180_eep_msb[0]) << 8) | BMP180_ReadReg(BMP180_eep_lsb[0]);
	bmp180_epp.BMP180_AC2 = (BMP180_ReadReg(BMP180_eep_msb[1]) << 8) | BMP180_ReadReg(BMP180_eep_lsb[1]);
	bmp180_epp.BMP180_AC3 = (BMP180_ReadReg(BMP180_eep_msb[2]) << 8) | BMP180_ReadReg(BMP180_eep_lsb[2]);
	bmp180_epp.BMP180_AC4 = (BMP180_ReadReg(BMP180_eep_msb[3]) << 8) | BMP180_ReadReg(BMP180_eep_lsb[3]);
	bmp180_epp.BMP180_AC5 = (BMP180_ReadReg(BMP180_eep_msb[4]) << 8) | BMP180_ReadReg(BMP180_eep_lsb[4]);
	bmp180_epp.BMP180_AC6 = (BMP180_ReadReg(BMP180_eep_msb[5]) << 8) | BMP180_ReadReg(BMP180_eep_lsb[5]);
	bmp180_epp.BMP180_B1 = (BMP180_ReadReg(BMP180_eep_msb[6]) << 8) | BMP180_ReadReg(BMP180_eep_lsb[6]);
	bmp180_epp.BMP180_B2 = (BMP180_ReadReg(BMP180_eep_msb[7]) << 8) | BMP180_ReadReg(BMP180_eep_lsb[7]);
	bmp180_epp.BMP180_MB = (BMP180_ReadReg(BMP180_eep_msb[8]) << 8) | BMP180_ReadReg(BMP180_eep_lsb[8]);
	bmp180_epp.BMP180_MC = (BMP180_ReadReg(BMP180_eep_msb[9]) << 8) | BMP180_ReadReg(BMP180_eep_lsb[9]);
	bmp180_epp.BMP180_MD = (BMP180_ReadReg(BMP180_eep_msb[10]) << 8) | BMP180_ReadReg(BMP180_eep_lsb[10]);

	// setup sensetivity
	oss = 3;

}

// gets raw temp values
int32_t BMP180_GetRawTemperature(void) {
	BMP180_WriteReg(BMP180_CONTROL_REG, BMP180_CMD_TEMP);
	HAL_Delay(BMP180_DELAY_TEMP);
	int32_t ut = (BMP180_ReadReg(BMP180_MSB_REG) << 8) | BMP180_ReadReg(BMP180_LSB_REG);
	int32_t x1 = (ut - bmp180_epp.BMP180_AC6) * bmp180_epp.BMP180_AC5 / (1 << 15);
	int32_t x2 = (bmp180_epp.BMP180_MC * (1 << 11)) / (x1 + bmp180_epp.BMP180_MD);
	int32_t b5 = x1 + x2;
	return (b5 + 8) / (1 << 4);
}

// just devide by 10
float BMP180_GetTemperature(void) {
	int32_t temp = BMP180_GetRawTemperature();
	return temp / 10.0;
}

// quality of life functions
int32_t BMP180_GetUT(void){
	return (BMP180_ReadReg(BMP180_MSB_REG) << 8) | BMP180_ReadReg(BMP180_LSB_REG);
}

int32_t BMP180_GetUP(void){
	return ((BMP180_ReadReg(BMP180_MSB_REG) << 16) | (BMP180_ReadReg(BMP180_LSB_REG) << 8) | BMP180_ReadReg(BMP180_XLSB_REG)) >> (8 - oss);
}

// gets pressure
const uint8_t BMP180_CMD_PRES[4] = { 0x34, 0x74, 0xb4, 0xf4 };
const uint8_t BMP180_DELAY_PRES[4] = { 5, 8, 14, 26 };
int32_t BMP180_GetPressure(void) {
	BMP180_WriteReg(BMP180_CONTROL_REG, BMP180_CMD_TEMP);
	HAL_Delay(BMP180_DELAY_TEMP);
	int32_t ut = BMP180_GetUT();
	BMP180_WriteReg(BMP180_CONTROL_REG, BMP180_CMD_PRES[oss]);
	HAL_Delay(BMP180_DELAY_PRES[oss]);
	int32_t up = BMP180_GetUP();
	int32_t x1 = (ut - bmp180_epp.BMP180_AC6) * bmp180_epp.BMP180_AC5 / (1 << 15);
	int32_t x2 = (bmp180_epp.BMP180_MC * (1 << 11)) / (x1 + bmp180_epp.BMP180_MD);
	int32_t b5 = x1 + x2;
	int32_t b6 = b5 - 4000;
	x1 = (bmp180_epp.BMP180_B2 * (b6 * b6 / (1 << 12))) / (1 << 11);
	x2 = bmp180_epp.BMP180_AC2 * b6 / (1 << 11);
	int32_t x3 = x1 + x2;
	int32_t b3 = (((bmp180_epp.BMP180_AC1 * 4 + x3) << oss) + 2) / 4;
	x1 = bmp180_epp.BMP180_AC3 * b6 / (1 << 13);
	x2 = (bmp180_epp.BMP180_B1 * (b6 * b6 / (1 << 12))) / (1 << 16);
	x3 = ((x1 + x2) + 2) / 4;
	uint32_t b4 = bmp180_epp.BMP180_AC4 * (uint32_t) (x3 + 32768) / (1 << 15);
	uint32_t b7 = ((uint32_t) up - b3) * (50000 >> oss);
	int32_t p;
	if (b7 < 0x80000000)
		p = (b7 * 2) / b4;
	else
		p = (b7 / b4) * 2;
	x1 = (p / (1 << 8)) * (p / (1 << 8));
	x1 = (x1 * 3038) / (1 << 16);
	x2 = (-7357 * p) / (1 << 16);
	p = p + (x1 + x2 + 3791) / (1 << 4);
	return p;
}


