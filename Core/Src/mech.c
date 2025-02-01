/*
 * mech.c
 *
 *  Created on: Jun 19, 2024
 *      Author: ivanm
 */

float P_rate_xy = 1.5, I_rate_xy = 1.0, D_rate_xy = 0.5;
float P_rate_z = 2, I_rate_z = 12, D_rate_z = 0.03;

#include "main.h"

void norm_motors(uint16_t *motors){
	for(uint8_t i = 0; i < 5; i++){
		if(motors[i] < 1000) motors[i] = 1000;
		if(motors[i] > 2000) motors[i] = 2000;
	}
}

void write_motors(uint16_t *motors){
	norm_motors(motors);
	TIM2->CCR1 = motors[3]; // propellers
	TIM2->CCR2 = motors[2];
	TIM2->CCR3 = motors[1];
	TIM2->CCR4 = motors[0];

	TIM3->CCR1 = motors[5]; // camera servo
}


void calculatePID_xy(float error, float *error_prev, float *itemr_prev, float *output){
	float Pterm = P_rate_xy * error;

	float Iterm = *itemr_prev + I_rate_xy * (error + *error_prev) * 0.002;
	if (Iterm > 400.0) Iterm = 400;
	if (Iterm < -400.0) Iterm = -400;

	float Dterm = D_rate_xy*(error - *error_prev)/0.004;
	if (Dterm > 400.0) Dterm = 400;
	if (Dterm < -400.0) Dterm = -400;

	*output = Pterm + Iterm + Dterm;
	*error_prev = error;
	*itemr_prev = Iterm;


}

void calculatePID_z(float error, float *error_prev, float *itemr_prev, float *output){
	float Pterm = P_rate_z * error;

	float Iterm = *itemr_prev + I_rate_z * (error + *error_prev) * 0.002;
	if (Iterm > 400.0) Iterm = 400;
	if (Iterm < -400.0) Iterm = -400;

	float Dterm = D_rate_z*(error - *error_prev)/0.004;

	*output = Pterm + Iterm + Dterm;
	*error_prev = error;
	*itemr_prev = Iterm;


}
