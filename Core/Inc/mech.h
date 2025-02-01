/*
 * mech.h
 *
 *  Created on: Jun 19, 2024
 *      Author: ivanm
 */

#ifndef INC_MECH_H_
#define INC_MECH_H_

void write_motors(int16_t *motors);
void norm_motors(int16_t *motors);
void calculatePID_xy(float error, float *error_prev, float *itemr_prev, float *output);
void calculatePID_z(float error, float *error_prev, float *itemr_prev, float *output);

#endif /* INC_MECH_H_ */
