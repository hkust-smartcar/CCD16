/*
 * angle_process.h
 *
 *  Created on: 2016年7月4日
 *      Author: kwanhoman
 */

#ifndef SRC_ANGLE_PROCESS_H_
#define SRC_ANGLE_PROCESS_H_

//typedef int int16_t;
#include<stdint.h>
#include<array>


int16_t data_process(int16_t left_boundary0, int16_t right_boundary0,
		int16_t left_boundary1, int16_t right_boundary1, uint8_t min0,
		uint8_t max0, uint8_t min1, uint8_t max1, int16_t angle,
		int16_t min_angle, int16_t max_angle, float kpr0, float kpr1, float kpr2,
		float kpl0, float kpl1, float kpl2,
		std::array<short unsigned int, 128> &lower_Ccd_data);



#endif /* SRC_ANGLE_PROCESS_H_ */
