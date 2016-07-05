/*
 * angle_process.cpp
 *
 *  Created on: 2016年7月4日
 *      Author: kwanhoman
 */

/*
 * angler_process.cpp
 *
 *  Created on: 2016年7月4日
 *      Author: kwanhoman
 */

#include "angle_process.h"

int16_t data_process(int16_t left_boundary0, int16_t right_boundary0,
		int16_t left_boundary1, int16_t right_boundary1, uint8_t min0,
		uint8_t max0, uint8_t min1, uint8_t max1, int16_t angle,
		int16_t min_angle, int16_t max_angle, float kpr0, float kpr1, float kpr2,
		float kpl0, float kpl1, float kpl2,
		std::array <short unsigned int, 128> &lower_Ccd_data) {
	bool right_boundary_found[2] = { 0 };
	bool left_boundary_found[2] = { 0 };
	int32_t sum_l = 0;
	int32_t sum_r = 0;
	float target = 0.5;
	int16_t PATH_WIDE0 = 140;
	int16_t PATH_WIDE1 = 140;
	int8_t case_id = 0;
	int16_t e = 0;

	float turning_Kp_r[3] = { kpr0, kpr1, kpr2 };
//	float turning_Kd_r[3] = { kdr0, kdr1, kdr2 };
	float turning_Kp_l[3] = { kpl0, kpl1, kpl2 };
//	float turning_Kd_l[3] = { kdl0, kdl1, kdl2 };

	if (left_boundary0 == min0) {
		left_boundary_found[0] = false;
	} else {
		left_boundary_found[0] = true;
	}

	if (left_boundary1 == min1) {
		left_boundary_found[1] = false;
	} else {
		left_boundary_found[1] = true;
	}

	if (right_boundary0 == max0) {
		right_boundary_found[0] = false;
	} else {
		right_boundary_found[0] = true;
	}

	if (right_boundary1 == max1) {
		right_boundary_found[1] = false;
	} else {
		right_boundary_found[1] = true;
	}

	for (int i = 0 + 15; i < 60; i++) {
		sum_l += lower_Ccd_data[i];
	}

	for (int i = 68; i < 127 - 15; i++) {
		sum_r += lower_Ccd_data[i];
	}

	if (right_boundary_found[1] && left_boundary_found[1]) {
		if ((right_boundary1 <= left_boundary1)
				|| (right_boundary1 - left_boundary1 < 30)) {
			if ((float) sum_l / sum_r >= 1.3)
				left_boundary_found[1] = false;
			else if ((float) sum_r / sum_l >= 1.3)
				right_boundary_found[1] = false;
			else if (angle > max_angle - 20) {
				left_boundary_found[1] = false;
			} else if (angle < min_angle + 20) {
				right_boundary_found[1] = false;
			} else {
				right_boundary_found[1] = left_boundary_found[1] = false;
			}
		}
	}

	if (right_boundary_found[0] && left_boundary_found[0]) {
		if ((right_boundary0 <= left_boundary0)
				|| (right_boundary0 - left_boundary0 < 30)) {
			if (sum_l / sum_r >= 1.3)
				left_boundary_found[0] = false;
			else if (sum_r / sum_l >= 1.3)
				right_boundary_found[0] = false;
			else {
				right_boundary_found[0] = left_boundary_found[0] = false;

			}
		}
	}
	if (right_boundary_found[1] || left_boundary_found[1]) {
		right_boundary_found[0] &= right_boundary_found[1];
		left_boundary_found[0] &= left_boundary_found[1];
	}

	if (right_boundary_found[0] && right_boundary_found[1]
			&& !left_boundary_found[0] && !left_boundary_found[1]) {
		if (((64 - (-((1 - target) * PATH_WIDE1) + right_boundary1))

		> (64 - (-((1 - target) * PATH_WIDE0) + right_boundary0))
				&& 64 - (-((1 - target) * PATH_WIDE1) + right_boundary1) > 0)
				|| ((64 - (-((1 - target) * PATH_WIDE1) + right_boundary1))
						< (64 - (-((1 - target) * PATH_WIDE0) + right_boundary0))
						&& 64 - (-((1 - target) * PATH_WIDE0) + right_boundary1)
								< 0)) {
			right_boundary_found[0] = false;
		}
	}

	else if (!right_boundary_found[0] && !right_boundary_found[1]
			&& left_boundary_found[0] && left_boundary_found[1]) {
		if (((63 - target * PATH_WIDE1 - left_boundary1)
				> (63 - target * PATH_WIDE1 - left_boundary0)
				&& 63 - target * PATH_WIDE1 - left_boundary1 > 0)
				|| ((63 - target * PATH_WIDE1 - left_boundary1)
						< (63 - target * PATH_WIDE0 - left_boundary0)
						&& 63 - target * PATH_WIDE1 - left_boundary1 < 0)) {
			left_boundary_found[0] = false;
		}
	}

//	if (right_boundary_found[0] ^ left_boundary_found[0]) {
//		if (!right_boundary_found[1] && !left_boundary_found[1]) {
//			switch (record.get_dir()) {
//			case 'l':
//				left_boundary_found[0] = false;
//				if(right_boundary0>70)right_boundary_found[0]=false;
//				break;
//			case 'r':
//				right_boundary_found[0] = false;
//				if(left_boundary0<57)left_boundary_found[0]=false;
//				break;
//
//			}
//		}
//
//		if (sum_l / sum_r >= 1.3 && sum_l > 6000)
//			left_boundary_found[1] = false;
//		else if (sum_r / sum_l >= 1.3 && sum_r > 6000)
//			right_boundary_found[1] = false;
//	}
	if (!right_boundary_found[1] && !left_boundary_found[1]) {
		if ((float) sum_l / sum_r >= 1.3 && sum_l > 5000)
			left_boundary_found[0] = false;
		else if ((float) sum_r / sum_l >= 1.3 && sum_r > 5000)
			right_boundary_found[0] = false;
		else {
//			switch (record.get_dir()) {
//			case 'l':
//				left_boundary_found[0] = false;
//				break;
//			case 'r':
//				right_boundary_found[0] = false;
//				break;
//
//			}
		}
	}
//	else if ((!left_boundary_found[0] && !right_boundary_found[0])) {
//		if ((float) sum_l / sum_r >= 2.2 && sum_l > 5000) {
//			left_boundary_found[1] = false;
//		} else if ((float) sum_r / sum_l >= 2.2 && sum_r > 5000) {
//			right_boundary_found[1] = false;
//		}
	//}

	if ((left_boundary_found[0] && right_boundary_found[0])
			&& (left_boundary_found[1] && right_boundary_found[1])) {
		if ((right_boundary0 - 64) - (63 - left_boundary0) > 35) {
			right_boundary_found[0] = false;
		}
		if ((63 - left_boundary0) - (right_boundary0 - 64) > 35) {
			left_boundary_found[0] = false;
		}

	}

	case_id = 0;
	if ((!left_boundary_found[0] && !right_boundary_found[0])
			|| (left_boundary_found[0] && right_boundary_found[0])) {

		if (left_boundary_found[1] && right_boundary_found[1]
				&& left_boundary1 < right_boundary1) {	//L1 & R1 only

			e = 64 - (left_boundary1 + right_boundary1) * target;

			if (left_boundary_found[0] && right_boundary_found[0]) {
				float t = 64 - (left_boundary0 + right_boundary0) * target;
				if (t - e < 50 && t - e > -50
						&& ((t > 0 && e >= 0) || (t < 0 && e <= 0))) {
					e = 2 * t;
				}
			}

			case_id = 0;
		} else if (left_boundary_found[1] && !right_boundary_found[1]) {//L1 only
			e = 63 - target * PATH_WIDE1 - left_boundary1;
			case_id = 2;
		} else if (!left_boundary_found[1] && right_boundary_found[1]) {//R1 only
			e = 64 - (-((1 - target) * PATH_WIDE1) + right_boundary1);
			case_id = 2;
		} else if (left_boundary_found[0] && right_boundary_found[0]) {
			e = 64 - (left_boundary0 + right_boundary0) * target;
			case_id = 0;
		}

		else {

			if ((float) sum_l / sum_r >= 1.8 && sum_l > 6000) {
				//turn left
				angle = min_angle;
				return angle;
			} else if ((float) sum_r / sum_l >= 1.8 && sum_r > 6000) {
				//turn right
				angle = max_angle;
				return angle;
			} else if (sum_l + sum_r > 13000) {
				e = 0;
			} else {
//				switch (record.get_dir()) {
//				case 'r':
//					e = -100;
//					angle = MIN_ANGLE;
//					record.new_record(MIN_ANGLE);
//					return MIN_ANGLE;
//				case 'l':
//					e = 100;
//					angle = MAX_ANGLE;
//					record.new_record(MAX_ANGLE);
//					return MAX_ANGLE;
//				}
			}
			case_id = 0;
			e = 0;
		}
	} else if (left_boundary_found[0]) {	//L0 only

		e = 63 - target * PATH_WIDE0 - left_boundary0;
		case_id = 1;
	} else if (right_boundary_found[0]) {	//R0 only
		e = 64 - (-((1 - target) * PATH_WIDE0) + right_boundary0);
		case_id = 1;
	}

	if (e < 40 && e > -40) {
		e = e / 6;
	} else if (e >= 40) {
		e = (((e * e) / 60) - 20);
	} else if (e <= -40) {
		e = -(((e * e) / 60) - 20);
	}
	int16_t off_set=0;
	if (e >= 0) {
		off_set -= turning_Kp_r[case_id] * e
//				+ turning_Kd_r[case_id] * (e - previous_error)
				;
	} else {
		off_set -= turning_Kp_l[case_id] * e
//				+ turning_Kd_l[case_id] * (e - previous_error)
				;
	}

	return off_set;

}

