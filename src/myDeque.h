/*
 * myDeque.h
 *
 *  Created on: 2 Jun 2016
 *      Author: ChungWa
 */
#include <cassert>
#include <cstring>
#include <array>
#include <deque>
#include <cstdint>
#include <functional>
#include <cmath>

#ifndef SRC_MYDEQUE_H_
#define SRC_MYDEQUE_H_

using namespace std;

class myDeque {
public:
	myDeque(const uint8_t no);
	void update(int32_t last);
	int32_t average();
private:
	deque<int32_t> data;
	uint8_t number;
};

#endif /* SRC_MYDEQUE_H_ */
