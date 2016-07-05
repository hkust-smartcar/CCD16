/*
 * myDeque.cpp
 *
 *  Created on: 2 Jun 2016
 *      Author: ChungWa
 */

#include "myDeque.h"
#include <cassert>
#include <cstring>
#include <array>
#include <deque>
#include <cstdint>
#include <functional>
#include <cmath>

myDeque::myDeque(const uint8_t no)
{
	for(int i=0; i<no; i++)
		data.push_back(0);
	number = no;
	for(int i=1; i<=no; i++){
		sumOfNumber += i;
	}
}

void myDeque::update(int32_t last)
{
	data.pop_front();
	data.push_back(last);

}

int32_t myDeque::average()
{
	int32_t average = 0;
	for(int i=0; i<number; i++){
		average += data.at(i) * ( i + 1 );
	}
	average /= sumOfNumber;
	return average;
}


