/*
 * main.cpp
 *
 * Author: Peter
 * Copyright (c) 2014-2015 HKUST SmartCar Team
 * Refer to LICENSE for details
 */

#include <cassert>
#include <cstring>
#include <array>
#include <cstdint>
#include <cmath>
#include <libbase/k60/mcg.h>
#include <libsc/system.h>
#include <libsc/ab_encoder.h>
#include <libsc/tsl1401cl.h>
#include <libsc/alternate_motor.h>
#include <libsc/futaba_s3010.h>
#include <libsc/lcd_console.h>
#include <libsc/lcd_typewriter.h>
#include <libsc/st7735r.h>
#include <libsc/button.h>
#include <libsc/joystick.h>
#include <libsc/led.h>
#include <libsc/k60/jy_mcu_bt_106.h>
#include <libbase/k60/gpio.h>
#include <libutil/string.h>
#include "car.h"

namespace libbase
{
	namespace k60
	{

		Mcg::Config Mcg::GetMcgConfig()
		{
			Mcg::Config config;
			config.external_oscillator_khz = 50000;
			config.core_clock_khz = 220000;
			return config;
		}

	}
}

using namespace libsc;
using namespace libbase::k60;

int main(void)
{
	System::Init();
	car car;

//	bool twiceLoopCounter = false;
	uint32_t previousTime = System::Time();
	while (true){
		if((System::Time()-previousTime)==10){
			previousTime = System::Time();
//			car.printSpeed();
//			car.eraceCcdData();

//			if(twiceLoopCounter==true){
//				car.ledSwitch(0);
//			}else{
//				car.ledSwitch(1);
//			}
			car.updateCcd();
			car.dirControl();
			car.updateEncoder();
			car.speedPID();
//			car.ledSwitch(0);
//			car.ledSwitch(1);

//			car.print();
//			car.printCcdData();

//			twiceLoopCounter = !twiceLoopCounter;
			}
		}
//	return 0;
}
