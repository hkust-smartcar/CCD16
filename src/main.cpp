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
			config.core_clock_khz = 150000;
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

	uint32_t previousTime = System::TimeIn100Us();
	while (true){
		if((System::TimeIn100Us()-previousTime)==100){
//			car.printSpeed();
//			car.eraceCcdData();
			car.dirControl();
			car.updateCcd();

			while((System::TimeIn100Us()-previousTime)<150){};

			car.updateEncoder();
			car.speedPID();
//			car.print();

//			car.printCcdData();
			previousTime = System::TimeIn100Us();
			}
		}
	return 0;
}
