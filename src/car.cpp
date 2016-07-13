/*
 * car.cpp
 *
 *  Created on: 21 Jan 2016
 *      Author: ChungWa
 */

#include "car.h"

#include <cassert>
#include <cstring>
#include <array>
#include <deque>
#include <cstdint>
#include <functional>
#include <cmath>
#include <libbase/k60/adc.h>
#include <libbase/k60/pin.h>
#include <libsc/ab_encoder.h>
#include <libsc/tsl1401cl.h>
#include <libsc/alternate_motor.h>
#include <libsc/futaba_s3010.h>
#include <libsc/st7735r.h>
#include <libsc/button.h>
#include <libsc/joystick.h>
#include <libsc/led.h>
#include <libsc/k60/jy_mcu_bt_106.h>
#include <libbase/k60/gpio.h>
#include <libbase/k60/pin.h>
#include <libsc/lcd_console.h>
#include "myDeque.h"
#include "pVarManager.h"
#include "angle_process.h"


using namespace std;
using namespace libsc;
using namespace libbase::k60;

car* car::m_instance=nullptr;

void car::listener(const std::vector<Byte>& data)
{
	char message[128];
	switch (data[0]) {
		case 'g':
			m_instance->targetCarSpeed = 1400;
			break;
		case 's':
			m_instance->targetCarSpeed = 0;
			break;
		default:
		;
	}
}

Led::Config getLedConfig(const uint8_t id)
{
	Led::Config config;
	config.id = id;
	return config;
}

Button::Config car::getButtonConfig(const uint8_t id)
{
	Button::Config config;
	config.id = id;
	config.is_active_low = true;
	config.listener_trigger = Button::Config::Trigger::kUp;
	config.listener =  [&](const uint8_t)
	{
//		accuLCount = 0;
//		accuRCount = 0;
//		isKP = !isKP;
//		allBlackConstant -=2;
		ccdMaxReady += 1;
		resetToNoObstacle();
//		printCcd = !printCcd;
//		lcd.Clear();

	};
	return config;
}

AbEncoder::Config getAbEncoderConfig(const uint8_t id)
{
	AbEncoder::Config config;
	config.id = id;
	return config;
}

Adc::Config getAdcConfig(Pin::Name name)
{
	Adc::Config config;
	config.pin = name;
	config.speed = Adc::Config::SpeedMode::kExFast;
	config.resolution = Adc::Config::Resolution::k12Bit;
	config.avg_pass = config.AveragePass::k32;
	return config;
}

AlternateMotor::Config getAlternateMotorConfig(const uint8_t id)
{
	AlternateMotor::Config config;
	config.id = id;
	return config;
}

FutabaS3010::Config getFutabaS3010Config(const uint8_t id)
{
	FutabaS3010::Config config;
	config.id = id;
	return config;
}

St7735r::Config getSt7735rConfig()
{
	St7735r::Config config;
	config.is_revert = false;
	config.is_bgr = false;
	return config;
}

Joystick::Config car::getJoystickConfig(const uint8_t id)
{
	Joystick::Config config;
	config.id = id;
	config.is_active_low = true;
	config.listener_triggers[0] = Joystick::Config::Trigger::kBoth;
	config.listener_triggers[1] = Joystick::Config::Trigger::kBoth;
	config.listener_triggers[2] = Joystick::Config::Trigger::kBoth;
	config.listener_triggers[3] = Joystick::Config::Trigger::kBoth;
	config.listener_triggers[4] = Joystick::Config::Trigger::kBoth;
	if(id==0){
		config.handlers[static_cast<int>(Joystick::State::kUp)] = [&](const uint8_t, const Joystick::State)
		{
			if(ccdMaxReady!=-100){
				ccdMaxReady += 1;
			}
//			if(targetCarSpeed==0){
//				targetCarSpeed = 890;
//			}else{
//				targetCarSpeed = 0;
//			}
	//		allBlackConstant += 2;
	//		servoTo(angleError += 5);
	//		if(isKP == true){
	//			angleKP0 += 0.2;
	//		}else{
	//			angleKD1 += 0.2;
	//		}
	//		offset += 1;
		};
		config.handlers[static_cast<int>(Joystick::State::kDown)] = [&](const uint8_t, const Joystick::State)
		{
			if(thirdCcdMaxReady!=100){
			thirdCcdMaxReady += 1;
			}
	//		allBlackConstant -= 2;
	//		servoTo(angleError -= 5);
	//		if(isKP == true){
	//			angleKP0 -= 0.2;
	//		}else{
	//			angleKD1 -= 0.2;
	//		}
	//		offset -= 1;
		};
		config.handlers[static_cast<int>(Joystick::State::kLeft)] = [&](const uint8_t, const Joystick::State)
		{
	//		errorIndex -=0.05;
		};
		config.handlers[static_cast<int>(Joystick::State::kRight)] = [&](const uint8_t, const Joystick::State)
		{
	//		errorIndex +=0.05;
		};
		config.handlers[static_cast<int>(Joystick::State::kSelect)] = [&](const uint8_t, const Joystick::State)
		{
			resetToNoObstacle();
		};
	}else{
		config.handlers[static_cast<int>(Joystick::State::kUp)] = [&](const uint8_t, const Joystick::State)
		{
//			if(ccdDataCounter==0){
//				ccdDataCounter = 1;
//			}else if(ccdDataCounter==2){
//				ccdDataCounter = 3;
//			}else if(ccdDataCounter==4){
//				ccdDataCounter = 5;
//			}else if(ccdDataCounter==6){
//				ccdDataCounter = 7;
//			}
//
//			ccdDataRecorded = false;
			if(ccdMinReady!=-100){
				ccdMinReady += 1;
			}
		};
		config.handlers[static_cast<int>(Joystick::State::kDown)] = [&](const uint8_t, const Joystick::State)
		{
			if(thirdCcdMinReady!=100){
				thirdCcdMinReady += 1;
			}
//			if(ccdDataCounter==1){
//				ccdDataCounter = 2;
//			}else if(ccdDataCounter==3){
//				ccdDataCounter = 4;
//			}else if(ccdDataCounter==5){
//				ccdDataCounter = 6;
//			}
//
//			ccdDataRecorded = false;
		};
		config.handlers[static_cast<int>(Joystick::State::kLeft)] = [&](const uint8_t, const Joystick::State)
		{
		};
		config.handlers[static_cast<int>(Joystick::State::kRight)] = [&](const uint8_t, const Joystick::State)
		{
		};
		config.handlers[static_cast<int>(Joystick::State::kSelect)] = [&](const uint8_t, const Joystick::State)
		{
		};
	}
	return config;
}

LcdConsole::Config car::getLcdConsoleConfig()
{
	LcdConsole::Config config;
	config.lcd = &lcd;
	config.region = Lcd::Rect(0, 0, 128, 160);
	return config;
}

//libsc::k60::JyMcuBt106::Config getJyMcuBt106Config(const uint8_t id)
//{
//	libsc::k60::JyMcuBt106::Config config;
//	config.id = id;
//	config.baud_rate = Uart::Config::BaudRate::k115200;
//	config.tx_buf_size = 200;
//	config.rx_isr = &car::m_instance->listener;
//	return config;
//}

car::car()
:
	currentLeftSpeedSq(4),
	currentRightSpeedSq(4),
	D1(getLedConfig(0)),
	D2(getLedConfig(1)),
	D3(getLedConfig(2)),
	D4(getLedConfig(3)),
//	sw(getButtonConfig(0)),
	encoderL(getAbEncoderConfig(0)),
	encoderR(getAbEncoderConfig(1)),
//	lCurrent(getAdcConfig(Pin::Name::kPtb10)),
//	rCurrent(getAdcConfig(Pin::Name::kPtb11)),
	ccd{Tsl1401cl(0), Tsl1401cl(1), Tsl1401cl(2)},
	servo(getFutabaS3010Config(0)),
	motorL(getAlternateMotorConfig(0)),
	motorR(getAlternateMotorConfig(1)),
	lcd(getSt7735rConfig()),
	joystick{Joystick(getJoystickConfig(0)),Joystick(getJoystickConfig(1))},
	console(getLcdConsoleConfig())
//	,bluetooth(getJyMcuBt106Config(0))
{
//	if (m_instance)
//		assert(false);
	m_instance = this;

	D1.SetEnable(1);
	D2.SetEnable(1);
	D3.SetEnable(1);
	D4.SetEnable(1);
	motorL.SetPower(LPWM);
	motorL.SetClockwise(false);
	motorR.SetPower(RPWM);
	motorR.SetClockwise(false);
	servoTo(angleError);//need to change the default degree
	lcd.Clear(0); // Clear the screen to black
	//default setting

//	pGrapher.addWatchedVar(&accuRCount, "accuRCount");
//	pGrapher.addWatchedVar(&accuLCount, "accuLCount");
	pGrapher.addWatchedVar(&currentLeftSpeed, "currentLeftSpeed");
	pGrapher.addWatchedVar(&currentRightSpeed, "currentRightSpeed");
	pGrapher.addWatchedVar(&targetLeftSpeed, "targetLeftSpeed");
	pGrapher.addWatchedVar(&targetRightSpeed, "targetRightSpeed");
//	pGrapher.addWatchedVar(&LPWM, "LPWM");
//	pGrapher.addWatchedVar(&RPWM, "RPWM");
//	pGrapher.addWatchedVar(&centreError[0], "centreError[0]");
	pGrapher.addWatchedVar(&centreError[1], "centreError[1]");
//	pGrapher.addWatchedVar(&max[1], "max[1]");
//	pGrapher.addWatchedVar(&angleError, "angleError");

//	pGrapher.addWatchedVar(&max[1], "max[1]");
//	pGrapher.addWatchedVar(&centreError[1], "centreError[1]");
//	pGrapher.addWatchedVar(&leftCurrent, "leftCurrent");
//	pGrapher.addWatchedVar(&rightCurrent, "rightCurrent");
//	pGrapher.addSharedVar(&targetCarSpeed, "targetCarSpeed");
//	pGrapher.addSharedVar(&sKP_l, "sKP_l");
//	pGrapher.addSharedVar(&sKI_l, "sKI_l");
//	pGrapher.addSharedVar(&sKP_r, "sKP_r");
//	pGrapher.addSharedVar(&sKI_r, "sKI_r");
	pGrapher.addSharedVar(&angleKP0Left, "angleKP0Left");
	pGrapher.addSharedVar(&angleKP0Right, "angleKP0Right");
//	pGrapher.addSharedVar(&maxServoAngleForSpeed, "maxServoAngleForSpeed");
	pGrapher.addSharedVar(&angleKP1, "angleKP1");
	pGrapher.addSharedVar(&angleKD1, "angleKD1");
	pGrapher.addSharedVar(&angleKD0Left, "angleKD0Left");
	pGrapher.addSharedVar(&angleKD0Right, "angleKD0Right");
	pGrapher.addSharedVar(&maxErrorForChangingSpeed, "maxErrorForChangingSpeed");
	pGrapher.addSharedVar(&straightLineSpeed, "straightLineSpeed");
//	pGrapher.addSharedVar(&angleKP1, "angleKP1");
//	pGrapher.addSharedVar(&currentBrakingTime, "currentBrakingTime");
	pGrapher.SetOnReceiveListener(&listener);
}

void car::updateCcd()
{

	for(int i=0; i<3; i++){
		while(!ccd[i].SampleProcess()){}
		ccdData.at(i) = ccd[i].GetData();
		ccd[i].StartSample();
	}

//	if(!ccdDataRecorded){
//		if(ccdDataCounter==1){
//			D4.SetEnable(0);
//		}else if(ccdDataCounter==2){
//			D3.SetEnable(0);
//		}else if(ccdDataCounter==3){
//			D3.SetEnable(0);
//			D4.SetEnable(0);
//		}else if(ccdDataCounter==4){
//			D2.SetEnable(0);
//		}else if(ccdDataCounter==5){
//			D2.SetEnable(0);
//			D4.SetEnable(0);
//		}else if(ccdDataCounter==6){
//			D2.SetEnable(0);
//			D3.SetEnable(0);
//		}
//		if(!((ccdDataCounter-1)/3)){
//			for(int i=0; i<128; i++)
//				ccdMax[(ccdDataCounter-1)%3][i] = ccdData.at(ccdDataCounter).at(i);
//		}else{
//			for(int i=0; i<128; i++)
//				ccdMin[(ccdDataCounter-1)%3][i] = ccdData.at(ccdDataCounter).at(i);
//		}
//		ccdDataRecorded = true;
//
//	}else if(ccdDataCounter==7){
//		if(obstacle){
//			D1.SetEnable(0);
//		}else{
//			D1.SetEnable(1);
//		}
//		if(afterObstacle){
//			D2.SetEnable(0);
//		}else{
//			D2.SetEnable(1);
//		}
//	}
//	//D1 first blinking indicates max recorded, D1 then not blinking indicates min recorded.
//
//	if(ccdMax[0]!=NULL&&ccdMax[1]!=NULL&&ccdMin[0]!=NULL&&ccdMin[1]!=NULL){
//
//		if(allBlackConstant==0){
//			allBlackConstant = 230 / 4;
//		}
//
//		for(int j=0; j<3; j++){
//
//			int16_t data[128] = { NULL };
//
//			for(int i=0; i<128; i++){
//				if(ccdData.at(j).at(i)<ccdMin[j][i]){
//					data[i] = 0;
//				}else if(ccdData.at(j).at(i)>ccdMax[j][i]){
//					data[i] = 230;
//				}else{
//					data[i] = ( ccdData.at(j).at(i) - ccdMin[j][i] ) * ( ccd0MaxMax ) / ( ccdMax[j][i] - ccdMin[j][i] );
//				}
//			}
//
//			for(int i=0; i<128; i++){
//				if(i!=127){
//					ccdData.at(j).at(i) = ( data[i] + data[i+1] ) / 2;
//				}
//			}
//		}
//	}

	if(ccdMinHasBeenChanged==false){
		if(ccdMaxHasBeenChanged==true){
			D1.SetEnable(0);
		}
	}else if(thirdCcdMinHasBeenChanged==false){
		if(thirdCcdMaxHasBeenChanged==true){
			D2.SetEnable(0);
		}
	}else{
		if(obstacle){
			D1.SetEnable(0);
		}else{
			D1.SetEnable(1);
		}
		if(afterObstacle){
			D2.SetEnable(0);
		}else{
			D2.SetEnable(1);
		}
	}
		//D1 first blinking indicates max recorded, D1 then not blinking indicates min recorded.

	if(ccdMaxReady>0){
		for(int i=0; i<128; i++)
			ccd0Max[i] = ccdData.at(0).at(i);
		for(int i=0; i<128; i++)
			ccd1Max[i] = ccdData.at(1).at(i);
		ccdMaxHasBeenChanged = true;
		ccdMaxReady = -100;
	}
	if(ccdMinReady>0){
		for(int i=0; i<128; i++)
			ccd0Min[i] = ccdData.at(0).at(i);
		for(int i=0; i<128; i++)
			ccd1Min[i] = ccdData.at(1).at(i);
		ccdMinHasBeenChanged = true;
		ccdMinReady = -100;
		D1.SetEnable(1);
	}
	if(thirdCcdMaxReady>0){
		for(int i=0; i<128; i++)
			ccd2Max[i] = ccdData.at(2).at(i);
		thirdCcdMaxHasBeenChanged = true;
		thirdCcdMaxReady = -100;
	}
	if(thirdCcdMinReady>0){
		for(int i=0; i<128; i++)
			ccd2Min[i] = ccdData.at(2).at(i);
		thirdCcdMinHasBeenChanged = true;
		thirdCcdMinReady = -100;
		D2.SetEnable(1);
	}
	//update 3 ccd
	if(ccd0Max!=NULL&&ccd0Min!=NULL&&ccd1Max!=NULL&&ccd1Min!=NULL){
		if(allBlackConstant==0){
			allBlackConstant = 230 / 4;
		}
		uint16_t data[128] = { NULL };
		for(int i=0; i<128; i++){
			if(ccdData.at(0).at(i)<ccd0Min[i]){
				data[i] = 0;
			}else if(ccdData.at(0).at(i)>ccd0Max[i]){
				data[i] = 230;
			}else{
				data[i] = ( ccdData.at(0).at(i) - ccd0Min[i] ) * ( ccd0MaxMax ) / ( ccd0Max[i] - ccd0Min[i] );
			}
		}
		for(int i=0; i<128; i++){
			if(i!=0||i!=127){
				ccdData.at(0).at(i) = ( data[i+1] + data[i] + data[i-1] ) / 3;
			}
		}


		for(int i=0; i<128; i++){
			if(ccdData.at(1).at(i)<ccd0Min[i]){
				data[i] = 0;
			}else if(ccdData.at(1).at(i)>ccd0Max[i]){
				data[i] = 230;
			}else{
				data[i] = ( ccdData.at(1).at(i) - ccd1Min[i] ) * ( ccd1MaxMax ) / ( ccd1Max[i] - ccd1Min[i] );
			}
		}
		for(int i=0; i<128; i++){
			if(i!=0||i!=127){
				ccdData.at(1).at(i) = ( data[i+1] + data[i] + data[i-1]) / 3;
			}
		}
	}

}

void car::updateEncoder()
{
	encoderL.Update();
	encoderR.Update();
	if(abs(encoderL.GetCount())>20000){
	}
//	else if(abs(currentLeftSpeed-(-encoderL.GetCount()))>900){
//	}
	else{
		currentLeftSpeedSq.update((int32_t)( - encoderL.GetCount() / 2 ));
		currentLeftSpeed = currentLeftSpeedSq.average();

//		currentLeftSpeed = (int32_t)( - encoderL.GetCount() );

//		accuLCount += currentLeftSpeed;
	}
	if(abs(encoderR.GetCount())>20000){
	}
//	else if(abs(currentRightSpeed-(encoderR.GetCount()))>900){
//	}
	else{
		currentRightSpeedSq.update((int32_t)( encoderR.GetCount() / 2 ) );
		currentRightSpeed = currentRightSpeedSq.average();

//		currentRightSpeed = (int32_t)( encoderR.GetCount() );

//		accuRCount += currentRightSpeed;
	}

//	leftCurrent = 2045-lCurrent.GetResult();
//	rightCurrent = rCurrent.GetResult() - 2045;
	//enconder update

	if(afterObstacle){
		distanceAfterSeeingObstacle += ( currentRightSpeed+currentLeftSpeed ) / 2;
	}

	pGrapher.sendWatchData();
}

void car::ledSwitch(const uint8_t id)
{
	switch (id)
		{
		case 0:
			D1.Switch(); break;

		case 1:
			D2.Switch(); break;

		case 2:
			D3.Switch(); break;

		case 3:
			D4.Switch(); break;
		default:
			;
		}
}

void car::ledOn(const uint8_t id)
{
	switch (id)
		{
		case 0:
			D1.SetEnable(0); break;

		case 1:
			D2.SetEnable(0); break;

		case 2:
			D3.SetEnable(0); break;

		case 3:
			D4.SetEnable(0); break;
		default:
			;
		}
}

void car::printCcdData()
{
//	for(int i=0; i<128; i++){
//		lcd.SetRegion(Lcd::Rect(i, 32 + (127 - 127 * ccdData.at(ccdId).at(i) / 255), 1, 1));
//		lcd.FillColor(0xFFFF);
//	}
//	previousCcdId = ccdId;


//		int no = 0;
//		for(ccdId=0; ccdId<2; ccdId++){
//			for(int i=(128-ccdLength[ccdId])/2; i<128-((128-ccdLength[ccdId])/2); i++){
//				lcd.SetRegion(Lcd::Rect(no*128/ccdLength[ccdId], (79 - 79 * ccdData.at(ccdId).at(i) / 255) + 80 * ccdId, 1, 1));
//				lcd.FillColor(0xFFFF);
//				no++;
//			}
//			no = 0;
//		}
		//print 2 ccd data
	for(int i=ccdLength[printCcd][0]; i<ccdLength[printCcd][1]; i++){
		lcd.SetRegion(Lcd::Rect(i, (79 - 79 * ccdData.at(printCcd).at(i) / 255) + 80, 1, 1));
		lcd.FillColor(0xFFFF);
	}


//		lcd.SetRegion(Lcd::Rect(0, (79 - 79 * threshold[0] / 255), 128, 1));
//		lcd.FillColor(0xFFFF);
//		lcd.SetRegion(Lcd::Rect(0, (79 - 79 * threshold[1] / 255) + 80, 128, 1));
//		lcd.FillColor(0xFFFF);

		lcd.SetRegion(Lcd::Rect(centre[printCcd], 80, 1, 80));
		lcd.FillColor(0xF800);
		lcd.SetRegion(Lcd::Rect(0, (79 - 79 * threshold[printCcd] / 255) + 80, 128, 1));
		lcd.FillColor(0xFFE0);
		lcd.SetRegion(Lcd::Rect(0, (79 - 79 * allBlackConstant / 255) + 80, 128, 1));
		lcd.FillColor(0x07FF);
		lcd.SetRegion(Lcd::Rect(trackCentre, 50, 1, 30));
		lcd.FillColor(0x001F);

//		lcd.SetRegion(Lcd::Rect(centre[1], 80, 1, 80));
//		lcd.FillColor(0xF800);
//		previousCcdId = ccdId;

}

void car::eraceCcdData()
{
//	for(int i=0; i<128; i++){
//		lcd.SetRegion(Lcd::Rect(i, 32 + (127 - 127 * ccdData.at(previousCcdId).at(i) / 255), 1, 1));
//		lcd.FillColor(0);
//	}

//		int no = 0;
//		for(ccdId=0; ccdId<2; ccdId++){
//			for(int i=(128-ccdLength[ccdId])/2; i<128-((128-ccdLength[ccdId])/2); i++){
//				lcd.SetRegion(Lcd::Rect(no*128/ccdLength[ccdId], (79 - 79 * ccdData.at(ccdId).at(i) / 255) + 80 * ccdId, 1, 1));
//				lcd.FillColor(0);
//				no++;
//			}
//			no = 0;
//		}
		//erace 2 ccd data

	for(int i=ccdLength[printCcd][0]; i<ccdLength[printCcd][1]; i++){
		lcd.SetRegion(Lcd::Rect(i, (79 - 79 * ccdData.at(printCcd).at(i) / 255) + 80, 1, 1));
		lcd.FillColor(0);
	}

//		lcd.SetRegion(Lcd::Rect(0, (79 - 79 * threshold[0] / 255), 128, 1));
//		lcd.FillColor(0);
//		lcd.SetRegion(Lcd::Rect(0, (79 - 79 * threshold[1] / 255) + 80, 128, 1));
//		lcd.FillColor(0);
		lcd.SetRegion(Lcd::Rect(centre[printCcd], 80, 1, 80));
		lcd.FillColor(0);
		lcd.SetRegion(Lcd::Rect(0, (79 - 79 * threshold[printCcd] / 255) + 80, 128, 1));
		lcd.FillColor(0);
//		lcd.SetRegion(Lcd::Rect(centre[1], 80, 1, 80));
//		lcd.FillColor(0);

}

void car::print()
{

//	char message[128];
//	sprintf(message,"%d\t%d\n%d\t%d\n%d\t%d\n%d\t%d\n%d\t%d\n\n\n\n\n\n",targetLeftSpeed, targetRightSpeed, currentLeftSpeed, currentRightSpeed ,lError ,rError ,accuLeftError ,accuRightError ,LPWM ,RPWM );
//	console.WriteString(message);

	char message[128];
	sprintf(message,"\n%f\n%d\n%d\n\n\n\n\n\n\n", errorIndex, centreError[0], angleError);
	console.WriteString(message);

//	char message[128];
//	sprintf(message,"%f\n%d\n%d\n\n\n\n\n\n\n\n", angleKP0, accuLCount, accuRCount);
//	console.WriteString(message);

}

void car::servoTo(int16_t degree)
{
	if(degree>maxServoAngleToRight){
		angleError = maxServoAngleToRight;
	}else if(degree<-maxServoAngleToLeft){
		angleError = -maxServoAngleToLeft;
	}
	servo.SetDegree(884 - angleError);
}

void car::resetToNoObstacle(){
	obstacle = false;
	afterObstacle = false;
	distanceAfterSeeingObstacle = 0;
}

void car::dirControl()
{
	for(ccdId=0; ccdId<3; ccdId++){
		int16_t maxData = 0;
		int16_t minData = 255;
		for(int j=ccdLength[ccdId][0]; j<ccdLength[ccdId][1]; j++){//find max and min
			if(ccdData.at(ccdId).at(j)>maxData){
				maxData = ccdData.at(ccdId).at(j);
			}
			if(ccdData.at(ccdId).at(j)<minData){
				minData = ccdData.at(ccdId).at(j);
			}
		}

		max[ccdId] = maxData;

		if(ccdId==0){
			if(maxData<=allBlackConstant){//ccd0 seeing nothing, then refer back to previous centre and turn to the most
				if(centre[0]>64){
					centre[0] = 127;
				}else{
					centre[0] = 0;
				}
				continue;//keep update ccd1 or you can use break
			}else if(minData>allBlackConstant){//ccd0 seeing all white
				centre[0] = 64;
				continue;
			}
		}else if(ccdId==1){//other ccd
			if(maxData<allBlackConstant){// ccd1 being black
				centre[1] = 0;// will not accelerate
				break;
			}else if(minData>allBlackConstant){
				centre[1] = 0;// will not accelerate
				break;
			}
		}

		threshold[ccdId] = ( maxData + minData ) / 2;
		//find threshold

		bool prePixelIsHigh = false;
		noOfSegment[ccdId] = 0;
		for(int j=ccdLength[ccdId][0]; j<ccdLength[ccdId][1]; j++){
			if(ccdData.at(ccdId).at(j)>=threshold[ccdId]){
				if(prePixelIsHigh == false){//from black to white
					noOfSegment[ccdId]++;
				}
				prePixelIsHigh = true;
			}else{
				prePixelIsHigh = false;
			}
		}
		//find no. of white segment

		prePixelIsHigh = false;//assume black first
		uint16_t edge[noOfSegment[ccdId]][2];
		uint8_t centreOfSegment[noOfSegment[ccdId]];
		uint16_t distance[noOfSegment[ccdId]];
		uint8_t indexOfSegment = 0;//index of first segment
		for(int j=ccdLength[ccdId][0]; j<ccdLength[ccdId][1]; j++){
			if(ccdData.at(ccdId).at(j)>=threshold[ccdId]){
				if(prePixelIsHigh == false){// from black to white edge
					edge[indexOfSegment][0] = j;//starting edge
				}
				prePixelIsHigh = true;
				if(j==(ccdLength[ccdId][1]-1)){//last pixel in each ccd
					edge[indexOfSegment][1] = (ccdLength[ccdId][1]-1);
				}
			}else{
				if(prePixelIsHigh == true){
					edge[indexOfSegment][1] = --j;//ending edge which is the previous index
					indexOfSegment++;//go to next segment
				}
				prePixelIsHigh = false;
			}
		}

		for(int i=0; i<noOfSegment[ccdId]; i++){//centre of each segment
			centreOfSegment[i] = ( edge[i][0] + edge[i][1] ) / 2;
		}
		for(int i=0; i<noOfSegment[ccdId]; i++){//distance between centres and preCentre
			distance[i] = abs((preCentre[ccdId]-centreOfSegment[i]));
		}
		int minSegment = 0;
		for(int i=0; i<noOfSegment[ccdId]; i++){//find the one that is the closest to 64
			if(distance[i]<=distance[minSegment]){
				minSegment = i;
			}
		}


		if(abs(preCentre[ccdId]-edge[minSegment][0])>(ccdLength[ccdId][1]-ccdLength[ccdId][0])*5/9){// missed the track and saw the other track
			//***use the previous centre***
		}else{
			centre[ccdId] = centreOfSegment[minSegment];
		}

		if(ccdId==0){
			if(noOfSegment[0]>1){
				for(int i=0;i<noOfSegment[0]-1;i++){
					if((edge[i+1][0]-edge[i][1])>8 && (edge[i+1][0]-edge[i][1])<15){
						obstacle=true;
						if((edge[i][1]-edge[i][0])>(edge[i+1][1]-edge[i+1][0])){
							obstacleLeft=false;
							afterObstacleCentreShift=(edge[i][1]-edge[i][0])/2;
							minSegment=i;
						}else{
							obstacleLeft=true;
							afterObstacleCentreShift=(edge[i+1][1]-edge[i+1][0])/2;
							minSegment=i+1;
						}
					}else{
						if(obstacle){
							obstacle=false;
							afterObstacle=true;
						}
					}
				}
			}else{
				if(obstacle){
					obstacle=false;
					afterObstacle=true;
				}
			}
			if(afterObstacle){
				if(distanceAfterSeeingObstacle > countsPerM*0.3){
					afterObstacle=false;
					distanceAfterSeeingObstacle=0;
				}
			}
			if(afterObstacle){
				if(obstacleLeft){
					centre[0] = edge[minSegment][1] - afterObstacleCentreShift;
				}else{
					centre[0] = edge[minSegment][0] + afterObstacleCentreShift;
				}
			}
		}


	}



//	switch(controlCase){
//		case 0:
			centreError[0] = ( centre[0] - trackCentre );
			centreError[1] = centre[1] - trackCentre;
			if(centreError[0]<0){
//				if((centre[0]-preCentre[0])>0){
//					angleError = centreError[0] * angleKP0Left;
//				}else{
					angleError = centreError[0] * angleKP0Left + ( centre[0] - preCentre[0] ) * angleKD0Left + centreError[1] *  + ( centre[1] - preCentre[1] ) * angleKD1;
//				}
			}else{
//				if((centre[0]-preCentre[0])<0){
//					angleError = centreError[0] * angleKP0Left;
//				}else{
					angleError = centreError[0] * angleKP0Right + ( centre[0] - preCentre[0] ) * angleKD0Right + centreError[1] * angleKP1 + ( centre[1] - preCentre[1] ) * angleKD1;
//				}
			}
			changingSpeed();
//			if(centreError<0){
//				if(abs(centreError)<12){
//					angleError = centreError * 6;
//				}else if(abs(centreError)<20){
//					angleError = centreError * 6;
//				}else if(abs(centreError)<25){
//					angleError = centreError * 6;
//				}else{
//					angleError = centreError * 6;
//				}
//			}else{
//				if(abs(centreError)<12){
//					angleError = centreError * 6.3;
//				}else if(abs(centreError)<20){
//					angleError = centreError * 6.3;
//				}else if(abs(centreError)<25){
//					angleError = centreError * 6.3;
//				}else{
//					angleError = centreError * 6.3;
//				}
//			}


//			}else if(abs(centreError)<18){
//				angleError = centreError * 5;
//			}else if(abs(centreError)<24){
//				angleError = centreError * 5.8;
//			}else{
//				angleError = centreError * 8;
//			}
//			int8_t sign = ( centre[0] - trackCentre ) / abs( centre[0] - trackCentre );
//			break;
//		case 1:
//			straightLine:
//			centreError = ( centre[1] - trackCentre );
//			angleError = centreError * angleKP1 + ( preCentre[1] - 64 ) * angleKD1;
//			break;
//	}


//	if(abs(centre[0]-trackCentre)<=10){
//		wheelRatio = 1;
//		angleError = ( centre[1] - trackCentre ) * angleKP;
//	}else if(sign<0){
//		wheelRatio = - twoWheelsRatio[trackCentre - centre[0] - 1];
//		angleError = - turningLeft_errorToServo[trackCentre - centre[0] - 1];
//	}else{
//		wheelRatio = twoWheelsRatio[centre[0] - trackCentre - 1];
//		angleError = turningRight_errorToServo[centre[0] - trackCentre - 1];
//	}
	//old one

//	wheelRatio = sign*(  pow( ( abs( centre[0] - trackCentre ) / noTurnRegion ) ,errorIndex ) * noTurnRegion * angleKP  ) +
//				 ( centre[0] - preCentre[0]  ) * angleKD;

//	if(wheelRatio<0){
//		wheelRatio -= 1;
//	}else{
//		wheelRatio += 1;
//	}

//	differentialNEW();
	servoTo(angleError);
	//set servo angle and limit it

	preCentre[0] = centre[0];
	preCentre[1] = centre[1];

	if(abs(angleError)<70){
		wheelRatio = 1;
	}else if(angleError<0){
//		wheelRatio = - ( 0.2876 * log( - angleError) - 0.0699 );
//	}else{
//		wheelRatio = ( 0.2876 * log(angleError) - 0.0699 );
//	}
		if(angleError<-maxServoAngleForSpeed){
			wheelRatio = - ( 0.0024 * ( maxServoAngleForSpeed ) + 0.8344 );
		}else{
			wheelRatio = - ( 0.0024 * ( - angleError ) + 0.8344 );
		}
	}else{
		if(angleError>maxServoAngleForSpeed){
			wheelRatio = ( 0.0024 * ( maxServoAngleForSpeed ) + 0.8344 );
		}else{
			wheelRatio = ( 0.0024 * ( angleError ) + 0.8344 );
		}
	}
}

//void car::differential()
//{
//	if(angleError>540){
//		angleError = 540;
//	}else if(angleError<-540){
//		angleError = -540;
//	}
//	uint8_t speedIndex = abs(angleError/3) + preTurning;
//	if(angleError>0){
//		if(speedIndex>182){
//			targetLeftSpeed = targetCarSpeed * outerSpeed[182] / 100000;
//			targetRightSpeed = targetCarSpeed * innerSpeed[182] / 100000;
//		}else{
//		targetLeftSpeed = targetCarSpeed * outerSpeed[speedIndex] / 100000;
//		targetRightSpeed = targetCarSpeed * innerSpeed[speedIndex] / 100000;
//		}
//	}else if(angleError<0){
//		if(speedIndex>182){
//			targetRightSpeed = targetCarSpeed * outerSpeed[182] / 100000;
//			targetLeftSpeed = targetCarSpeed * innerSpeed[182] / 100000;
//		}else{
//			targetRightSpeed = targetCarSpeed * outerSpeed[speedIndex] / 100000;
//			targetLeftSpeed = targetCarSpeed * innerSpeed[speedIndex] / 100000;
//		}
//	}else{
//		targetLeftSpeed = targetCarSpeed;
//		targetRightSpeed = targetCarSpeed;
//	}
//}

//void car::differentialNEW()
//{
//	if(wheelRatio>-1.005 && wheelRatio<1.005){
//		angleError = 0;
//	}else if(wheelRatio<-1){
//		angleError = - ( int16_t ) ( ( - wheelRatio - 0.8965 ) / 0.0022 );
//	}else{
//		angleError = ( int16_t ) ( ( wheelRatio - 0.9531 ) / 0.002 );
//	}
//}

//void car::printCentre(const uint16_t color)
//{
//	lcd.SetRegion(Lcd::Rect(centre, 128, 1, 32));
//	lcd.FillColor(color);
//}

//void car::setMotorPower()
//{
//	bool forward = true;
//	if(output==0){
//		motorL.SetPower(0);
//	}else if(output>0||output<=1000){
//		motorL.SetPower(output);
//		forward = true;
//		motorL.SetClockwise(!forward);
//
//	}else if(output>1000){
//		motorL.SetPower(1000);
//		forward = true;
//		motorL.SetClockwise(!forward);
//
//	}else if(output<0||output>=(-1000)){
//		motorL.SetPower(-output);
//		forward = false;
//		motorL.SetClockwise(!forward);
//
//	}else if(output<(-1000)){
//		motorL.SetPower(1000);
//		forward = false;
//		motorL.SetClockwise(!forward);
//
//	}
//}

void car::dirControl_1()
{
	int r0=112,r1=112,l0=16,l1=16;
	for(ccdId=0; ccdId<2; ccdId++){
		int16_t maxData = 0;
		int16_t minData = 255;
		for(int j=ccdLength[ccdId][0]; j<ccdLength[ccdId][1]; j++){//find max and min
			if(ccdData.at(ccdId).at(j)>maxData){
				maxData = ccdData.at(ccdId).at(j);
			}
			if(ccdData.at(ccdId).at(j)<minData){
				minData = ccdData.at(ccdId).at(j);
			}
		}

		max[ccdId] = maxData;

		if(ccdId==0){
			if(maxData<=allBlackConstant){//ccd0 seeing nothing, then refer back to previous centre and turn to the most
				if(centre[0]>64){
					centre[0] = 127;
				}else{
					centre[0] = 0;
				}
				continue;//keep update ccd1 or you can use break
			}else if(minData>allBlackConstant){//ccd0 seeing all white
				centre[0] = 64;
				continue;
			}
		}else if(ccdId==1){//other ccd
			if(maxData<allBlackConstant){// ccd1 being black
				centre[1] = 0;// will not accelerate
				break;
			}else if(minData>allBlackConstant){
				centre[1] = 0;// will not accelerate
				break;
			}
		}

		threshold[ccdId] = ( maxData + minData ) / 2;
		//find threshold

		bool prePixelIsHigh = false;
		noOfSegment[ccdId] = 0;
		for(int j=ccdLength[ccdId][0]; j<ccdLength[ccdId][1]; j++){
			if(ccdData.at(ccdId).at(j)>=threshold[ccdId]){
				if(prePixelIsHigh == false){//from black to white
					noOfSegment[ccdId]++;
				}
				prePixelIsHigh = true;
			}else{
				prePixelIsHigh = false;
			}
		}
		//find no. of white segment

		prePixelIsHigh = false;//assume black first
		uint16_t edge[noOfSegment[ccdId]][2];
		uint8_t distance[noOfSegment[ccdId]];
		uint16_t segmentCentre[noOfSegment[ccdId]];
		uint8_t indexOfSegment = 0;//index of first segment
		for(int j=ccdLength[ccdId][0]; j<ccdLength[ccdId][1]; j++){
			if(ccdData.at(ccdId).at(j)>=threshold[ccdId]){
				if(prePixelIsHigh == false){// from black to white edge
					edge[indexOfSegment][0] = j;//starting edge
				}
				prePixelIsHigh = true;
				if(j==(ccdLength[ccdId][1]-1)){//last pixel in each ccd
					edge[indexOfSegment][1] = (ccdLength[ccdId][1]-1);
				}
			}else{
				if(prePixelIsHigh == true){
					edge[indexOfSegment][1] = --j;//ending edge which is the previous index
					indexOfSegment++;//go to next segment
				}
				prePixelIsHigh = false;
			}
		}
		for(int i=0; i<noOfSegment[ccdId]; i++){//centre of each segment
			segmentCentre[i] = ( edge[i][0] + edge[i][1] ) / 2;
		}
		for(int i=0; i<noOfSegment[ccdId]; i++){//distance between centres and preCentre
			distance[i] = abs((preCentre[ccdId]-segmentCentre[i]));
		}
		int minSegment = 0;
		for(int i=0; i<noOfSegment[ccdId]; i++){//find the one that is the closest to 64
			if(distance[i]<=distance[minSegment]){
				minSegment = i;
			}
			if(ccdId==0){
			l0=edge[minSegment][0];
			r0=edge[minSegment][1];
			}
			else if(ccdId==1){
				l1=edge[minSegment][0];
				r1=edge[minSegment][1];

			}
		}


		if(abs(preCentre[ccdId]-segmentCentre[minSegment])>(ccdLength[ccdId][1]-ccdLength[ccdId][0])*5/9){// missed the track and saw the other track
			//***use the previous centre***
		}else{
			centre[ccdId] = segmentCentre[minSegment];
		}
	}
	angleError=
			data_process(l1, r1,
						 l0, r0, 16,
						112, 16, 112, PREANGLE,
						-500, 500, 3, 9.6, 9.6,
						3, 10.4, 10.4,
						ccdData[0]);
	servoTo(angleError);
	PREANGLE = angleError;
}

void car::changingSpeed()
{
	a = ( minSpeed - straightLineSpeed ) / ( - ( maxErrorForChangingSpeed * maxErrorForChangingSpeed ) );
	float error = centreError[1];
	if(abs(error)>maxErrorForChangingSpeed){
		error = maxErrorForChangingSpeed;
	}
	if(targetCarSpeed==0){
	}else{
		targetCarSpeed = - a * error * error + straightLineSpeed;
		if(targetCarSpeed>straightLineSpeed){
			targetCarSpeed = straightLineSpeed;
		}
		if(abs(centreError[0])>maxErrorForChangingSpeed){
			targetCarSpeed = 860;
		}
	}
}

void car::speedPID()
{
//	differential();
	//update left and right speed according to the servo angle

	if(wheelRatio<-1){
		targetRightSpeed = ( int32_t ) ( targetCarSpeed * 2 * ( - wheelRatio) / ( - wheelRatio + 1) );
		targetLeftSpeed = ( int32_t ) ( targetCarSpeed * 2 / ( - wheelRatio + 1) );
	}else{
		targetLeftSpeed = ( int32_t ) ( targetCarSpeed * 2 * ( wheelRatio) / ( wheelRatio + 1) );
		targetRightSpeed = ( int32_t ) ( targetCarSpeed * 2 / ( wheelRatio + 1) );
	}

//	targetLeftSpeed = targetCarSpeed;
//	targetRightSpeed = targetCarSpeed;

	lError = targetLeftSpeed - currentLeftSpeed;

//	if((accuLeftError+lError)>(1000/sKI_l)){
//	}else{
//		accuLeftError = accuLeftError + lError;
//	}

	if(lError<=200){
		if((accuLeftError+lError)>(1000/sKI_l[0])){
		}else{
			accuLeftError = accuLeftError + lError;
		}
		LPWM = (int16_t)( lError*sKP_l[0] + accuLeftError*sKI_l[0] );
	}else{
		if((accuLeftError+lError)>(1000/sKI_l[1])){
		}else{
			accuLeftError = accuLeftError + lError;
		}
		LPWM = (int16_t)( lError*sKP_l[1] + accuLeftError*sKI_l[1] );
	}

//	LPWM = (int16_t)( lError*sKP_l + accuLeftError*sKI_l );
	preLeftError = lError;
	preTargetLeftSpeed = targetLeftSpeed;

	rError = targetRightSpeed - currentRightSpeed;

//	if((accuRightError+rError)>(1000/sKI_r)){
//	}else{
//		accuRightError = accuRightError + rError;
//	}

	if(lError<=200){
		if((accuRightError+rError)>(1000/sKI_r[0])){
		}else{
			accuRightError = accuRightError + rError;
		}
		RPWM = (int16_t)( rError*sKP_r[0] + accuRightError*sKI_r[0] );
	}else{
		if((accuRightError+rError)>(1000/sKI_r[1])){
		}else{
			accuRightError = accuRightError + rError;
		}
		RPWM = (int16_t)( rError*sKP_r[1] + accuRightError*sKI_r[1] );
	}

//	RPWM = (int16_t)( rError*sKP_r + accuRightError*sKI_r );
	preRightError = rError;
	preTargetRightSpeed = targetRightSpeed;
	//PID

	preTargetLeftSpeed = targetLeftSpeed;
	preTargetRightSpeed = targetRightSpeed;

	if(targetCarSpeed == 0){
		motorTo(0, 0);
		motorTo(1, 0);
		accuLeftError = 0;
		accuRightError = 0;
	}else{
		motorTo(0, LPWM);
		motorTo(1, RPWM);
	}
	//set PWM
}

void car::motorTo(bool id, int16_t PWM)
{
	if(id==0){
		if(PWM>0){
			motorL.SetPower(PWM);
			motorL.SetClockwise(true);
		}else{
			motorL.SetPower(-PWM);
			motorL.SetClockwise(false);
		}
	}else{
		if(PWM>0){
			motorR.SetPower(PWM);
			motorR.SetClockwise(false);
		}else{
			motorR.SetPower(-PWM);
			motorR.SetClockwise(true);
		}
	}

}
