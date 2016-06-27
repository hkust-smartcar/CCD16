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

Joystick::Config car::getJoystickConfig()
{
	Joystick::Config config;
	config.id = 0;
	config.is_active_low = true;
	config.listener_triggers[0] = Joystick::Config::Trigger::kUp;
	config.listener_triggers[1] = Joystick::Config::Trigger::kUp;
	config.listener_triggers[2] = Joystick::Config::Trigger::kUp;
	config.listener_triggers[3] = Joystick::Config::Trigger::kUp;
	config.listener_triggers[4] = Joystick::Config::Trigger::kUp;
	config.listeners[static_cast<int>(Joystick::State::kUp)] = [&](const uint8_t)
	{
		if(targetCarSpeed==0){
			targetCarSpeed = 1250;
		}else{
			targetCarSpeed = 0;
		}
//		allBlackConstant += 2;
//		servoTo(angleError += 5);
//		if(isKP == true){
//			angleKP0 += 0.2;
//		}else{
//			angleKD1 += 0.2;
//		}
//		offset += 1;
	};
	config.listeners[static_cast<int>(Joystick::State::kDown)] = [&](const uint8_t)
	{
//		allBlackConstant -= 2;
//		servoTo(angleError -= 5);
//		if(isKP == true){
//			angleKP0 -= 0.2;
//		}else{
//			angleKD1 -= 0.2;
//		}
//		offset -= 1;
	};
	config.listeners[static_cast<int>(Joystick::State::kLeft)] = [&](const uint8_t)
	{
//		errorIndex -=0.05;
	};
	config.listeners[static_cast<int>(Joystick::State::kRight)] = [&](const uint8_t)
	{
//		errorIndex +=0.05;
	};
	config.listeners[static_cast<int>(Joystick::State::kSelect)] = [&](const uint8_t)
	{
		ccdMinReady += 1;
	};
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
	sw(getButtonConfig(0)),
	encoderR(getAbEncoderConfig(0)),
	encoderL(getAbEncoderConfig(1)),
	lCurrent(getAdcConfig(Pin::Name::kPtb10)),
	rCurrent(getAdcConfig(Pin::Name::kPtb11)),
	ccd{Tsl1401cl(0), Tsl1401cl(1), Tsl1401cl(2), Tsl1401cl(3)},
	motorL(getAlternateMotorConfig(0)),
	motorR(getAlternateMotorConfig(1)),
	servo(getFutabaS3010Config(0)),
	lcd(getSt7735rConfig()),
	joystick(getJoystickConfig()),
	console(getLcdConsoleConfig())
//	,bluetooth(getJyMcuBt106Config(0))
{
//	if (m_instance)
//		assert(false);
	m_instance = this;

	D1.SetEnable(true);
	D2.SetEnable(true);
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
	pGrapher.addWatchedVar(&centreError[0], "centreError[0]");
//	pGrapher.addWatchedVar(&max[1], "max[1]");
//	pGrapher.addWatchedVar(&centreError[1], "centreError[1]");
//	pGrapher.addWatchedVar(&leftCurrent, "leftCurrent");
//	pGrapher.addWatchedVar(&rightCurrent, "rightCurrent");
	pGrapher.addSharedVar(&targetCarSpeed, "targetCarSpeed");
	pGrapher.addSharedVar(&sKP_l, "sKP_l");
	pGrapher.addSharedVar(&sKI_l, "sKI_l");
	pGrapher.addSharedVar(&sKD_l, "sKD_l");
	pGrapher.addSharedVar(&sKP_r, "sKP_r");
	pGrapher.addSharedVar(&sKI_r, "sKI_r");
	pGrapher.addSharedVar(&sKD_r, "sKD_r");
//	pGrapher.addSharedVar(&angleKP0Left, "angleKP0Left");
//	pGrapher.addSharedVar(&angleKP0Right, "angleKP0Right");
//	pGrapher.addSharedVar(&maxServoAngleForSpeed, "maxServoAngleForSpeed");
//	pGrapher.addSharedVar(&maxServoAngleToLeft, "maxServoAngleToLeft");
//	pGrapher.addSharedVar(&maxServoAngleToRight, "maxServoAngleToRight");
//	pGrapher.addSharedVar(&angleKD0Left, "angleKD0Left");
//	pGrapher.addSharedVar(&angleKD0Right, "angleKD0Right");
//	pGrapher.addSharedVar(&straightLineSpeed, "straightLineSpeed");
//	pGrapher.addSharedVar(&straightLineRegionOfCcd1, "straightLineRegionOfCcd1");
//	pGrapher.addSharedVar(&angleKP1, "angleKP1");
//	pGrapher.addSharedVar(&currentBrakingTime, "currentBrakingTime");
	pGrapher.SetOnReceiveListener(&listener);
}

void car::updateCcd()
{

	for(int i=0; i<2; i++){
		while(!ccd[i].SampleProcess()){}
		ccdData.at(i) = ccd[i].GetData();
		ccd[i].StartSample();
	}

	if(ccdMinHasBeenChanged==false){
		if(ccdMaxHasBeenChanged==true){
			ledSwitch(0);
		}
	}	//D1 first blinking indicates max recorded, D1 then not blinking indicates min recorded.

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
	//update 2 ccd
	if(ccd0Max!=NULL&&ccd0Min!=NULL&&ccd1Max!=NULL&&ccd1Min!=NULL){
		if(allBlackConstant==0){
			allBlackConstant = 230 /4;
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
			if(data[i]>230){
				data[i] = 230;
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
			if(data[i]>230){
				data[i] = 230;
			}
		}
		for(int i=0; i<128; i++){
			if(i!=0||i!=127){
				ccdData.at(1).at(i) = ( data[i+1] + data[i] + data[i-1] ) / 3;
			}
		}
	}
}

void car::updateEncoder()
{
	encoderL.Update();
	encoderR.Update();
	if(abs(encoderL.GetCount())>20000){
	}else{
		currentLeftSpeedSq.update((int32_t)( - encoderL.GetCount() / 2 ));
		currentLeftSpeed = currentLeftSpeedSq.average();

//		currentLeftSpeed = (int32_t)( - encoderL.GetCount() / 2 );

//		accuLCount += currentLeftSpeed;
	}
	if(abs(encoderR.GetCount())>20000){
	}else{
		currentRightSpeedSq.update((int32_t)( encoderR.GetCount() ) );
		currentRightSpeed = currentRightSpeedSq.average();

//		currentRightSpeed = (int32_t)( encoderR.GetCount() );

//		accuRCount += currentRightSpeed;
	}

	leftCurrent = 2045-lCurrent.GetResult();
	rightCurrent = rCurrent.GetResult() - 2045;
	//enconder update

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
	servo.SetDegree(857 + angleError);
}

void car::dirControl()
{
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
			edge[i][0] = ( edge[i][0] + edge[i][1] ) / 2;
		}
		for(int i=0; i<noOfSegment[ccdId]; i++){//distance between centres and preCentre
			edge[i][1] = abs((preCentre[ccdId]-edge[i][0]));
		}
		int minSegment = 0;
		for(int i=0; i<noOfSegment[ccdId]; i++){//find the one that is the closest to 64
			if(edge[i][1]<=edge[minSegment][1]){
				minSegment = i;
			}
		}


		if(abs(preCentre[ccdId]-edge[minSegment][0])>(ccdLength[ccdId][1]-ccdLength[ccdId][0])*5/9){// missed the track and saw the other track
			//***use the previous centre***
		}else{
			centre[ccdId] = edge[minSegment][0];
		}
	}


//	switch(controlCase){
//		case 0:
			centreError[0] = ( centre[0] - trackCentre );
			centreError[1] = centre[1] - trackCentre;
			if(centreError[0]<0){
				angleError = centreError[0] * angleKP0Left + ( centre[0] - preCentre[0] ) * angleKD0Left;
			}else{
				angleError = centreError[0] * angleKP0Right + ( centre[0] - preCentre[0] ) * angleKD0Right;
			}
//			changingSpeed();
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

void car::changingSpeed(){
	if(abs(centreError[0])<straightLineRegionOfCcd0&&abs(centreError[1])<straightLineRegionOfCcd1){
		if(ccdMinHasBeenChanged==true){
			D1.SetEnable(0);
			D2.SetEnable(0);
		}//indicator

		angleError = centreError[1] * angleKP1;
		if(targetCarSpeed!=0){
			targetCarSpeed = straightLineSpeed;
		}
	}else if(abs(centreError[0])<20&&abs(centreError[1])<21){
		if(ccdMinHasBeenChanged==true){
			D1.SetEnable(1);
			D2.SetEnable(1);
		}//indicator

		if(targetCarSpeed!=0){
//			if(brakingTime>0&&brakingTime<=currentBrakingTime){
//				brakingTime++;
//			}
//			if(targetCarSpeed==straightLineSpeed){
//				targetCarSpeed = 1100;
//				brakingTime++;
//			}else{
				targetCarSpeed = 1300;
				brakingTime = 0;
			}
//		}
	}else{
		if(ccdMinHasBeenChanged==true){
			D1.SetEnable(1);
			D2.SetEnable(1);
		}//indicator

		if(targetCarSpeed!=0){
//			if(brakingTime>0&&brakingTime<=currentBrakingTime){
//				brakingTime++;
//			}
//			if(targetCarSpeed==straightLineSpeed){
//				targetCarSpeed = 1100;
//				brakingTime++;
//			}else{
				targetCarSpeed = 1280;
				brakingTime = 0;
//			}
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

	targetLeftSpeed = targetCarSpeed;
	targetRightSpeed = targetCarSpeed;

	lError = targetLeftSpeed - currentLeftSpeed;
	accuLeftError = accuLeftError + lError;
//	if(preTargetLeftSpeed==targetLeftSpeed){
//		LPWM = (int16_t)( lError*sKP_l[previousIndex_l] + accuLeftError*sKI_l[previousIndex_l] );
//	}else{
//		if(lError<=350){
//			LPWM = (int16_t)( lError*sKP_l[0] + accuLeftError*sKI_l[0] );
//			previousIndex_l = 0;
//		}else if(lError<=700){
//			LPWM = (int16_t)( lError*sKP_l[1] + accuLeftError*sKI_l[1] );
//			previousIndex_l = 1;
//		}else{
//			LPWM = (int16_t)( lError*sKP_l[2] + accuLeftError*sKI_l[2] );
//			previousIndex_l = 2;
//		}
//	}
	LPWM = (int16_t)( lError*sKP_l + accuLeftError*sKI_l + (lError - preLeftError)*sKD_l);
	preLeftError = lError;
	preTargetLeftSpeed = targetLeftSpeed;

	rError = targetRightSpeed - currentRightSpeed;
	accuRightError = accuRightError + rError;
//	if(	preTargetRightSpeed==targetRightSpeed){
//		RPWM = (int16_t)( rError*sKP_r[previousIndex_r] + accuRightError*sKI_r[previousIndex_r] );
//	}else{
//		if(lError<=350){
//			RPWM = (int16_t)( rError*sKP_r[0] + accuRightError*sKI_r[0] );
//			previousIndex_r = 0;
//		}else if(lError<=700){
//			RPWM = (int16_t)( rError*sKP_r[1] + accuRightError*sKI_r[1] );
//			previousIndex_r = 1;
//		}else{
//			RPWM = (int16_t)( rError*sKP_r[2] + accuRightError*sKI_r[2] );
//			previousIndex_r = 2;
//		}
//	}
	RPWM = (int16_t)( rError*sKP_r + accuRightError*sKI_r + (rError - preRightError)*sKD_r);
	preRightError = rError;
	preTargetRightSpeed = targetRightSpeed;
	//PID

//	preTargetLeftSpeed = targetLeftSpeed;
//	preTargetRightSpeed = targetRightSpeed;

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
