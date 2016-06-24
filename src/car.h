/*
 * car.h
 *
 *  Created on: 21 Jan 2016
 *      Author: ChungWa
 */

#ifndef CAR_H_
#define CAR_H_

#include <cassert>
#include <cstring>
#include <array>
#include <deque>
#include <cstdint>
#include <functional>
#include <cmath>
#include <libbase/k60/adc.h>
#include <libbase/k60/pin.h>
#include <libbase/k60/gpio.h>
#include <libsc/ab_encoder.h>
#include <libsc/tsl1401cl.h>
#include <libsc/alternate_motor.h>
#include <libsc/futaba_s3010.h>
#include <libsc/st7735r.h>
#include <libsc/button.h>
#include <libsc/joystick.h>
#include <libsc/led.h>
#include <libsc/k60/jy_mcu_bt_106.h>
#include <libbase/k60/pin.h>
#include <libsc/lcd_console.h>
#include "myDeque.h"
#include "pVarManager.h"

using namespace std;
using namespace libsc;
using namespace libbase::k60;

class car {
public:
	car();
	void updateCcd();
	void updateEncoder();
	void ledSwitch(const uint8_t id);
	void printCcdData();
	void eraceCcdData();
	void print();
	void printAngle();
	void dirControl();
	void speedPID();
	void torquePI();
//	void setMotorPower(int);// power from -1000 ~ 1000 and positive is pushing forward

	static car *m_instance;
	static void listener(const std::vector<Byte>& data);
	int32_t accuLeftError = 0;
	int32_t accuRightError = 0;
private:
	bool halfCounter = false;

	//degree:36.2, 48.5, 58.9, 67.1
	//length:508, 534, 584, 651
	const int16_t ccdLength[4][2] = { {16, 113}, {16, 113}, {0, 128}, {0, 128} };
	int8_t ccdMaxReady = 0;
	int8_t ccdMinReady = 0;
	const uint8_t ccd0MaxMax = 230;
	bool ccdMaxHasBeenChanged = false;
	bool ccdMinHasBeenChanged = false;
	uint8_t ccd0Max[128] = { NULL };
	uint8_t ccd0Min[128] = { NULL };
	const uint8_t ccd1MaxMax = 230;
	uint8_t ccd1Max[128] = { NULL };
	uint8_t ccd1Min[128] = { NULL };
	array<array<uint16_t, 128>, 4> ccdData;
	int8_t ccdId = 1;
	int8_t previousCcdId = 0;

	int32_t accuLCount = 0;
	int32_t accuRCount = 0;
	int16_t max[2];
	uint8_t printCcd = 1;
	int8_t centreError[2] = { NULL };
	uint16_t allBlackConstant = 0;

	myDeque currentLeftSpeedSq;
	myDeque currentRightSpeedSq;
	int16_t leftTCurrent = 0;
	int16_t rightTCurrent = 0;
	int16_t leftCurrent = 0;
	int16_t rightCurrent = 0;
	uint32_t iKP = 0;
	uint32_t iKD = 0;
	int32_t lError = 0;
	int32_t rError = 0;
	int32_t preTargetLeftSpeed = 0;
	int32_t preTargetRightSpeed = 0;
	int32_t targetCarSpeed = 0;
	int32_t targetLeftSpeed = 0;
	int32_t targetRightSpeed = 0;
	int32_t currentLeftSpeed = 0;
	int32_t currentRightSpeed = 0;

	int32_t preLeftError = 0;
	int32_t preRightError = 0;
//	float sKP_l = 0.16;
//	float sKI_l = 0.062;
//	float sKD_l = 0;
//	float sKP_r = 0.138;
//	float sKI_r = 0.06;
//	float sKD_r = 0;
//	float sKP_l = 0.15;
//	float sKI_l = 0.04;
//	float sKD_l = 0;
//	float sKP_r = 0.12;
//	float sKI_r = 0.04;
//	float sKD_r = 0;

//	float sKP_l = 0.12;
//	float sKI_l = 0.01;
//	float sKD_l = 0.3;
//	float sKP_r = 0.12;
//	float sKI_r = 0.01;
//	float sKD_r = 0.3;
//	int16_t LPWM = 0;
//	int16_t RPWM = 0;

	float sKP_l[3] = { 3, 2, 0.8 };
	float sKI_l[3] = { 0.1, 0.1, 0.02 };
	float sKP_r[3] = { 3, 2, 0.8 };
	float sKI_r[3] = { 0.1, 0.1, 0.02 };

//	float sKP_l = 0.68;
//	float sKI_l = 0.06;
//	float sKD_l = 1.1;
//	float sKP_r = 0.68;
//	float sKI_r = 0.06;
//	float sKD_r = 1.2;
	int16_t LPWM = 0;
	int16_t RPWM = 0;

	bool isKP = true;
	double wheelRatio = 1;//+ is turning right; - is turning left; 1 is both wheels moving at the same speed;
	uint8_t preTurning = 50;
	float errorIndex = 1.15;
	int32_t angleError = 0;
	float angleKP0Left = 7.5;
	float angleKP0Right = 8;
	int16_t maxServoAngleForSpeed = 280;
	int16_t maxServoAngleToLeft = 269;
	int16_t maxServoAngleToRight = 285;
	uint8_t straightLineRegion = 0;
	uint8_t noOfSegment[2] = {0, 0};
	int32_t centre[4] = {64, 64, 64, 64};
	int32_t preCentre[4] = {64, 64, 64, 64};
	int16_t threshold[4];
	const int8_t trackCentre = (ccdLength[0][0]+ccdLength[0][1])/2;
//	int32_t offset = 16;
	const uint8_t noTurnRegion = 5;
	int32_t previousAngleError = 0;
//	int32_t previousError = 0;
//	int32_t encoderData[128] = {};
//	int16_t errorData[128] = {};
//	int32_t counter = 0;
	//variable

	Led D1;
	Led D2;
	Button sw;
	AbEncoder encoderR;
	AbEncoder encoderL;
	Adc lCurrent;
	Adc rCurrent;
	Tsl1401cl ccd[4];
	AlternateMotor motorL;
	AlternateMotor motorR;
	FutabaS3010 servo;
	St7735r lcd;
	Joystick joystick;
	LcdConsole console;
	pVarManager pGrapher;
//	libsc::k60::JyMcuBt106 bluetooth;
	Button::Config getButtonConfig(const uint8_t id);
	LcdConsole::Config getLcdConsoleConfig();
	Joystick::Config getJoystickConfig();
	void servoTo(int16_t degree); //Left=-546, M=0, Right=+546
	void motorTo(bool id, int16_t PWM);
	void differential();
	void changingSpeed();
//	void differentialNEW();
//	void setMotorPower();

	const uint16_t turningLeft_errorToServo[64] = {
			51,
			55,
			60,
			65,
			70,
			76,
			81,
			87,
			93,
			98,
			104,
			110,
			117,
			123,
			129,
			135,
			142,
			148,
			155,
			161,
			168,
			174,
			181,
			188,
			195,
			201,
			208,
			215,
			222,
			229,
			236,
			243,
			250,
			257,
			264,
			271,
			279,
			286,
			293,
			300,
			308,
			315,
			322,
			330,
			337,
			345,
			352,
			359,
			367,
			374,
			382,
			390,
			397,
			405,
			412,
			420,
			428,
			435,
			443,
			451,
			459,
			466,
			474,
			482
	};

	const uint16_t turningRight_errorToServo[64] = {
			27,
			32,
			38,
			43,
			49,
			55,
			61,
			67,
			74,
			80,
			87,
			93,
			100,
			107,
			114,
			121,
			128,
			135,
			142,
			149,
			156,
			164,
			171,
			178,
			186,
			193,
			201,
			208,
			216,
			224,
			231,
			239,
			247,
			255,
			262,
			270,
			278,
			286,
			294,
			302,
			310,
			318,
			326,
			334,
			343,
			351,
			359,
			367,
			375,
			384,
			392,
			400,
			409,
			417,
			425,
			434,
			442,
			451,
			459,
			468,
			476,
			485,
			493,
			502
	};

	const float twoWheelsRatio[64] = {
			1.00801225330836,
			1.01778030334687,
			1.02834288887548,
			1.03945696359559,
			1.051,
			1.06289680849289,
			1.07509610587072,
			1.08756048453231,
			1.10026135206799,
			1.11317608615092,
			1.12628630575393,
			1.13957675718841,
			1.15303456248958,
			1.16664869309065,
			1.18040958978925,
			1.19430888119304,
			1.20833917050291,
			1.22249387096578,
			1.23676707677787,
			1.25115346032236,
			1.26564818931189,
			1.28024685920958,
			1.29494543754061,
			1.30974021757298,
			1.32462777946434,
			1.33960495741939,
			1.35466881173142,
			1.36981660482677,
			1.38504578061607,
			1.40035394659687,
			1.41573885826133,
			1.43119840544691,
			1.44673060033464,
			1.46233356685194,
			1.47800553127902,
			1.49374481389167,
			1.50954982150022,
			1.52541904076694,
			1.54135103220239,
			1.55734442475579,
			1.57339791092744,
			1.58951024234114,
			1.60568022572344,
			1.62190671924369,
			1.638188629175,
			1.65452490684152,
			1.67091454582172,
			1.68735657938126,
			1.70385007811212,
			1.7203941477576,
			1.73698792720515,
			1.75363058663091,
			1.77032132578196,
			1.78705937238352,
			1.80384398065988,
			1.82067442995913,
			1.83755002347246,
			1.85447008704024,
			1.87143396803737,
			1.88844103433153,
			1.90549067330834,
			1.92258229095822,
			1.93971531101989,
			1.95688917417645
	};

	const uint32_t innerSpeed[183] = {//mSpeed = 100000
			99544,
			99470,
			99382,
			99301,
			99213,
			99132,
			99045,
			98957,
			98877,
			98790,
			98703,
			98622,
			98536,
			98449,
			98369,
			98283,
			98196,
			98110,
			98031,
			97945,
			97859,
			97773,
			97687,
			97608,
			97523,
			97437,
			97352,
			97267,
			97182,
			97097,
			97012,
			96927,
			96842,
			96757,
			96673,
			96588,
			96504,
			96420,
			96335,
			96251,
			96167,
			96077,
			95993,
			95909,
			95825,
			95742,
			95652,
			95568,
			95485,
			95395,
			95311,
			95228,
			95139,
			95055,
			94966,
			94883,
			94794,
			94711,
			94622,
			94539,
			94450,
			94361,
			94278,
			94190,
			94101,
			94018,
			93930,
			93841,
			93752,
			93664,
			93575,
			93493,
			93405,
			93317,
			93228,
			93140,
			93045,
			92957,
			92869,
			92781,
			92693,
			92605,
			92511,
			92423,
			92329,
			92241,
			92153,
			92059,
			91971,
			91877,
			91783,
			91695,
			91601,
			91507,
			91413,
			91326,
			91232,
			91138,
			91044,
			90950,
			90856,
			90756,
			90662,
			90568,
			90474,
			90374,
			90280,
			90180,
			90087,
			89986,
			89886,
			89792,
			89692,
			89592,
			89492,
			89392,
			89291,
			89191,
			89091,
			88984,
			88884,
			88784,
			88677,
			88576,
			88470,
			88363,
			88256,
			88149,
			88042,
			87935,
			87828,
			87721,
			87607,
			87500,
			87387,
			87273,
			87159,
			87045,
			86931,
			86817,
			86703,
			86582,
			86468,
			86347,
			86226,
			86105,
			85984,
			85862,
			85734,
			85606,
			85485,
			85356,
			85221,
			85093,
			84957,
			84828,
			84693,
			84557,
			84414,
			84271,
			84135,
			83985,
			83841,
			83691,
			83540,
			83389,
			83231,
			83073,
			82914,
			82749,
			82583,
			82409,
			82236,
			82055,
			81874,
			81692,
			81496,
			81300,
			81102,
			80891,
			80679,
			80452,
			80224
	};

	const uint32_t outerSpeed[183] = {//mSpeed = 100000
			100456,
			100530,
			100618,
			100699,
			100787,
			100868,
			100955,
			101043,
			101123,
			101210,
			101297,
			101378,
			101464,
			101551,
			101631,
			101717,
			101804,
			101890,
			101969,
			102055,
			102141,
			102227,
			102313,
			102392,
			102477,
			102563,
			102648,
			102733,
			102818,
			102903,
			102988,
			103073,
			103158,
			103243,
			103327,
			103412,
			103496,
			103580,
			103665,
			103749,
			103833,
			103923,
			104007,
			104091,
			104175,
			104258,
			104348,
			104432,
			104515,
			104605,
			104689,
			104772,
			104861,
			104945,
			105034,
			105117,
			105206,
			105289,
			105378,
			105461,
			105550,
			105639,
			105722,
			105810,
			105899,
			105982,
			106070,
			106159,
			106248,
			106336,
			106425,
			106507,
			106595,
			106683,
			106772,
			106860,
			106955,
			107043,
			107131,
			107219,
			107307,
			107395,
			107489,
			107577,
			107671,
			107759,
			107847,
			107941,
			108029,
			108123,
			108217,
			108305,
			108399,
			108493,
			108587,
			108674,
			108768,
			108862,
			108956,
			109050,
			109144,
			109244,
			109338,
			109432,
			109526,
			109626,
			109720,
			109820,
			109913,
			110014,
			110114,
			110208,
			110308,
			110408,
			110508,
			110608,
			110709,
			110809,
			110909,
			111016,
			111116,
			111216,
			111323,
			111424,
			111530,
			111637,
			111744,
			111851,
			111958,
			112065,
			112172,
			112279,
			112393,
			112500,
			112613,
			112727,
			112841,
			112955,
			113069,
			113183,
			113297,
			113418,
			113532,
			113653,
			113774,
			113895,
			114016,
			114138,
			114266,
			114394,
			114515,
			114644,
			114779,
			114907,
			115043,
			115172,
			115307,
			115443,
			115586,
			115729,
			115865,
			116015,
			116159,
			116309,
			116460,
			116611,
			116769,
			116927,
			117086,
			117251,
			117417,
			117591,
			117764,
			117945,
			118126,
			118308,
			118504,
			118700,
			118898,
			119109,
			119321,
			119548,
			119776
	};
};

#endif /* CAR_H_ */
