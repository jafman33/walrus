#ifndef __CONFIG_H_INCLUDED__
#define __CONFIG_H_INCLUDED__

#include <vector>

union num32_t
{
	int32_t i;
	uint32_t ui;
	float f;
	uint8_t c[4];
};

union num16_t
{
	int16_t i;
	uint16_t ui;
	uint8_t c[2];
};

//////////////////SERVO//////////////////
#define SERVO1_PWM_PIN 2 // MAIN SERVO
#define SERVO2_PWM_PIN 3 // TAIL SERVO TOP
#define SERVO3_PWM_PIN 4 // TAIL SERVO BOTTOM LEFT (LOOKING FORWARD ORIENTATION)
#define SERVO4_PWM_PIN 5 // TAIL SERVO BOTTOM RIGHT (LOOKING FORWARD ORIENTATION)
#define SERVO5_PWM_PIN 6 // SPARE PWM

#define SERVO1_ANALOG_PIN A10 // PIN 24 MAIN SERVO
#define SERVO2_ANALOG_PIN A11 // PIN 25 TAIL SERVO TOP
#define SERVO3_ANALOG_PIN A12 // PIN 26 TAIL SERVO BOTTOM LEFT (LOOKING FORWARD ORIENTATION)
#define SERVO4_ANALOG_PIN A13 // PIN 27 TAIL SERVO BOTTOM RIGHT (LOOKING FORWARD ORIENTATION)
#define SERVO5_ANALOG_PIN A17 // PIN 41 SPARE FEEDBACK
/////////////////////////////////////////

/////////////////ETHERNET////////////////
#define ASYNC_UDP_TEENSY41_DEBUG_PORT Serial
#define _ASYNC_UDP_TEENSY41_LOGLEVEL_ 0
#define SHIELD_TYPE "Teensy4.1 QNEthernet"
#define USING_DHCP true
#define RECEIVE_PORT 1560
#define SEND_PORT 1561
#define PACKET_SIZE_OUT 72
// IP ADDRESS OF THE SURFACE COMPUTER
#define IP1 192
#define IP2 168
#define IP3 2
#define IP4 1

/////////////////////////////////////////

//////////////YOST_TTS_LX////////////////
#define YOST_TTS_LX_PIN_SS 9
#define YOST_TTS_LX_PIN_ATT 19
#define YOST_TTS_LX_RECEIVE_TIMEOUT 100000000//10000
#define YOST_TTS_LX_RESET_SETTINGS true
#define YOST_TTS_LX_MODE IMUAbstract::MODE::FUSIONED // IMUAbstract::MODE::FUSIONED or IMUAbstract::MODE::RAW
/////////////////////////////////////////

//////////////////DEPTH/////////////////
#define FLUID_DENSITY 1029 // kg/m^3 (997 freshwater, 1029 for seawater)
#define DEPTH_SDA 18 // white
#define DEPTH_SCL 19 // green
/////////////////////////////////////////

/////////////MY_AHRS_PLUS_IMU////////////
#define MYAHRS_I2C_ADDRESS           0x20
#define MYMOTION_WHO_AM_I_VALUE      0xB1
/////////////////////////////////////////

//////////////////LEAK/////////////////
#define LEAK_PIN 14
/////////////////////////////////////////


#endif
