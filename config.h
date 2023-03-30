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
#define SERVO1_PIN 23
#define SERVO2_PIN 22
/////////////////////////////////////////

/////////////////ETHERNET////////////////
#define ASYNC_UDP_TEENSY41_DEBUG_PORT Serial
#define _ASYNC_UDP_TEENSY41_LOGLEVEL_ 0
#define SHIELD_TYPE "Teensy4.1 QNEthernet"
#define USING_DHCP true
#define RECEIVE_PORT 1560
#define SEND_PORT 1561
#define PACKET_SIZE_OUT 48
#define IP1 192
#define IP2 168
#define IP3 1
#define IP4 130
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

#endif
