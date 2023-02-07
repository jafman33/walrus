#ifndef __CONFIG_H_INCLUDED__
#define __CONFIG_H_INCLUDED__

//////////////////SERVO//////////////////
#define SERVO1_PIN 23
#define SERVO2_PIN 24
/////////////////////////////////////////

/////////////////ETHERNET////////////////
#define ASYNC_UDP_TEENSY41_DEBUG_PORT Serial
#define _ASYNC_UDP_TEENSY41_LOGLEVEL_ 0
#define SHIELD_TYPE "Teensy4.1 QNEthernet"
#define USING_DHCP true
#define RECEIVE_PORT 1560
#define SEND_PORT 1561
#define PACKET_SIZE_OUT 4
/////////////////////////////////////////

#endif
