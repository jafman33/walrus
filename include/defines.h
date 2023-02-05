#ifndef defines_h
#define defines_h

#include "QNEthernet.h"       // https://github.com/ssilverman/QNEthernet
using namespace qindesign::network;

#define ASYNC_UDP_TEENSY41_DEBUG_PORT Serial
#define _ASYNC_UDP_TEENSY41_LOGLEVEL_ 0 // Debug Level from 0 to 4
#define SHIELD_TYPE "Teensy4.1 QNEthernet"
#define USING_DHCP true

////////////////////////
// Define Incoming Struc 
////////////////////////
struct __attribute__((packed)) JoyStick {
  // The data sending from Ros2 node is a series of bytes that can be represented as:
  uint32_t time;
  float axis1;
  float axis2;
  uint32_t sync;
};

////////////////////////
// Define Outgoing Struc 
////////////////////////
struct __attribute__((packed)) outPacket {
  // The data sending from Ros2 node is a series of bytes that can be represented as:
  float depth;
};

#endif    //defines_h


      // Serial.printf("%u %f %f %u\n", 
      //               myJoystick.time, 
      //               myJoystick.axis1, 
      //               myJoystick.axis2, 
      //               myJoystick.sync);           // access the data in the struct


      /////////////////////////
// Create outgoing packet 
// void createPacket(void)
// {
//   myPacket.depth = 69.69;
//   memset(packetBuffer, 0, PACKET_SIZE);
//   memcpy(packetBuffer, &myPacket, PACKET_SIZE);
// }