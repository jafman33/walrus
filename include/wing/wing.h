#ifndef __WING_H_INCLUDED__
#define __WING_H_INCLUDED__

#include "Arduino.h"
#include "../config.h"
#include "AsyncUDP_Teensy41.hpp"    
#include "QNEthernet.h"   

#include "Servo.h"

#include "SPI.h"
// #include "YOST_TTS_LX.h"
// #include "myAHRS_plus.h"

#include <Wire.h>
#include "MS5837.h"

using namespace qindesign::network;

namespace Cyberwing
{
	class Wing
	{
		enum class STATUS : uint8_t {
			INIT     = 1,
			RUNNING  = 2,
			FAILURE  = 3
		};

		struct InPacket {
			uint32_t time;
			float axis1;
			float axis2;
			float axis3;
			float axis4;
			float axis5;
			uint32_t sync;
		};

		struct OutPacket {
			float ax, ay, az;
			float wx, wy, wz;
			float q1, q2, q3, q4;
			float depth, temperature;
			float leak;
			float d1, d2, d3, d4, d5;
		};

	public:
		Wing(void);
		void init(void);
		void update(void);

	protected:
		STATUS status_;

		// IPAddress remoteIP_; 
		AsyncUDP udp1_;
		AsyncUDP udp2_;

		Servo servo1_;
		Servo servo2_;
		Servo servo3_;
		Servo servo4_;
		Servo servo5_;

		// YOST_TTS_LX imu_;

		MS5837 depth_;

		InPacket inPacket_;
		OutPacket outPacket_;

		float leak_= 0;
		float state_[18];
		float input_[5];
		float servoFeedback_[5];

		byte packetBuffer_[PACKET_SIZE_OUT];
		
		void parsePacket(AsyncUDPPacket packet);
		void publishState(void);
		void forwardInputs(void);
		void updateState(void);

	};
}

#endif
