#include "wing/wing.h"


namespace Cyberwing
{
    //Constructor
	Wing::Wing(void):
        status_(STATUS::INIT),
        udp1_(),
        udp2_(),
        servo1_(),
        servo2_(),
        // imu_(YOST_TTS_LX_MODE,
	    //  SPI,
	    //  YOST_TTS_LX_PIN_SS,
	    //  YOST_TTS_LX_PIN_ATT,
	    //  YOST_TTS_LX_RECEIVE_TIMEOUT),
        depth_(FLUID_DENSITY),
        inPacket_(),
        outPacket_(),
        state_{0.0F},
        input_{0.0F}, 
        packetBuffer_{PACKET_SIZE_OUT}
	{}
    

    void Wing::init(void) {
        /////////////////////////
        // Initialize Serial Port
        Serial.begin(9600);

        
        ///////////////////////////////////
        // Initialize Ethernet (using DHCP)
        Ethernet.begin();
        if (!Ethernet.waitForLocalIP(5000)) {
            status_ = STATUS::FAILURE;
            Serial.println(F("Failed to configure Ethernet"));
            if (!Ethernet.linkStatus())
            Serial.println(F("Ethernet cable is not connected."));
            while (true)
            delay(1);
        } else {
            status_ = STATUS::RUNNING;
            Serial.print(F("Connected! Teensy IP address:"));
            Serial.println(Ethernet.localIP());
            delay(1000);
            // Bind to port for receiving packets asynchronously
            udp1_.connect(Ethernet.localIP(), RECEIVE_PORT);
            if (udp1_.listen(RECEIVE_PORT)) {
                udp1_.onPacket([&](AsyncUDPPacket packet) {
                    parsePacket(packet);
                });
            }
            // Connect to IP and Port for sending packets
            udp2_.connect(IPAddress(10, 250, 225, 92), SEND_PORT);
        }

        ///////////////////
        // Initialize Servo
        servo1_.attach(SERVO1_PIN);    // attaches the servo on pin

        ////////////////////
        // Init depth sensor
        Wire.begin();
        depth_.init();

        ////////////////////
        // Initialize IMU
        
		// imu_.initSPI();
        // //Initialize IMU
		// int32_t rtCode = imu_.init(YOST_TTS_LX_RESET_SETTINGS);
		// if(rtCode!=1)
		// {
		// 	Serial.println("IMU init failed");
		// 	status_ = STATUS::FAILURE;
		// 	// errorMessage_ = 1;
		// 	return;
		// }

        //ready
		return;
	}

    //Update internal state
	void Wing::update(void)
	{
        // Serial.println(state_[0]);

        switch(status_)
		{
			case STATUS::RUNNING: 
            {
                //////////////////////
                // update depth sensor
                depth_.read();

                ////////////////
                // // Update IMU

				// while(true)
				// {
				// 	//Check for user commands
				// 	if(Serial.available())
				// 	{
				// 		char cmd = (char)Serial.read();
				// 		switch(cmd)
				// 		{
				// 			case 't':
				// 			{
				// 				imu_.tareCustom();
				// 				Serial.println("Taring IMU");
				// 				break;
				// 			}
				// 			default :
				// 			{
				// 				Serial.println("Unkown command");
				// 				break;
				// 			}
				// 		}
				// 	}
				// 	//Updating main IMU
				// 	imu_.update();
				// 	if(imu_.status_ != IMUAbstract::STATUS::RUNNING)
				// 	{
				// 		Serial.println("State Observer - update - IMU stopped running");
				// 		// errorMessage_ = 3;
				// 		status_ = STATUS::FAILURE;
				// 		return;
				// 	}
				// }


                // ///////////////////////////////
                // receive input packet and update 
                // ///////////////////////////////
                // receive(); - done through asynchonous function on init()
                forwardInputs();

                // ///////////////////////////////////
                // Send vehicle state to host computer
                // ///////////////////////////////////
                updateState();
                publishState();

                break;
            }

            case STATUS::FAILURE:
			{
                servo1_.writeMicroseconds(1500);
                servo2_.writeMicroseconds(1500);  

				break;
			}

            default:
			{
				status_ = STATUS::FAILURE;
				break;
			}
        } // end switch

		return;
	}

    void Wing::parsePacket(AsyncUDPPacket packet) {
        if (packet.length()) {    
            memcpy(&inPacket_, packet.data(), packet.length());  
            input_[0] = inPacket_.axis1;
            input_[1] = inPacket_.axis2;
        }
    }

    // prototype fn.
    void Wing::publishState(void) {

        outPacket_.ax = state_[0];
        outPacket_.ay = state_[1];
        outPacket_.az = state_[2];
        outPacket_.wx = state_[3];
        outPacket_.wy = state_[4];
        outPacket_.wz = state_[5];
        outPacket_.q1 = state_[6];
        outPacket_.q2 = state_[7];
        outPacket_.q3 = state_[8];
        outPacket_.q4 = state_[9];
        outPacket_.depth = state_[10];
        outPacket_.temperature = state_[11];

        // memset(packetBuffer_, 0, sizeof(packetBuffer_));
        memcpy(packetBuffer_, &outPacket_, sizeof(packetBuffer_));
        udp2_.write(packetBuffer_, sizeof(packetBuffer_));
    }

    void Wing::forwardInputs(void) {
        int del1_ms = map(input_[0], -1.57, 1.57, 1000, 2000);
        int del2_ms = map(input_[1], -1.57, 1.57, 1000, 2000);
        servo1_.writeMicroseconds(del1_ms);
        servo2_.writeMicroseconds(del2_ms);
    }

    void Wing::updateState(void)
	{
		// Get IMU and depth data
        // float ax = imu_.rawData_.acc[0];
        // float ay = imu_.rawData_.acc[1];
        // float az = imu_.rawData_.acc[2];
        // float wx = imu_.fusionedData_.rates[0];
        // float wy = imu_.fusionedData_.rates[1];
        // float wz = imu_.fusionedData_.rates[2];
        // float q1 = imu_.fusionedData_.quat[0];
        // float q2 = imu_.fusionedData_.quat[1];
        // float q3 = imu_.fusionedData_.quat[2];
        // float q4 = imu_.fusionedData_.quat[3];
        // float depth = depth_.depth();
        // float temperature = depth_.temperature();
        float ax = 0.1;
        float ay = 0.2;
        float az = 0.3;
        float wx = 0.4;
        float wy = 0.5;
        float wz = 0.6;
        float q1 = 0.7;
        float q2 = 0.8;
        float q3 = 0.9;
        float q4 = 0.11;
        float depth = 2.0;
        float temperature = 88.0;
                
        float stateNew[12];
        stateNew[0] = ax;
        stateNew[1] = ay;
        stateNew[2] = az;
        stateNew[3] = wx;
        stateNew[4] = wy;
        stateNew[5] = wz;
        stateNew[6] = q1;
        stateNew[7] = q2;
        stateNew[8] = q3;
        stateNew[9] = q4;
        stateNew[10] = depth;
        stateNew[11] = temperature;

        memcpy(state_,stateNew,sizeof(state_));	
	}

}