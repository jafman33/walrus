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
            udp2_.connect(IPAddress(IP1, IP2, IP3, IP4), SEND_PORT);
            status_ = STATUS::RUNNING;
        }

        ///////////////////
        // Initialize Servos
        servo1_.attach(SERVO1_PIN);    // attaches the servo on pin
        servo2_.attach(SERVO2_PIN);    // attaches the servo on pin

        ////////////////////
        // Init depth sensor
        Wire.begin();
        // Wire.setSCL(DEPTH_SCL);
        // Wire.setSDA(DEPTH_SDA);

        if (!depth_.init()) {
            Serial.println("Init failed!");
            Serial.println("Are SDA/SCL connected correctly?");
            Serial.println("Blue Robotics Bar30: White=SDA, Green=SCL");
            Serial.println("\n\n\n");
            status_ = STATUS::FAILURE;
            delay(1000);
        } else {
            depth_.setFluidDensity(FLUID_DENSITY);
            depth_.setModel(MS5837::MS5837_30BA);
            Serial.println("Depth Sensor Detected!");
            Serial.print(F("Fluid density set to: "));
            Serial.println(FLUID_DENSITY);
            status_ = STATUS::RUNNING;
        }

        


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

        // depth_.read();

        // Serial.print("Pressure: ");
        // Serial.print(depth_.pressure());
        // Serial.println(" mbar");

        // Serial.print("Temperature: ");
        // Serial.print(depth_.temperature());
        // Serial.println(" deg C");

        // Serial.print("Depth: ");
        // Serial.print(depth_.depth());
        // Serial.println(" m");

        // Serial.print("Altitude: ");
        // Serial.print(depth_.altitude());
        // Serial.println(" m above mean sea level");

        switch(status_)
		{
			case STATUS::RUNNING: 
            {
                //////////////////////
                // update depth sensor
                depth_.read();

                ////////////////
                // // Update IMU

				


                // ///////////////////////////////
                // receive input packet and update 
                // receive(); - done through asynchonous function on init()
                forwardInputs();

                // ///////////////////////////////////
                // Send vehicle state to host computer
                updateState();
                publishState();

                break;
            }

            case STATUS::FAILURE:
			{
                // Set Servos to 0 deg
                int del1_ms = map(0, -1.57, 1.57, 1000, 2000);
                int del2_ms = map(0, -1.57, 1.57, 1000, 2000);
                servo1_.writeMicroseconds(del1_ms);
                servo2_.writeMicroseconds(del2_ms);

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

        memcpy(packetBuffer_, &outPacket_, sizeof(packetBuffer_));
        udp2_.write(packetBuffer_, sizeof(packetBuffer_));
    }

    void Wing::forwardInputs(void) {
        int del1_ms = map(input_[0]*1.5, -1.57, 1.57, 1000, 2000);
        int del2_ms = map(input_[1]*1.5, -1.57, 1.57, 1000, 2000);
        servo1_.writeMicroseconds(del1_ms);
        servo2_.writeMicroseconds(del2_ms);
    }

    void Wing::updateState(void)
	{

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

        Serial.print("Depth: ");
        Serial.print(depth_.depth());
        Serial.println(" m");
        float depth = depth_.depth();
        float temperature = depth_.temperature();
                
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