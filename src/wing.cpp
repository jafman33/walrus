#include "wing/wing.h"
#include "myAHRS_plus.h"


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
        Serial.begin(115200);

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
        Wire1.begin();

        
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

        ///////////////////////
        // Leak Sensor
        pinMode(LEAK_PIN, INPUT);

        //ready
		return;
	}

    //Update internal state
	void Wing::update(void)
	{       

        switch(status_)
		{
			case STATUS::RUNNING: 
            {

                //////////////////////
                // update leak sensor
                leak_ = digitalRead(LEAK_PIN);
                // if (leak_ == 1) {
                //     Serial.println("Leak Detected!");
                // } 
            
                //////////////////////
                // update depth sensor
                depth_.read();

                ////////////////
                // // Update IMU
                // Done in update functdion

                /////////////////////////////////
                // receive input packet and update 
                // receive(); - done through asynchonous function on init()
                forwardInputs();

                /////////////////////////////////////
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
        outPacket_.leak = state_[12];

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
        // read state
        uint8_t buf_comp_data[18];
        read(I2C_SLAVE_REG_C_ACC_X_LOW, buf_comp_data, 18);
        int16_t acc_c_x = (buf_comp_data[1]<<8) | buf_comp_data[0];
        int16_t acc_c_y = (buf_comp_data[3]<<8) | buf_comp_data[2];
        int16_t acc_c_z = (buf_comp_data[5]<<8) | buf_comp_data[4];
        int16_t gyro_c_x = (buf_comp_data[7]<<8) | buf_comp_data[6];
        int16_t gyro_c_y = (buf_comp_data[9]<<8) | buf_comp_data[8];
        int16_t gyro_c_z = (buf_comp_data[11]<<8) | buf_comp_data[10];
        // compensate
        float comp_acc_x = (float)acc_c_x * 16.0 / 32767;
        float comp_acc_y = (float)acc_c_y * 16.0 / 32767;
        float comp_acc_z = (float)acc_c_z * 16.0 / 32767;
        float comp_gyro_x = (float)gyro_c_x * 2000 / 32767;
        float comp_gyro_y = (float)gyro_c_y * 2000 / 32767;
        float comp_gyro_z = (float)gyro_c_z * 2000 / 32767;

        // read quaternion
        uint8_t buf_quat[8];
        read(I2C_SLAVE_REG_QUATERNIAN_X_LOW, buf_quat, 8);
        int16_t quat_x = (buf_quat[1]<<8) | buf_quat[0];
        int16_t quat_y = (buf_quat[3]<<8) | buf_quat[2];
        int16_t quat_z = (buf_quat[5]<<8) | buf_quat[4];
        int16_t quat_w = (buf_quat[7]<<8) | buf_quat[6];
        // compensate
        float quaternion_x = (float)quat_x / 32767;
        float quaternion_y = (float)quat_y / 32767;
        float quaternion_z = (float)quat_z / 32767;
        float quaternion_w = (float)quat_w / 32767;

        // New State                
        float stateNew[13];
        stateNew[0] = comp_acc_x;
        stateNew[1] = comp_acc_y;
        stateNew[2] = comp_acc_z;
        stateNew[3] = comp_gyro_x;
        stateNew[4] = comp_gyro_y;
        stateNew[5] = comp_gyro_z;
        stateNew[6] = quaternion_x;
        stateNew[7] = quaternion_y;
        stateNew[8] = quaternion_z;
        stateNew[9] = quaternion_w;
        stateNew[10] = depth_.depth();;
        stateNew[11] = depth_.temperature();
        stateNew[12] = leak_;

        memcpy(state_,stateNew,sizeof(state_));	
	}

}