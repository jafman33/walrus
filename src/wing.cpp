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
        servo3_(),
        servo4_(),
        servo5_(),
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
        servo1_.attach(SERVO1_PWM_PIN);    // attaches the servo on pin
        servo2_.attach(SERVO2_PWM_PIN);    // attaches the servo on pin
        servo3_.attach(SERVO3_PWM_PIN);    // attaches the servo on pin
        servo4_.attach(SERVO4_PWM_PIN);    // attaches the servo on pin
        servo5_.attach(SERVO5_PWM_PIN);    // attaches the servo on pin

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
            // depth_.setModel(MS5837::MS5837_30BA);
            depth_.setModel(MS5837::MS5837_02BA);
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

                // update leak sensor
                leak_ = digitalRead(LEAK_PIN);
                // if (leak_ == 1) Serial.println("Leak Detected!");
            
                // update depth sensor
                depth_.read();

                // Update IMU
                // - Done in update functdion

                // receive input packet and update 
                // receive(); - done through asynchonous function on init()
                forwardInputs();

                // - Send vehicle state to host computer
                updateState();
                publishState();

                break;
            }

            case STATUS::FAILURE:
			{
                // Set Servos to 0 deg (must be tuned...)
                int del1_ms = map(0, -1.57, 1.57, 1000, 2000);
                int del2_ms = map(0, -1.57, 1.57, 1000, 2000);
                int del3_ms = map(0, -1.57, 1.57, 1000, 2000);
                int del4_ms = map(0, -1.57, 1.57, 1000, 2000);
                int del5_ms = map(0, -1.57, 1.57, 1000, 2000);

                servo1_.writeMicroseconds(del1_ms);
                servo2_.writeMicroseconds(del2_ms);
                servo3_.writeMicroseconds(del3_ms);
                servo4_.writeMicroseconds(del4_ms);
                servo5_.writeMicroseconds(del5_ms);

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
            input_[2] = inPacket_.axis3;
            input_[3] = inPacket_.axis4;
            input_[4] = inPacket_.axis5;
        }
    }


    void Wing::forwardInputs(void) {

        // TODO> THESE GUYS MUST BE TUNNED!
        int del1_ms = map(input_[0], -1.57, 1.57, 1000, 2000);
        int del2_ms = map(input_[1], -1.57, 1.57, 1000, 2000);
        int del3_ms = map(input_[2], -1.57, 1.57, 1000, 2000);
        int del4_ms = map(input_[3], -1.57, 1.57, 1000, 2000);
        int del5_ms = map(input_[4], -1.57, 1.57, 1000, 2000);
        
        // Write to the servos
        servo1_.writeMicroseconds(del1_ms);
        servo2_.writeMicroseconds(del2_ms);
        servo3_.writeMicroseconds(del3_ms);
        servo4_.writeMicroseconds(del4_ms);
        servo5_.writeMicroseconds(del5_ms);

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


        // update servo feedback readings,
        int d1 = analogRead(SERVO1_ANALOG_PIN);
        int d2 = analogRead(SERVO2_ANALOG_PIN);
        int d3 = analogRead(SERVO3_ANALOG_PIN);
        int d4 = analogRead(SERVO4_ANALOG_PIN);
        int d5 = analogRead(SERVO5_ANALOG_PIN);
        // then map the analog input to a radian value.
        // Todo: MAPPING MUST BE DONE. each servo has different potentiometer.
        servoFeedback_[0] = map(d1, 0, 940, -1.57, 1.57);
        servoFeedback_[0] = map(d2, 0, 940, -1.57, 1.57);
        servoFeedback_[0] = map(d3, 0, 940, -1.57, 1.57);
        servoFeedback_[0] = map(d4, 0, 940, -1.57, 1.57);
        servoFeedback_[0] = map(d5, 0, 940, -1.57, 1.57);

        // New State                
        float stateNew[18];
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
        stateNew[13] = servoFeedback_[0];
        stateNew[14] = servoFeedback_[1];
        stateNew[15] = servoFeedback_[2];
        stateNew[16] = servoFeedback_[3];
        stateNew[17] = servoFeedback_[4];

        memcpy(state_,stateNew,sizeof(state_));	
	}



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
        outPacket_.d1 = state_[13];
        outPacket_.d2 = state_[14];
        outPacket_.d3 = state_[15];
        outPacket_.d4 = state_[16];
        outPacket_.d5 = state_[17];

        memcpy(packetBuffer_, &outPacket_, sizeof(packetBuffer_));
        udp2_.write(packetBuffer_, sizeof(packetBuffer_));
    }

}