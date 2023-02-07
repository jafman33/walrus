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
        inPacket_(),
        outPacket_(),
        state_{0.0F},
        input_{0.0F}, 
        packetBuffer_{PACKET_SIZE_OUT}
	{}
    

    void Wing::init(void) {
        // Initialize Serial Port
        Serial.begin(9600);

        // Initialize Servo
        servo1_.attach(SERVO1_PIN);    // attaches the servo on pin

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
            if (udp1_.listen(RECEIVE_PORT)) {
                udp1_.onPacket([&](AsyncUDPPacket packet) {
                    parsePacket(packet);
                });
            }
            // Connect to IP and Port for sending packets
            udp2_.connect(IPAddress(10, 250, 225, 92), SEND_PORT);
        }
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
                // receive input packet and update - setup through asynchonous function on init()
                // parsePacket(); 
                // updateInputs();
                forwardInputs();

                // IMU, depth, etc.
                // updateState();
                publishState();



                break;
            }

            case STATUS::FAILURE:
			{
                // Set Actuators to zero
                servo1_.writeMicroseconds(1500);
                servo2_.writeMicroseconds(1500);  
                publishState();

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
        outPacket_.depth = input_[0];
        memset(packetBuffer_, 0, sizeof(packetBuffer_));
        memcpy(packetBuffer_, &outPacket_, sizeof(packetBuffer_));
        udp2_.write(packetBuffer_, sizeof(packetBuffer_));
    }

    void Wing::forwardInputs(void) {
        // check input_ is within range ()
        int del1_ms = map(input_[0], -1.57, 1.57, 1000, 2000);
        int del2_ms = map(input_[1], -1.57, 1.57, 1000, 2000);
        servo1_.writeMicroseconds(del1_ms);
        servo2_.writeMicroseconds(del2_ms);
    }

}