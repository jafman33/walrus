#include "defines.h"
#include <AsyncUDP_Teensy41.h>        // https://github.com/khoih-prog/AsyncUDP_Teensy41
#include <Ticker.h>                   // https://github.com/sstaub/Ticker
#include "Servo.h"
 
// Servo 1 setup 
Servo servo1;           // create servo object to control a servo 
int servo1_pin = 23;    // Servo 1 attaches to pin 23 
int del1_out = 0;       // variable to store the servo position 
float del1_dmd = 0.0;  // incoming demand from ROS2 network
int trim1 = 50;         // variable for trimming the servo to zero


void parsePacket(AsyncUDPPacket packet);
AsyncUDP udp;
#define UDP_REQUEST_INTERVAL_MS     60000  //600000
void sendRequest();
Ticker sendUDPRequest(sendRequest, UDP_REQUEST_INTERVAL_MS, 0, MILLIS);
void sendRequest()
{
  UDP_LOGDEBUG("Send broadcast");
  udp.broadcast("Anyone here?");
}


///////////////
// Define Struc 
///////////////
struct __attribute__((packed)) JoyStick {
  // The data sending from Ros2 node is a series of bytes that can be represented as:
  uint32_t time;
  float axis1;
  float axis2;
  uint32_t sync;
};


///////////////
// Parse Packet 
///////////////
void parsePacket(AsyncUDPPacket packet)
{
  // Since the packet data is an array of uint8_t, copy the data into a variable of JoyStick_t type, so that you can access the data as a struct.
  int packetSize = packet.length();
  if (packetSize) {
    Serial.printf("Received %d bytes\n", packetSize);
    
      JoyStick myJoystick;                      //create a struct
      memcpy(&myJoystick, packet.data(), packetSize);  //copy packet array to the a struct
      Serial.printf("%u %f %f %u\n", 
                    myJoystick.time, 
                    myJoystick.axis1, 
                    myJoystick.axis2, 
                    myJoystick.sync);           // access the data in the struct

    del1_dmd = myJoystick.axis1;
  }
}

/////////////
// Void Setup 
/////////////
void setup() {
  Serial.begin(9600);
  servo1.attach(23);    // attaches the servo on pin

  Serial.print("Initialize Ethernet using DHCP => ");
  Ethernet.begin();

  if (!Ethernet.waitForLocalIP(5000))
  {
    Serial.println(F("Failed to configure Ethernet"));
    if (!Ethernet.linkStatus())
      Serial.println(F("Ethernet cable is not connected."));
    while (true)
      delay(1);
  }
  else
    Serial.print(F("Connected!"));
  delay(1000);


  if (udp.listen(1560))
  {
    udp.onPacket([](AsyncUDPPacket packet)
    {
      parsePacket(packet);
      packet.printf("Got %u bytes of data", packet.length());
    });
  }

  sendRequest();
  sendUDPRequest.start(); //start the ticker
}





void loop() {
  sendUDPRequest.update();

  // servo here!
  del1_out = map(del1_dmd, -1.57, 1.57, 1000, 2000);     // scale delta input to microseconds
  servo1.writeMicroseconds(del1_out+trim1);              // write to the servo
}
