#include "defines.h"
#include <AsyncUDP_Teensy41.h>        // https://github.com/khoih-prog/AsyncUDP_Teensy41
#include <Ticker.h>                   // https://github.com/sstaub/Ticker
#include "Servo.h"
 
///////////////
// Servo Setup
///////////////
Servo servo1;           // create servo object to control a servo 
int servo1_pin = 23;    // Servo 1 attaches to pin 23 
int del1_out = 0;       // variable to store the servo position 
float del1_dmd = 0.0;   // incoming demand from ROS2 network
int trim1 = 50;         // variable for trimming the servo to zero

// Define Structs
JoyStick myJoystick;                             
outPacket myPacket;

// Definitions
AsyncUDP udp1; // A UDP instance to let us send and receive packets over UDP 
AsyncUDP udp2; // A UDP instance to let us send and receive packets over UDP 

IPAddress remoteIP = IPAddress(10, 250, 225, 92); 
#define RECEIVE_PORT 1560
#define SEND_PORT 1561

const int PACKET_SIZE = 4;        // NTP timestamp is in the first 48 bytes of the message
byte packetBuffer[PACKET_SIZE];   // buffer to hold incoming and outgoing packets


///////////////////////
// Send outgoing packet 
void sendPacket(void) {
  // createPacket();
  myPacket.depth = del1_dmd;
  memset(packetBuffer, 0, PACKET_SIZE);
  memcpy(packetBuffer, &myPacket, PACKET_SIZE);
  udp2.write(packetBuffer, PACKET_SIZE);
}


/////////////////////////
// Parse Incoming Packets
void parsePacket(AsyncUDPPacket packet)
{
  if (packet.length()) {    
    memcpy(&myJoystick, packet.data(), packet.length());  
    del1_dmd = myJoystick.axis1;
  }
}


/////////////
// Void Setup 
void setup() {
  Serial.begin(9600);
  servo1.attach(23);    // attaches the servo on pin

  Serial.print("Initialize Ethernet using DHCP => ");
  Ethernet.begin();

  if (!Ethernet.waitForLocalIP(5000)) {
    Serial.println(F("Failed to configure Ethernet"));
    if (!Ethernet.linkStatus())
      Serial.println(F("Ethernet cable is not connected."));
    while (true)
      delay(1);
  } else {
    Serial.print(F("Connected! Teensy IP address:"));
    Serial.println(Ethernet.localIP());
  }
  delay(1000);

  if (udp1.listen(RECEIVE_PORT)) {
    udp1.onPacket([](AsyncUDPPacket incomingPacket) {
      parsePacket(incomingPacket);
    });
  }
  udp2.connect(remoteIP, SEND_PORT);
}


/////////////
// Void Loop
void loop() {
  sendPacket();

  // servo here!
  del1_out = map(del1_dmd, -1.57, 1.57, 1000, 2000);     // scale delta input to microseconds
  servo1.writeMicroseconds(del1_out+trim1);              // write to the servo
}
