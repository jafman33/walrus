
// This is the servo example

// #include <Arduino.h>
// #include "Servo.h"
 
// // Servo 1 setup 
// Servo servo1;           // create servo object to control a servo 
// int servo1_pin = 23;    // Servo 1 attaches to pin 23 
// int del1_out = 0;       // variable to store the servo position 
// float del1_dmd = 0.01;  // incoming demand from ROS2 network
// int trim1 = 50;         // variable for trimming the servo to zero

// void setup() 
// { 
//   servo1.attach(23);    // attaches the servo on pin
//   Serial.begin(9600);   // Starts the serial port
// } 
 
// void loop() 
// {
//   del1_out = map(del1_dmd, -1.57, 1.57, 1000, 2000);     // scale delta input to microseconds
//   servo1.writeMicroseconds(del1_out+trim1);              // write to the servo
//   // Serial.println(del1_out+trim1);                        // print the output
//   delay(10);                                             // waits 10ms 
// } 

