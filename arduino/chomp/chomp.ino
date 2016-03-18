#include "SoftwareSerial.h"
#include "chomp_main.h"

extern HardwareSerial& Debug = Serial;          // RX pin 0, TX pin 1
extern HardwareSerial& Xbee = Serial1;          // RX pin 19, TX pin 18
extern HardwareSerial& LeddarSerial = Serial2;  // RX pin 17, TX pin 16
extern HardwareSerial& Sbus = Serial3;          // RX pin 15, TX pin 14
// pins 22 and 23 not pinned out on Grove shield, so they are safe to "throw away" to initialize TX
SoftwareSerial mySerial4(22, 6);                // RX pin 22, TX pin 6
SoftwareSerial mySerial5(23, 7);                // RX pin 23, TX pin 7
extern SoftwareSerial& LeftWheelSerial = mySerial4;
extern SoftwareSerial& RightWheelSerial = mySerial5;

void setup(){
  chompSetup();
}

void loop(){
  chompLoop();
}

