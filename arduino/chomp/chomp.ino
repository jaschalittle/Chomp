#include "chomp_main.h"

HardwareSerial& Debug = Serial;
HardwareSerial& Xbee = Serial;           // RX pin 0, TX pin 1
HardwareSerial& DriveSerial = Serial1;   // RX pin 19, TX pin 18
HardwareSerial& LeddarSerial = Serial2;  // RX pin 17, TX pin 16
HardwareSerial& Sbus = Serial3;          // RX pin 15, TX pin 14

volatile bool g_enabled = false;

void setup(){
  chompSetup();
}

void loop(){
  chompLoop();
}
