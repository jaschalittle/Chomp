#include "chomp_main.h"

extern HardwareSerial& Debug = Serial;
extern HardwareSerial& Xbee = Serial;           // RX pin 0, TX pin 1
extern HardwareSerial& DriveSerial = Serial1;   // RX pin 19, TX pin 18
extern HardwareSerial& LeddarSerial = Serial2;  // RX pin 17, TX pin 16
extern HardwareSerial& Sbus = Serial3;          // RX pin 15, TX pin 14

extern volatile bool g_enabled = false;

void setup(){
  chompSetup();
}

void loop(){
  chompLoop();
}
