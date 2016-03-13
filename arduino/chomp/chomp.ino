#include "chump_main.h"

extern HardwareSerial& Debug = Serial;
extern HardwareSerial& Xbee = Serial1;
extern HardwareSerial& LeddarSerial = Serial2;
extern HardwareSerial& Sbus = Serial3;

void setup(){
  chumpSetup();
}

void loop(){
  chumpLoop();
}

