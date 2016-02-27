#include "Arduino.h"
#include "xbee.h"

HardwareSerial & Xbee = Serial2;
void xbee_init(){
  Xbee.begin(57600);
}

