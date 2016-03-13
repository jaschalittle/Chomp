#include "Arduino.h"
#include "xbee.h"

HardwareSerial & Xbee = Serial2;
void xbeeInit(){
  Xbee.begin(57600);
}

