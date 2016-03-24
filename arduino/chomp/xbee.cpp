#include "Arduino.h"
#include "xbee.h"
#include "pins.h"


void xbeeInit(){
  Xbee.begin(115200);
  pinMode(XBEE_CTS, INPUT);
}

