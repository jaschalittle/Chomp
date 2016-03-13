#include "Arduino.h"
#include "xbee.h"
#include "pins.h"


void xbeeInit(){
  Xbee.begin(57600);
}

