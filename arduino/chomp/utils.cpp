#include "Arduino.h"
#include "utils.h"
#include "pins.h"

void safeDigitalWrite( uint32_t ulPin, uint32_t ulVal){
  if (g_enabled){
    digitalWrite(ulPin, ulVal);
  }
}


