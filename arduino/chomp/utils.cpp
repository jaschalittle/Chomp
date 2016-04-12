#include "Arduino.h"
#include "utils.h"
#include "pins.h"

void safeDigitalWrite( uint32_t ulPin, uint32_t ulVal){
  if (g_enabled){
    digitalWrite(ulPin, ulVal);
  }
}

void debug_print(uint16_t num){
#ifdef HARD_WIRED
  Debug.print(num);
#endif
}

void debug_print(int16_t num){
#ifdef HARD_WIRED
  Debug.print(num);
#endif
}

void debug_print(char* str){
#ifdef HARD_WIRED
  Debug.print(str);
#endif
}

void debug_println(uint16_t num){
#ifdef HARD_WIRED
  Debug.println(num);
#endif 
}

void debug_println(int16_t num){
#ifdef HARD_WIRED
  Debug.println(num);
#endif
}

void debug_println(char* str){
#ifdef HARD_WIRED
  Debug.println(str);
#endif 
}

