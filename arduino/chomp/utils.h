#ifndef UTILS_H
#define UTILS_H

void safeDigitalWrite( uint32_t ulPin, uint32_t ulVal); // matches Arduino

void debug_print(uint16_t num);
void debug_print(int16_t num);
void debug_print(char* str);
void debug_println(uint16_t num);
void debug_println(int16_t num);
void debug_println(char* str);
#endif
