#include "Arduino.h"
#include "sensors.h"

float readMLHPressure(){
  int counts = analogRead(7);
  float voltage = counts * (5.0 / 1023);
  float pressure = (voltage - 0.5) * (500.0/4.0);
  return pressure;
}
