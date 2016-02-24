#include "Arduino.h"
#include "sensors.h"
#include "pins.h"

float readMLHPressure(){
  int counts = analogRead(PRESSURE_AI);
  float voltage = counts * (5.0 / 1023);
  float pressure = (voltage - 0.5) * (500.0/4.0);
  return pressure;
}

// 0 deg is 10% of input voltage, empirically observed to be 100 counts
// 360 deg is 90% of input voltage, empirically observed to be 920 counts
float readAngle(){
  int counts = analogRead(ANGLE_AI);
  float angle = (counts - 100.0) * (360.0 / 820.0);
  return angle;
}

