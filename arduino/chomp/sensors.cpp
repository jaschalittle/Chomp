#include "Arduino.h"
#include "sensors.h"
#include "pins.h"

void sensorSetup(){
    pinMode(ANGLE_AI, INPUT);
    pinMode(PRESSURE_AI, INPUT);
}

float readMlhPressure(){
  int counts = analogRead(PRESSURE_AI);
  float voltage = counts * (5.0 / 1023);
  float pressure = (voltage - 0.5) * (500.0/4.0);
  return pressure;
}

// 0 deg is 10% of input voltage, empirically observed to be 100 counts
// 360 deg is 90% of input voltage, empirically observed to be 920 counts
float readAngle(){
  short counts = analogRead(ANGLE_AI);
  float angle = (counts - 110.0) * (180.0 / 419.0);
  return angle;
}

float angularVelocity () {
    // This function could filter low angle values and ignore them for summing. If we only rail to 0V, we could still get a velocity.
    float angle_traversed = 0.0;
    float last_angle = readAngle();
    long read_time = micros();
    // Take 50 readings. This should be 5-10 ms.
    for (int i = 0; i < 50; i++) {
        float new_angle = readAngle();
        angle_traversed += abs(new_angle - last_angle);
        last_angle = new_angle;
    }
    read_time = micros() - read_time / 1000;    // convert to milliseconds
    float angular_velocity = angle_traversed / read_time * 1000; // degrees per second
}
