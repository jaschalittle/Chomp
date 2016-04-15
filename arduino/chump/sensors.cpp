#include "Arduino.h"
#include "sensors.h"
#include "pins.h"

void sensorSetup(){
    pinMode(ANGLE_AI, INPUT);
    pinMode(PRESSURE_AI, INPUT);
}

// static const uint32_t pressure_sensor_range = 920 - 102;
bool readMlhPressure(int16_t* pressure){
  uint16_t counts = analogRead(PRESSURE_AI);
  if (counts < 102) { *pressure = 0; return true; }
  if (counts > 920) { *pressure = 500; return true; }
  
  *pressure = (int16_t) (counts - 102) * 11 / 18; // safe with 16 bit uints, accurate to 0.02%%
  return true;
}

// 0 deg is 10% of input voltage, empirically observed to be 100 counts
// 360 deg is 90% of input voltage, empirically observed to be 920 counts
bool readAngle(uint16_t* angle){
    uint16_t counts = analogRead(ANGLE_AI);
    if ( counts < 50 ) { return false; } // Failure mode in shock, rails to 0;
    if ( counts < 102 ) { *angle = 0; return true; }
    if ( counts > 920 ) { *angle = 360; return true; }

    *angle = (int16_t) (counts - 102) * 11 / 25;  // safe with 16 bit uints, accurate to 0.02%%
    return true;
}

// bool relativeAngleReference(uint16_t* counts_reference) {
//     uint16_t counts = analogRead(ANGLE_AI);
//     if ( counts < 50 ) { return false; } // Failure mode in shock, rails to 0;
//     if ( counts < 102 ) { counts = 102; return true; }
//     if ( counts > 920 ) { counts = 920; return true; }

//     *counts_reference = counts - 102;  // safe with 16 bit uints, accurate to 0.02%%
//     return true;
// }

// bool relativeAngle(int16_t* angle, uint16_t counts_reference) {
//     uint16_t counts = analogRead(ANGLE_AI);
//     if ( counts < 50 ) { return false; } // Failure mode in shock, rails to 0;
//     if ( counts < 102 ) { counts = 102; }
//     if ( counts > 920 ) { counts = 920; }
//     counts -= 102;
//     uint16_t diff = counts_reference - counts;
//     Debug.print(counts_reference);
//     Debug.print("\t");
//     Debug.println(counts);
//     *angle = diff * 11 / 25;  // safe with 16 bit uints, accurate to 0.02%%
//     return true;
// }

bool angularVelocity (int16_t* angular_velocity) {
    // This function could filter low angle values and ignore them for summing. If we only rail to 0V, we could still get a velocity.
    int16_t angle_traversed = 0;
    int16_t abs_angle_traversed = 0;
    uint16_t last_angle;
    bool angle_read_ok = readAngle(&last_angle);
    uint16_t new_angle;
    int16_t delta;
    uint32_t read_time = micros();
    // Take 50 readings. This should be 5-10 ms. 1 rps = 2.78 deg/s
    uint8_t num_readings = 50;
    for (uint8_t i = 0; i < num_readings; i++) {
        if (readAngle(&new_angle)) {
            delta = (int16_t) new_angle - last_angle;
            abs_angle_traversed += abs(delta);
            angle_traversed += delta;
            last_angle = new_angle;
        } else {
            angle_read_ok = false;
        }
    }
    // if angle read ever sketchy or if data too noisy, do not return angular velocity
    if (angle_read_ok && abs(angle_traversed) - angle_traversed < num_readings * 2) {
        read_time = (micros() - read_time) / 1000;    // convert to milliseconds
        *angular_velocity = angle_traversed * 1000 / read_time;  // degrees per second
        return true;
    } else {
        return false;
    }
}

bool angularVelocityBuffered (int16_t* angular_velocity, const uint16_t* angle_data, uint16_t datapoints_buffered) {
    const uint16_t DATAPOINTS_TO_AVERAGE = 100;
    // do not report velocity if too few datapoints have been buffered
    if (datapoints_buffered < DATAPOINTS_TO_AVERAGE) {
        return false;
    }
    int16_t angle_traversed = 0;
    int16_t abs_angle_traversed = 0;
    int16_t delta;
    uint32_t read_time = micros();
    for (uint16_t i = datapoints_buffered - DATAPOINTS_TO_AVERAGE + 1; i < datapoints_buffered; i++) {
        delta = (int16_t) angle_data[i] - angle_data[i-1];
        abs_angle_traversed += abs(delta);
        angle_traversed += delta;
    }
    // if angle data too noisy, do not return angular velocity
    if (abs(angle_traversed) - angle_traversed < DATAPOINTS_TO_AVERAGE * 2) {
        read_time = (micros() - read_time) / 1000;    // convert to milliseconds
        *angular_velocity = angle_traversed * 1000 / read_time;  // degrees per second
        return true;
    } else {
        return false;
    }
}
