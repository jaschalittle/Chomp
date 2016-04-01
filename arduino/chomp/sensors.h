#ifndef SENSORS_H
#define SENSORS_H

void sensorSetup();
bool readMlhPressure(int16_t* pressure);
bool readAngle(uint16_t* angle);
// bool relativeAngleReference(uint16_t* counts_reference);
// bool relativeAngle(int16_t* angle, uint16_t counts_reference);
bool angularVelocity(int16_t* angular_velocity);
bool angularVelocityBuffered(int16_t* angular_velocity, const uint16_t* angle_data, uint16_t datapoints_buffered);

#endif // SENSORS_H
