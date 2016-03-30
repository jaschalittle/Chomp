#ifndef SENSORS_H
#define SENSORS_H

void sensorSetup();
bool readMlhPressure(int16_t* pressure);
bool readAngle(int16_t* angle);
bool angularVelocity(int16_t* angular_velocity);
bool angularVelocityBuffered(int16_t* angular_velocity, const int16_t* angle_data, uint16_t datapoints_buffered);

#endif // SENSORS_H
