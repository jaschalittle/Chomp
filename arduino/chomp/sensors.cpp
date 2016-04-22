#include "Arduino.h"
#include "sensors.h"
#include "pins.h"
// #include "imu.h"
#include "drive.h"
#include <SPI.h>
#include "MPU9250.h"

#define SPI_CLOCK 8000000  // 8MHz clock works.

#define SCK_PIN  52
#define SS_PIN   A11 
#define INT_PIN  3
#define LED      13

MPU9250 mpu(SPI_CLOCK, SS_PIN);

uint32_t last_imu_read;

void sensorSetup(){
    pinMode(ANGLE_AI, INPUT);
    pinMode(PRESSURE_AI, INPUT);
    // SPI.begin();
    // delay(2000);
    // mpu.init(true, true);
    // delay(2000);
    // mpu.calib_acc();
    // last_imu_read = micros();
}

// static const uint32_t pressure_sensor_range = 920 - 102;
bool readMlhPressure(int16_t* pressure){
    uint16_t counts = analogRead(PRESSURE_AI);
    if (counts < 102) { *pressure = 0; return true; }
    if (counts > 920) { *pressure = 500; return true; }

    *pressure = (int16_t) (counts - 102) * 11 / 18; // safe with 16 bit uints, accurate to 0.02%%
    return true;
}

#define MIN_ANGLE_ANALOG_READ 50
#define ZERO_ANGLE_ANALOG_READ 102
#define MAX_ANGLE_ANALOG_READ 920
// 0 deg is 10% of input voltage, empirically observed to be 100 counts
// 360 deg is 90% of input voltage, empirically observed to be 920 counts
bool readAngle(uint16_t* angle){
    uint16_t counts = analogRead(ANGLE_AI);
    if ( counts < MIN_ANGLE_ANALOG_READ ) { return false; } // Failure mode in shock, rails to 0;
    if ( counts < ZERO_ANGLE_ANALOG_READ ) { *angle = 0; return true; }
    if ( counts > MAX_ANGLE_ANALOG_READ ) { *angle = 359; return true; }

    *angle = (int16_t) (counts - ZERO_ANGLE_ANALOG_READ) * 11 / 25;  // safe with 16 bit uints, accurate to 0.02%%
    return true;
}

#define ANGLE_CONVERSION_FLOAT 0.4400978f  // 360 / (920 - 102)
bool readAngleFloat(float* angle){
    uint16_t counts = analogRead(ANGLE_AI);
    if ( counts < MIN_ANGLE_ANALOG_READ ) { return false; } // Failure mode in shock, rails to 0;
    if ( counts < ZERO_ANGLE_ANALOG_READ ) { *angle = 0.0; return true; }
    if ( counts > MAX_ANGLE_ANALOG_READ ) { *angle = 359.9; return true; }

    *angle = (float) (counts - ZERO_ANGLE_ANALOG_READ) * ANGLE_CONVERSION_FLOAT;
    return true;
}


bool angularVelocity (float* angular_velocity) {
    // This function could filter low angle values and ignore them for summing. If we only rail to 0V, we could still get a velocity.
    float angle_traversed = 0;
    float abs_angle_traversed = 0;
    float last_angle;
    bool angle_read_ok = readAngleFloat(&last_angle);
    float new_angle;
    float delta;
    uint32_t read_time = micros();
    // Take 50 readings. This should be ~25 ms. 1 rps = 2.78 deg/s
    uint8_t num_readings = 50;
    for (uint8_t i = 0; i < num_readings; i++) {
        if (readAngleFloat(&new_angle)) {
            delta = new_angle - last_angle;
            abs_angle_traversed += abs(delta);
            angle_traversed += delta;
            last_angle = new_angle;
            delayMicroseconds(300);
        } else {
            angle_read_ok = false;
        }
    }
    // if angle read ever sketchy or if data too noisy, do not return angular velocity
    if (angle_read_ok && abs(angle_traversed) - angle_traversed < num_readings * 2) {
        float read_time_seconds = (float) (micros() - read_time) / 1000000.0f;    // convert to milliseconds
        *angular_velocity = angle_traversed / read_time_seconds;  // degrees per second
        return true;
    } else {
        return false;
    }
}

// Returns absolute angular velocity in deg/sec
bool angularVelocityBuffered (float* angular_velocity, const uint16_t* angle_data, uint16_t datapoints_buffered, uint16_t timestep_ms ) {
    const uint32_t DATAPOINTS_TO_AVERAGE = 20;
    // do not report velocity if too few datapoints have been buffered
    if (datapoints_buffered < DATAPOINTS_TO_AVERAGE) {
        return false;
    }
    
    uint32_t angle_min = 360;
    uint32_t angle_max = 0;
    for (uint16_t i = datapoints_buffered - DATAPOINTS_TO_AVERAGE; i < datapoints_buffered; i++) {
        if (angle_data[i] < angle_min) { 
           angle_min = angle_data[i];
         }
        if (angle_data[i] > angle_max) { 
            angle_max = angle_data[i];
          }
    }

    *angular_velocity = (angle_max - angle_min) * 1000 / (DATAPOINTS_TO_AVERAGE * timestep_ms);
    return true;
}

float getYaccel() {
    // reading * voltage (3.3v?) * ADC range = voltage, - zero voltage = signal voltage, / sensitivity = g
    // return m/s/s
    return mpu.accel_data[1] * 9.8 * 100;
}

float getZgyro() {
    // reading * Vref / ADC range - Vzero / sensitivity = deg/s, / 360 * 2 * PI = rad/s?
    return mpu.gyro_data[2] * DEG_TO_RAD;
}

#define IMU_BUFFER_SIZE 5
#define Y_ACCEL_STOPPED_THRESHOLD 0.1  // figure out accel data at rest and set this properly
#define DRIVE_STOPPED_THRESHOLD 10  // figure out what this should be. max drive command that doesn't move chomp.
#define CHUMP_DRIVE_COEFF 0.09345191f  // cm/s per unit drive command on Chump
float y_accel_readings[IMU_BUFFER_SIZE];
float z_gyro_readings[IMU_BUFFER_SIZE];
float imu_read_timesteps[IMU_BUFFER_SIZE];
uint8_t imu_buffer_index = 0;
uint32_t last_imu_time;
void readImu(float* our_forward_vel, float* our_angular_vel) {
    // get y accel reading, z gyro reading, timesteps, and add to ring buffers
    mpu.read_all();
    y_accel_readings[imu_buffer_index] = getYaccel();
    z_gyro_readings[imu_buffer_index] = getZgyro();
    imu_read_timesteps[imu_buffer_index] = (micros() - last_imu_time) / 1000;
    last_imu_time = micros();
    imu_buffer_index = (imu_buffer_index + 1) % IMU_BUFFER_SIZE;
    float avg_y_accel = 0.0;
    for (uint8_t i = 0; i < IMU_BUFFER_SIZE; i++) { avg_y_accel += y_accel_readings[i]; }
    avg_y_accel /= IMU_BUFFER_SIZE;
    
    // if drive command below threshold (i.e. neutral) and y accel is below threshold, set velocity to 0 and zero out y accel buffer
    // else, integrate over accel reading history to estimate velocity
    if (avg_y_accel < Y_ACCEL_STOPPED_THRESHOLD && getAvgDriveCommand() < DRIVE_STOPPED_THRESHOLD) { 
        *our_forward_vel = 0.0;
        for (uint8_t i = 0; i < IMU_BUFFER_SIZE; i++) { y_accel_readings[i] = 0.0; }
    } else {
        float velocity_estimate = 0.0;
        // iterate over ring buffer in two halves
        for (uint8_t i = imu_buffer_index; i < IMU_BUFFER_SIZE; i++) {
            velocity_estimate += y_accel_readings[i] * imu_read_timesteps[i] / 1000;
        }
        for (uint8_t i = 0; i < imu_buffer_index; i++) {
            velocity_estimate += y_accel_readings[i] * imu_read_timesteps[i] / 1000;
        }
        *our_forward_vel = 0.9 * velocity_estimate + 0.1 * getAvgDriveCommand() * CHUMP_DRIVE_COEFF;
    }
    
    // estimate angular velocity by averaging z gyro buffer
    float angular_velocity_estimate = 0.0;
    // iterate over ring buffer in two halves
    for (uint8_t i = imu_buffer_index; i < IMU_BUFFER_SIZE; i++) {
        angular_velocity_estimate += z_gyro_readings[i] * imu_read_timesteps[i];
    }
    for (uint8_t i = 0; i < imu_buffer_index; i++) {
        angular_velocity_estimate += z_gyro_readings[i] * imu_read_timesteps[i];
    }
    angular_velocity_estimate /= IMU_BUFFER_SIZE;
    *our_angular_vel = angular_velocity_estimate / 360 * 2 * PI;
}

// call this when tracking is started or restarted to store an initial IMU timestamp and zero out buffers
// don't bother taking IMU readings, because they are useless without a timestep
void resetImu() {
    last_imu_time = micros();
    for (uint8_t i = 0; i < IMU_BUFFER_SIZE; i++) {
        y_accel_readings[i] = 0.0;
        z_gyro_readings[i] = 0.0;
    }
}
