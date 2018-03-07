#include "I2C.h"
#include "telem_message_stream.h"
#include "MPU6050.h"

//TelemetryMessageStream telemetry_stream;
#define NUM_STABLE_ORIENTATIONS 8
static const int16_t stable_orientation[NUM_STABLE_ORIENTATIONS][3] = {
    {    0,     0,  2048},
    {    0,  2048,     0},
    { 2048,     0,     0},
    {    0,  1448,  1448},
    { 1448,     0,  1448},
    { 1448,  1448,     0},
    { 1182,  1182,  1182},
    { 1182, -1182,  1182}
};

MPU6050 IMU;
int16_t acceleration[3], angular_rate[3];
int16_t temperature;
uint32_t last_imu_process;
uint32_t imu_period=100000;
int32_t stationary_threshold=100;
bool stationary;
size_t best_orientation=NUM_STABLE_ORIENTATIONS;
int32_t best_accum;
int32_t sum_angular_rate;


void initializeIMU(void) {
    I2c.begin();
    I2c.setSpeed(false);
    I2c.timeOut(2);
    //I2c.scan(telemetry_stream);
    IMU.initialize();
    debug_print(String("IMU.getDeviceID() = ") + IMU.getDeviceID());
    IMU.setFullScaleGyroRange(MPU6050_GYRO_FS_2000);
    IMU.setFullScaleAccelRange(MPU6050_ACCEL_FS_16);
    IMU.setDLPFMode(MPU6050_DLPF_BW_5);
    last_imu_process = micros();
}


void processIMU(void) {
    uint32_t now = micros();
    if(now-last_imu_process > imu_period) {
        last_imu_process = now;
        temperature = IMU.getTemperature();
        IMU.getMotion6(&acceleration[0], &acceleration[1], &acceleration[2],
                       &angular_rate[0], &angular_rate[1], &angular_rate[2]);
        sum_angular_rate = labs(angular_rate[0]) + labs(angular_rate[1]) + labs(angular_rate[2]);
        stationary = sum_angular_rate<stationary_threshold;
        if(stationary) {
            best_accum = 0;
            for(size_t i=0;i<NUM_STABLE_ORIENTATIONS;i++) {
                int32_t accum = 0;
                for(uint8_t j=0;j<3;i++) {
                    accum += acceleration[j]*stable_orientation[i][j];
                }
                if(accum>best_accum) {
                    best_accum = accum;
                    best_orientation = i;
                }
            }
        } else {
            best_orientation = NUM_STABLE_ORIENTATIONS;
        }
    }
}


void telemetryIMU(void) {
    sendIMUTelem(acceleration, angular_rate, temperature);
    sendORNTelem(stationary, best_orientation, best_accum, sum_angular_rate);
}
