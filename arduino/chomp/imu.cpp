#include "I2C.h"
#include "telem_message_stream.h"
#include "MPU6050.h"

//TelemetryMessageStream telemetry_stream;
#define NUM_STABLE_ORIENTATIONS 8
static const int16_t stable_orientation[NUM_STABLE_ORIENTATIONS][3] = {
    {    0,     0,  2048},
    { 2037,     0,   205},
    {-2027,     0,   287},
    {    0, -1925,  -614},
    {    0,   205, -2037},
    { 1966,   409,  -614},
    {-1966,   409,  -614},
    {  163,  1925,   614},
};

MPU6050 IMU;
int16_t acceleration[3], angular_rate[3];
int16_t temperature;
uint32_t last_imu_process;
uint32_t imu_period=100000;
int32_t stationary_threshold=50;
int32_t min_accum = 100;
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
        sum_angular_rate = (abs(angular_rate[0]) +
                            abs(angular_rate[1]) +
                            abs(angular_rate[2]));
        stationary = sum_angular_rate<stationary_threshold;
        if(stationary) {
            best_accum = min_accum;
            best_orientation = NUM_STABLE_ORIENTATIONS;
            for(size_t i=0;i<NUM_STABLE_ORIENTATIONS;i++) {
                int32_t accum = 0;
                for(uint8_t j=0;j<3;j++) {
                    accum += (((int32_t)acceleration[j])*
                              ((int32_t)stable_orientation[i][j]));
                }
                if(abs(accum)>abs(best_accum)) {
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
