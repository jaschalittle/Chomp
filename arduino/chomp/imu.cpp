#include "I2C.h"
#include "telem_message_stream.h"
#include "MPU6050.h"
#include "imu.h"

//TelemetryMessageStream telemetry_stream;

MPU6050 IMU;
int16_t acceleration[3], angular_rate[3];
int16_t temperature;
uint32_t last_imu_process;
uint32_t imu_period=100000;
int32_t stationary_threshold=200;
int32_t min_valid_sumsq = 3800000;
int32_t max_valid_sumsq = 5000000;
int32_t acceleration_z_threshold = 1900;
bool stationary, upright, imu_read_valid;
int32_t sumsq = 0;
int32_t sum_angular_rate = 0;


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
    // if enough time has passed, read the IMU
    uint32_t now = micros();
    if(now-last_imu_process > imu_period) {
        last_imu_process = now;
        uint8_t imu_err = IMU.getMotion6(
            &acceleration[0], &acceleration[1], &acceleration[2],
            &angular_rate[0], &angular_rate[1], &angular_rate[2]);
        if(imu_err != 0) {
            imu_read_valid = false;
            stationary = false;
            upright = false;
            return;
        }
        imu_read_valid = true;
        temperature = IMU.getTemperature();
        sum_angular_rate = (abs(angular_rate[0]) +
                            abs(angular_rate[1]) +
                            abs(angular_rate[2]));
        sumsq = 0;
        for(uint8_t j=0;j<3;j++) {
            sumsq += (((int32_t)acceleration[j])*((int32_t)acceleration[j]));
        }
        bool meaningful_accel = ((min_valid_sumsq<sumsq) &&
                                 (sumsq<max_valid_sumsq));
        stationary = ((sum_angular_rate<stationary_threshold) &&
                      meaningful_accel);
        upright = (meaningful_accel &&
                   (acceleration_z_threshold < acceleration[2]));
    }
}


void telemetryIMU(void) {
    sendIMUTelem(acceleration, angular_rate, temperature);
    sendORNTelem(stationary, upright, sumsq, sum_angular_rate);
}


bool isUpright(void) {
    return upright;
}


bool isStationary(void) {
    return stationary;
}


bool getOmegaZ(int16_t *omega_z) {
    *omega_z = angular_rate[2];
    return imu_read_valid;
}
