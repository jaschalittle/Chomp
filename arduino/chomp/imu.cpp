#include "I2C.h"
#include "telem_message_stream.h"
#include "MPU6050.h"

//TelemetryMessageStream telemetry_stream;

MPU6050 IMU;
int16_t acceleration[3], angular_rate[3];
int16_t temperature;
uint32_t last_imu_process;
uint32_t imu_period=100000;


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
    }
}


void telemetryIMU(void) {
    sendIMUTelem(acceleration, angular_rate, temperature);
}
