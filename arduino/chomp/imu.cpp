#include "I2C.h"
#include "telem_message_stream.h"
#include "MPU6050.h"
#include "imu.h"

//TelemetryMessageStream telemetry_stream;
static const int16_t stable_orientation[NUM_STABLE_ORIENTATIONS][3] = {
    {    0,     0,  2048},  // upright
    { 2037,     0,   205},  // left
    {-2027,     0,   287},  // right
    {    0, -1925,  -614},  // front
    {    0,   205, -2037},  // top
    { 1966,   409,  -614},  // top right
    {-1966,   409,  -614},  // top left 
    {  163,  1925,   614},  // tail
};

MPU6050 IMU;
int16_t acceleration[3], angular_rate[3];
int16_t temperature;
uint32_t last_imu_process;
uint32_t imu_period=100000;
int32_t stationary_threshold=50;
int32_t min_valid_accum = 100;
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


// State machine to distribute compute over several cycles
void processIMU(void) {
    // current orientation value to test
    // NUM_STABLE_ORIENTATIONS when all have been tested
    static uint8_t imu_step = NUM_STABLE_ORIENTATIONS;
    // maximum dot product seen so far
    static int32_t max_accum = min_valid_accum;
    // corresponding orientation
    static uint8_t max_orientation = ORN_UNKNOWN;

    // First, check to see if the step is non-zero.
    // If so, we are in the middle of checking
    if(imu_step<NUM_STABLE_ORIENTATIONS && stationary) {
        // Compute the current dot product
        int32_t accum = 0;
        for(uint8_t j=0;j<3;j++) {
            accum += (((int32_t)acceleration[j])*
                      ((int32_t)stable_orientation[imu_step][j]));
        }
        // If this is a new highest, write it down
        if(abs(accum)>abs(best_accum)) {
            max_accum = accum;
            max_orientation = imu_step;
        }
        // if this was the last one, record the best result
        if(++imu_step == NUM_STABLE_ORIENTATIONS) {
            best_accum = max_accum;
            best_orientation = max_orientation;
        }
    }
    // if enough time has passed, read the IMU
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
        // if stationary, trigger a new orientation calculation
        // on the next call
        if(stationary) {
            imu_step = 0;
            max_accum = min_valid_accum;
            max_orientation = ORN_UNKNOWN;
        } else {
            // if not stationary, refuse to guess
            best_orientation = ORN_UNKNOWN;
        }
    }
}


void telemetryIMU(void) {
    sendIMUTelem(acceleration, angular_rate, temperature);
    sendORNTelem(stationary, best_orientation, best_accum, sum_angular_rate);
}


enum Orientation getOrientation(void) {
    return (enum Orientation)best_orientation;
}
