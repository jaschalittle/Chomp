#include <Arduino.h>
#include "targeting.h"
#include "utils.h"
#include "imu.h"
#include "telem.h"


static int32_t steer_p = 512;
static int32_t steer_d = 0;
static int32_t steer_max = 500;
static int32_t gyro_gain = 0;
static int32_t drive_p = 256;
static int32_t drive_d = 0;
static int32_t drive_max = 500;
static uint32_t autodrive_telem_interval = 10000;
static uint32_t last_autodrive_telem = 0;


void setDriveControlParams(int16_t p_steer_p,
                           int16_t p_steer_d,
                           int16_t p_steer_max,
                           int16_t p_gyro_gain,
                           int16_t p_drive_p,
                           int16_t p_drive_d,
                           int16_t p_drive_max
                           ) {
    steer_p = p_steer_p;
    steer_d = p_steer_d;
    gyro_gain = p_gyro_gain;
    steer_max = p_steer_max;
    drive_p = p_drive_p;
    drive_d = p_drive_d;
    drive_max = p_drive_max;
}

bool pidSteer(const Track &tracked_object,
              int16_t depth, int16_t *drive_bias, int16_t *steer_bias) {
    int32_t now=micros();
    bool valid = tracked_object.valid(now);
    if(valid) {
        int32_t bias;
        int32_t theta = tracked_object.angle();
        int32_t vtheta = tracked_object.vtheta();
        bias  = steer_p * theta/16384L;
        bias += steer_d * vtheta/16384L;
        int16_t omegaZ = 0;
        if(getOmegaZ(&omegaZ)) {
            bias += -gyro_gain*omegaZ/1024;
        }
        *steer_bias  = clip(bias, -steer_max, steer_max);

        bias  = drive_p * (tracked_object.x-(int32_t)depth*16L)/16384L;
        bias += drive_d * tracked_object.vx/16384L;
        *drive_bias  = clip(bias, -drive_max, drive_max);
        if(now - last_autodrive_telem > autodrive_telem_interval) {
            sendAutodriveTelemetry(*steer_bias,
                                   *drive_bias,
                                   clip(theta, -32768L, 32767L),
                                   clip(vtheta, -32768L, 32767L));
        }
    }
    return valid;
}
