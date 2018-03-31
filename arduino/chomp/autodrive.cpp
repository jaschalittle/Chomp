#include <Arduino.h>
#include "targeting.h"
#include "utils.h"


static int32_t steer_p = 0;
static int32_t steer_d = 0;
static int32_t steer_max = 500;
static int32_t drive_p = 0;
static int32_t drive_d = 0;
static int32_t drive_max = 500;


void setDriveControlParams(int16_t p_steer_p,
                           int16_t p_steer_d,
                           int16_t p_drive_p,
                           int16_t p_drive_d) {
    steer_p = p_steer_p;
    steer_d = p_steer_d;
    drive_p = p_drive_p;
    drive_d = p_drive_d;
}

bool pidSteer(const Track &tracked_object,
              int16_t depth, int16_t *drive_bias, int16_t *steer_bias) {
    uint32_t now = micros();
    if(tracked_object.valid(now)) {
        *steer_bias = -clip(steer_p * tracked_object.y/16384L,
                            -steer_max, steer_max);
        *drive_bias = clip(drive_p * (tracked_object.x-(int32_t)depth*16L)/16384L,
                           -drive_max, drive_max);
    }
    return tracked_object.valid(now);
}
