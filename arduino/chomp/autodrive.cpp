#include <Arduino.h>
#include "targeting.h"
#include "utils.h"


static int32_t steer_p = 512;
static int32_t steer_d = 0;
static int32_t steer_max = 500;
static int32_t drive_p = 256;
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
    bool valid = tracked_object.valid(micros());
    if(valid) {
        *steer_bias  = -steer_p * tracked_object.y/16384L;
        *steer_bias += -steer_d * tracked_object.vy/16384L;
        *steer_bias  = clip(*steer_bias, -steer_max, steer_max);

        *drive_bias  = drive_p * (tracked_object.x-(int32_t)depth*16L)/16384L;
        *drive_bias += drive_d * tracked_object.vx/16384L;
        *drive_bias  = clip(*drive_bias, -drive_max, drive_max);
    }
    return valid;
}
