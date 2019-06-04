#include <Arduino.h>
#include <avr/eeprom.h>
#include "targeting.h"
#include "utils.h"
#include "imu.h"
#include "telem.h"

static void saveDriveControlParameters();

struct DriveControlParams {
    int32_t steer_p;
    int32_t steer_d;
    int32_t steer_max;
    int32_t gyro_gain;
    int32_t drive_p;
    int32_t drive_d;
    int32_t drive_max;
    uint32_t autodrive_telem_interval;
} __attribute__((packed));

static struct DriveControlParams EEMEM saved_params = {
    .steer_p = 3000,
    .steer_d = 0,
    .steer_max = 600,
    .gyro_gain = 0,
    .drive_p = 1500,
    .drive_d = 0,
    .drive_max = 600,
    .autodrive_telem_interval = 50000,
};

static struct DriveControlParams params;
static uint32_t last_autodrive_telem = 0;

void setDriveControlParams(int16_t p_steer_p,
                           int16_t p_steer_d,
                           int16_t p_steer_max,
                           int16_t p_gyro_gain,
                           int16_t p_drive_p,
                           int16_t p_drive_d,
                           int16_t p_drive_max
                           ) {
    params.steer_p = p_steer_p;
    params.steer_d = p_steer_d;
    params.gyro_gain = p_gyro_gain;
    params.steer_max = p_steer_max;
    params.drive_p = p_drive_p;
    params.drive_d = p_drive_d;
    params.drive_max = p_drive_max;
    saveDriveControlParameters();
}

int32_t integer_sqrt(int32_t n)
{
    int shift = 0;
    do
    {
        shift += 2;
    }
    while((n>>shift) > 0 && shift<32);
    shift -= 2;

    int32_t result=0, candidateResult;
    while(shift >= 0)
    {
        result <<= 1;
        candidateResult = result + 1;
        if((candidateResult * candidateResult) <= (n>>shift))
            result = candidateResult;
        shift -= 2;
    }
    return result;
}

bool pidSteer(const Track &tracked_object,
              int16_t depth, int16_t *drive_bias, int16_t *steer_bias) {
    int32_t now=micros();
    bool valid = tracked_object.valid(now);
    if(valid) {
        int32_t bias;
        int32_t theta = tracked_object.angle();
        int32_t vtheta = tracked_object.vtheta();
        bias  = params.steer_p * (0 - theta) / 16384L;
        bias += -params.steer_d * vtheta / 16384L;
        int16_t omegaZ = 0;
        if(getOmegaZ(&omegaZ)) {
            bias += -params.gyro_gain*omegaZ/1024;
        }
        *steer_bias  = clip(bias, -params.steer_max, params.steer_max);
        int32_t tracked_r = integer_sqrt(
            (tracked_object.x / 4L) * (tracked_object.x / 4L) +
            (tracked_object.y / 4L) * (tracked_object.y / 4L));
        int32_t tracked_vr = integer_sqrt(
            (tracked_object.vx / 4L) * (tracked_object.vx / 4L)+
            (tracked_object.vy / 4L) * (tracked_object.vy / 4L));
        bias  = params.drive_p * ((int32_t)depth*4L - tracked_r)/16384L;
        bias += -params.drive_d * tracked_vr * 4 / 16384L;
        *drive_bias  = clip(bias, -params.drive_max, params.drive_max);
        if(now - last_autodrive_telem > params.autodrive_telem_interval) {
            sendAutodriveTelemetry(*steer_bias,
                                   *drive_bias,
                                   clip(theta, -32768L, 32767L),
                                   clip(vtheta, -32768L, 32767L),
                                   clip(tracked_r / 4, -32768L, 32767L),
                                   clip(tracked_vr / 4, -32768L, 32767L));
        }
    }
    return valid;
}

void saveDriveControlParameters() {
    eeprom_write_block(&params, &saved_params, sizeof(struct DriveControlParams));
}

void restoreDriveControlParameters() {
    eeprom_read_block(&params, &saved_params, sizeof(struct DriveControlParams));
}
