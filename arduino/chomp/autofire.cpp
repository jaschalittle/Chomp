#include <Arduino.h>
#include "autofire.h"
#include "imu.h"
#include "telem.h"

extern uint8_t HAMMER_INTENSITIES_ANGLE[9];

static int32_t xtol = 200, ytol=200, ttol=5;
static int32_t max_omegaZ = 50;

int32_t swingDuration(int16_t hammer_intensity) {
    int16_t hammer_angle = HAMMER_INTENSITIES_ANGLE[hammer_intensity];
    int32_t x=(40L-hammer_angle);
    return 230L + (3L*x*x*x)/1024L;
}

bool omegaZLockout(int16_t *omegaZ) {
    *omegaZ = 0;
    bool imu_valid = getOmegaZ(omegaZ);
    return imu_valid && *omegaZ>max_omegaZ;
}

int8_t nsteps=3;
enum AutofireState willHit(const Track &tracked_object,
                           int16_t depth, int16_t hammer_intensity) {
    int16_t rawOmegaZ=0;
    bool hit = false;
    bool lockout = omegaZLockout(&rawOmegaZ);
    bool valid = tracked_object.valid(micros());
    if(valid && !lockout) {
        int32_t swing=swingDuration(hammer_intensity)*1000;
        int32_t x=tracked_object.x, y=tracked_object.y;
        int32_t dt=swing/nsteps;
        int32_t omegaZ = (int32_t)rawOmegaZ*35L/16;
        for(int s=0;s<nsteps;s++) {
            tracked_object.project(dt, dt*omegaZ, &x, &y);
        }
        hit = abs(x/16-depth)<xtol && abs(y/16)<ytol;
    }
    enum AutofireState st;
    if(lockout) st =     AF_OMEGAZ_LOCKOUT;
    else if(!valid) st = AF_NO_TARGET;
    else if(!hit) st =   AF_NO_HIT;
    else st =            AF_HIT;
    return st;
}

void setAutoFireParams(int16_t p_xtol,
                       int16_t p_ytol,
                       int16_t p_ttol){
    xtol = p_xtol;
    ytol = p_ytol;
    ttol = p_ttol;
}
