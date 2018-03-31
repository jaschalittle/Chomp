#pragma once
#include <stdint.h>
#include "track.h"

bool willHit(const Track &tracked_object,
             int16_t depth, int16_t hammer_intensity);

bool omegaZLockout(int16_t *omegaZ);

void setAutoFireParams(int16_t p_xtol, int16_t p_ytol, int16_t p_ttol);
