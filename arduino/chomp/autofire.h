#pragma once
#include <stdint.h>
#include "track.h"

bool timeToHit(const Track &tracked_object, int32_t *dt, int16_t depth, int16_t omegaZ);

void setAutoFireParams(int16_t p_xtol, int16_t p_ytol);
