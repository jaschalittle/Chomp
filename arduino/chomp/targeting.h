#ifndef CHUMP_TARGETING_H
#define CHUMP_TARGETING_H

#include <stdint.h>
#include "leddar_io.h"

void trackObject(const Detection (&min_detections)[LEDDAR_SEGMENTS], int16_t distance_threshold);

void setTrackingFilterParams(int16_t alpha, int16_t beta);

// angle deltas over 5 times / total time = avg angular velocity per unit time

void pidSteer(const Detection (&detections)[LEDDAR_SEGMENTS], uint16_t threshold, int16_t* steer_bias, bool reset_targeting);

#endif  // CHUMP_TARGETING_H
