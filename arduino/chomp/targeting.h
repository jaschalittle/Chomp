#ifndef CHUMP_TARGETING_H
#define CHUMP_TARGETING_H

#include <stdint.h>
#include "leddar_io.h"

void trackObject(const Detection (&min_detections)[LEDDAR_SEGMENTS]);

void setTrackingFilterParams(int16_t alpha, int16_t beta,
                             int16_t p_min_object_size,
                             int16_t p_max_object_size,
                             int16_t p_edge_call_threshold,
                             int8_t p_min_num_updates,
                             uint32_t p_track_lost_dt,
                             int16_t p_max_off_track,
                             int16_t p_max_start_distance,
                             int16_t xtol,
                             int16_t ytol,
                             int16_t p_steer_p,
                             int16_t p_drive_p
        );

bool timeToHit(int32_t *dt, int16_t depth, int16_t omegaZ);

bool pidSteer(int16_t depth, int16_t *drive_bias, int16_t *steer_bias);

#endif  // CHUMP_TARGETING_H
