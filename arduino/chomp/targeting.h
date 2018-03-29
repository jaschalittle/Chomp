#ifndef CHUMP_TARGETING_H
#define CHUMP_TARGETING_H

#include <stdint.h>
#include "leddar_io.h"

void trackObject(const Detection (&min_detections)[LEDDAR_SEGMENTS], int16_t distance_threshold);

void setTrackingFilterParams(int16_t alpha, int16_t beta,
                             int16_t p_min_object_size,
                             int16_t p_max_object_size,
                             int16_t p_edge_call_threshold,
                             int8_t p_min_num_updates,
                             uint32_t p_track_lost_dt,
                             int16_t p_max_off_track,
                             int16_t p_max_start_distance
        );

void pidSteer(const Detection (&detections)[LEDDAR_SEGMENTS], uint16_t threshold, int16_t* steer_bias, bool reset_targeting);

#endif  // CHUMP_TARGETING_H
