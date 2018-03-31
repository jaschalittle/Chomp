#ifndef CHUMP_TARGETING_H
#define CHUMP_TARGETING_H

#include <stdint.h>
#include "leddar_io.h"
#include "track.h"

void trackObject(const Detection (&min_detections)[LEDDAR_SEGMENTS],
                 struct Track& tracked_object);

void setObjectSegmentationParams(int16_t p_min_object_size,
                                 int16_t p_max_object_size,
                                 int16_t p_edge_call_threshold);

#endif  // CHUMP_TARGETING_H
