#ifndef AUTOFIRE_H
#define AUTOFIRE_H
#include "leddar_io.h"

enum LeddarState {
  FAR_ZONE = 1,
  ARM_ZONE,
  HIT_ZONE,
  PREDICTIVE_HIT_ZONE
};

LeddarState getState(const Detection (&detections)[LEDDAR_SEGMENTS], int16_t fire_threshold);
LeddarState getStatePredictive(int16_t target_x_after_leadtime, int16_t target_y_after_leadtime);

#endif  // AUTOFIRE_H
