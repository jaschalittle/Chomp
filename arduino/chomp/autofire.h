#ifndef AUTOFIRE_H
#define AUTOFIRE_H

//Forward decl
struct Detection;

enum LeddarState {
  FAR_ZONE = 1,
  ARM_ZONE,
  HIT_ZONE,
  PREDICTIVE_HIT_ZONE
};

LeddarState getState(unsigned int detection_count, Detection* detections, int16_t fire_threshold);
LeddarState getStatePredictive(int16_t target_x_after_leadtime, int16_t target_y_after_leadtime);

#endif  // AUTOFIRE_H
