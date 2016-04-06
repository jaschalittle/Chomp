#ifndef AUTOFIRE_H
#define AUTOFIRE_H

//Forward decl
struct Detection;

enum LeddarState {
  FAR_ZONE,
  ARM_ZONE,
  HIT_ZONE
};

LeddarState getState(unsigned int detection_count, Detection* detections);
LeddarState getStatePredictive(int16_t target_x_after_leadtime, int16_t target_y_after_leadtime);

#endif  // AUTOFIRE_H
