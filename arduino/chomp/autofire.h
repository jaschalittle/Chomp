#ifndef AUTOFIRE_H
#define AUTOFIRE_H

//Forward decl
struct Detection;

enum LeddarState {
  FAR_ZONE,
  ARM_ZONE,
  HIT_ZONE
};

LeddarState get_state(unsigned int detection_count, Detection* detections);

#endif  // AUTOFIRE_H
