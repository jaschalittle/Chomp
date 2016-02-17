#ifndef LEDDAR_WRAPPER_H
#define LEDDAR_WRAPPER_H

enum LeddarState {
  FAR_ZONE,
  ARM_ZONE,
  HIT_ZONE
};

void leddar_wrapper_init();
int poll_leddar();

void request_detections();
bool buffer_detections();
void process_detections();

#endif  // LEDDAR_WRAPPER_H
