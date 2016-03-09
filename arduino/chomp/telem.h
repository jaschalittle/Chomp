#ifndef TELEM_H
#define TELEM_H
#include "leddar_io.h"

void send_sensor_telem(unsigned long loop_speed, float pressure);
void send_leddar_telem(Detection* detections, unsigned int count, LeddarState state);

#endif //TELEM_H
