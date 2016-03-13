#ifndef TELEM_H
#define TELEM_H
#include "autofire.h"

// Forward decls
struct Detection;

void send_sensor_telem(unsigned long loop_speed, float pressure);
void sendLeddarTelem(Detection* detections, unsigned int count, LeddarState state);

#endif //TELEM_H
