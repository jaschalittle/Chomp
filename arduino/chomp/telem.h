#ifndef TELEM_H
#define TELEM_H
#include "autofire.h"

// Forward decls
struct Detection;

bool sendHealthSensorTelem(uint32_t loop_speed, uint16_t cmd_bitfield, int16_t pressure, uint16_t angle);
bool sendLeddarTelem(Detection* detections, unsigned int count, LeddarState state);

#endif //TELEM_H
