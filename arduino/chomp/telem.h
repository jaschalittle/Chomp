#ifndef TELEM_H
#define TELEM_H
#include "autofire.h"

// Forward decls
struct Detection;

bool sendHealthSensorTelem(uint32_t loop_speed, uint16_t cmd_bitfield, int16_t pressure, uint16_t angle, uint16_t leddar_overrun, uint16_t sbus_overrun);
bool sendLeddarTelem(Detection* detections, unsigned int count, LeddarState state);
void sendSwingData(uint16_t datapoints_collected, 
                   uint16_t* angle_data, 
                   int16_t* pressure_data, 
                   uint16_t data_collect_timestep, 
                   uint16_t throw_close_timestep, 
                   uint16_t vent_open_timestep, 
                   uint16_t throw_close_angle, 
                   uint16_t start_angle);

#endif //TELEM_H
