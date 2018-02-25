#ifndef TELEM_H
#define TELEM_H
#include "autofire.h"

// Forward decls
struct Detection;

bool sendSystemTelem(uint32_t loop_speed, uint16_t leddar_overrun,
                     uint16_t leddar_crc_error, uint16_t sbus_overrun);
bool sendSensorTelem(int16_t pressure, uint16_t angle);
bool sendSbusTelem(uint16_t cmd_bitfield);
bool sendDebugMessageTelem(char *msg);
void debug_print(const String &msg);
bool sendLeddarTelem(Detection* detections, unsigned int count, LeddarState state);
bool sendSwingTelem(uint16_t datapoints_collected,
                    uint16_t* angle_data,
                    int16_t* pressure_data,
                    uint16_t data_collect_timestep,
                    uint16_t throw_close_timestep,
                    uint16_t vent_open_timestep,
                    uint16_t throw_close_angle,
                    uint16_t start_angle);
bool sendPWMTelem(int16_t left_drive, int16_t right_drive);

#endif //TELEM_H
