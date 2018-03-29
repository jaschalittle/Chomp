#ifndef TELEM_H
#define TELEM_H
#include "autofire.h"

enum TelemetryPacketId {
    TLM_ID_HS=1,
    TLM_ID_LEDDAR=2,
    TLM_DBGM_AUTOFIRE=3,
    TLM_ID_TRK=4,
    TLM_ID_DRV=8,
    TLM_ID_SNS=10,
    TLM_ID_SYS=11,
    TLM_ID_SBS=12,
    TLM_ID_DBGM=13,
    TLM_ID_SWG=14,
    TLM_ID_LEDDARV2=15,
    TLM_ID_PWM=16,
    TLM_ID_IMU=17,
    TLM_ID_DMP=18,
    TLM_ID_ORN=19,
    TLM_ID_SRT=20,
};

extern uint32_t enabled_telemetry;

#define _LBV(bit) (1L << (bit))

#define IS_TLM_ENABLED(TLM_ID) (enabled_telemetry & _LBV(TLM_ID))

// Forward decls
struct Detection;

bool sendSystemTelem(uint32_t loop_speed_min, uint32_t loop_speed_avg,
                     uint32_t loop_speed_max, uint32_t loop_count,
                     uint16_t leddar_overrun, uint16_t leddar_crc_error,
                     uint16_t sbus_overrun, uint8_t last_command,
                     uint16_t command_overrun, uint16_t invalid_command);
bool sendSensorTelem(int16_t pressure, uint16_t angle);
bool sendSbusTelem(uint16_t cmd_bitfield);
bool sendDebugMessageTelem(const char *msg);
void debug_print(const String &msg);
bool sendLeddarTelem(const Detection (&detections)[LEDDAR_SEGMENTS], unsigned int count, LeddarState state);
bool sendSwingTelem(uint16_t datapoints_collected,
                    uint16_t* angle_data,
                    int16_t* pressure_data,
                    uint16_t data_collect_timestep,
                    uint16_t throw_close_timestep,
                    uint16_t vent_open_timestep,
                    uint16_t throw_close_angle,
                    uint16_t start_angle);
bool sendPWMTelem(bool targeting_enable, int16_t left_drive, int16_t right_drive);
bool sendIMUTelem(int16_t (&a)[3], int16_t (&g)[3], int16_t temperature);
bool sendORNTelem(bool stationary, uint8_t orientation, int32_t best_accum, int32_t sum_angular_rate);
bool sendDMPTelem(size_t fifoCount, uint8_t intStatus, float w, float x, float y, float z);
bool sendSelfRightTelem(uint8_t state);
bool sendDriveTelem(int16_t const (&vwheel)[4], int16_t vweapon);
bool sendTrackingTelemetry(int16_t detection_x,
                           int16_t detection_y,
                           int32_t filtered_x,
                           int32_t filtered_vx,
                           int32_t filtered_y,
                           int32_t filtered_vy,
                           int32_t rx,
                           int32_t ry);
#endif //TELEM_H
