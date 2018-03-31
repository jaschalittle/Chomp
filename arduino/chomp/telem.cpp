#include "Arduino.h"
#include "telem.h"
#include "xbee.h"
#include "leddar_io.h"
#include "pins.h"
#include "DMASerial.h"

extern DMASerial& Xbee;

const uint16_t TLM_TERMINATOR=0x6666;

#define CHECK_ENABLED(TLM_ID) if(!(enabled_telemetry & _LBV(TLM_ID))) return false;

uint32_t enabled_telemetry=(
    _LBV(TLM_ID_SYS)|
    _LBV(TLM_ID_SBS)|
    _LBV(TLM_ID_PWM)|
    _LBV(TLM_ID_DRV)|
    _LBV(TLM_ID_DBGM));


template <uint8_t packet_id, typename packet_inner> struct TelemetryPacket{
    uint8_t pkt_id;
    packet_inner inner;
    uint16_t terminator;
    TelemetryPacket() : pkt_id(packet_id), terminator(TLM_TERMINATOR) {};
} __attribute__((packed));


struct SystemTelemetryInner {
    uint8_t  weapons_enabled:1;
    uint32_t loop_speed_min;
    uint32_t loop_speed_avg;
    uint32_t loop_speed_max;
    uint32_t loop_count;
    uint16_t leddar_overrun;
    uint16_t leddar_crc_error;
    uint16_t sbus_overrun;
    uint8_t last_command;
    uint16_t command_overrun;
    uint16_t invalid_command;
    uint16_t valid_command;
} __attribute__((packed));
typedef TelemetryPacket<TLM_ID_SYS, SystemTelemetryInner> SystemTelemetry;

bool sendSystemTelem(uint32_t loop_speed_min, uint32_t loop_speed_avg,
                     uint32_t loop_speed_max, uint32_t loop_count,
                     uint16_t leddar_overrun, uint16_t leddar_crc_error,
                     uint16_t sbus_overrun, uint8_t last_command,
                     uint16_t command_overrun, uint16_t invalid_command,
                     uint16_t valid_command){
    CHECK_ENABLED(TLM_ID_SYS);
    SystemTelemetry tlm;
    tlm.inner.weapons_enabled = g_enabled;
    tlm.inner.loop_speed_min = loop_speed_min;
    tlm.inner.loop_speed_avg = loop_speed_avg;
    tlm.inner.loop_speed_max = loop_speed_max;
    tlm.inner.loop_count = loop_count;
    tlm.inner.leddar_overrun = leddar_overrun;
    tlm.inner.leddar_crc_error = leddar_crc_error;
    tlm.inner.sbus_overrun = sbus_overrun;
    tlm.inner.last_command = last_command;
    tlm.inner.command_overrun = command_overrun;
    tlm.inner.invalid_command = invalid_command;
    tlm.inner.valid_command = valid_command;
    return Xbee.write((unsigned char *)&tlm, sizeof(tlm));
}


struct SensorTelemetryInner {
    uint16_t pressure;
    uint16_t angle;
} __attribute__((packed));
typedef TelemetryPacket<TLM_ID_SNS, SensorTelemetryInner> SensorTelemetry;

bool sendSensorTelem(int16_t pressure, uint16_t angle){
    CHECK_ENABLED(TLM_ID_SNS);
    SensorTelemetry tlm;
    tlm.inner.pressure = pressure;
    tlm.inner.angle = angle;
    return Xbee.write((unsigned char *)&tlm, sizeof(tlm));
}


struct SBusTelemetryInner {
    uint16_t bitfield;
    int16_t hammer_intensity;
    int16_t hammer_distance;
} __attribute__((packed));
typedef TelemetryPacket<TLM_ID_SBS, SBusTelemetryInner> SBusTelemetry;

bool sendSbusTelem(uint16_t cmd_bitfield, int16_t hammer_intensity, int16_t hammer_distance) {
    CHECK_ENABLED(TLM_ID_SBS);
    SBusTelemetry tlm;
    tlm.inner.bitfield = cmd_bitfield;
    tlm.inner.hammer_intensity = hammer_intensity;
    tlm.inner.hammer_distance = hammer_distance;
    return Xbee.write((unsigned char *)&tlm, sizeof(tlm));
}


const size_t MAX_DEBUG_MSG_LENGTH=128;
bool sendDebugMessageTelem(const char *msg){
    CHECK_ENABLED(TLM_ID_DBGM);
    unsigned char pkt[1+MAX_DEBUG_MSG_LENGTH+2]={0};
    pkt[0] = TLM_ID_DBGM;
    size_t copied = 0;
    size_t pos=1;
    while(copied<MAX_DEBUG_MSG_LENGTH && msg[copied]) {
        pkt[pos++] = msg[copied++];
    }
    *((uint16_t *)(pkt+pos)) = TLM_TERMINATOR;
    size_t sendlen = 1+copied+sizeof(TLM_TERMINATOR);
    return Xbee.write(pkt, sendlen);
}

void debug_print(const String &msg){
    sendDebugMessageTelem(msg.c_str());
}


struct LeddarTelemetryInner {
    uint16_t state;
    uint16_t count;
    uint16_t range[LEDDAR_SEGMENTS];
    uint16_t amplitude[LEDDAR_SEGMENTS];
} __attribute__((packed));
typedef TelemetryPacket<TLM_ID_LEDDARV2, LeddarTelemetryInner> LeddarTelemetry;

static LeddarTelemetry leddar_tlm;
bool sendLeddarTelem(const Detection (&min_detections)[LEDDAR_SEGMENTS], unsigned int count){
  CHECK_ENABLED(TLM_ID_LEDDARV2);
  leddar_tlm.inner.count = count;
  for (uint8_t i = 0; i < LEDDAR_SEGMENTS; i++){
      leddar_tlm.inner.range[i] = min_detections[i].Distance;
      leddar_tlm.inner.amplitude[i] = min_detections[i].Amplitude;
  }
  return Xbee.enqueue((unsigned char *)&leddar_tlm, sizeof(leddar_tlm),
                      NULL, NULL);
}


struct SwingTelemInner {
    uint16_t sample_period;
    uint16_t throw_close;
    uint16_t vent_open;
    uint16_t throw_close_angle;
    uint16_t start_angle;
    uint16_t datapoints;
} __attribute__((packed));
typedef TelemetryPacket<TLM_ID_SWG, SwingTelemInner> SwingTelemetry;

bool sendSwingTelem(uint16_t datapoints_collected,
                    uint16_t* angle_data,
                    int16_t* pressure_data,
                    uint16_t data_collect_timestep,
                    uint16_t throw_close_timestep,
                    uint16_t vent_open_timestep,
                    uint16_t throw_close_angle,
                    uint16_t start_angle) {
    CHECK_ENABLED(TLM_ID_SWG);
    SwingTelemetry tlm;
    tlm.inner.sample_period = data_collect_timestep;
    tlm.inner.throw_close = throw_close_timestep;
    tlm.inner.vent_open = vent_open_timestep;
    tlm.inner.throw_close_angle = throw_close_angle;
    tlm.inner.start_angle = start_angle;
    tlm.inner.datapoints = datapoints_collected;
    bool success = Xbee.write((unsigned char *)&tlm, sizeof(tlm)-sizeof(TLM_TERMINATOR));
    if(success)
    {
        success &= Xbee.enqueue((uint8_t *)angle_data, sizeof(uint16_t)*256, NULL, NULL);
        success &= Xbee.enqueue((uint8_t *)pressure_data, sizeof(int16_t)*256, NULL, NULL);
    }
    success &= Xbee.write((uint8_t *)&tlm.terminator, sizeof(tlm.terminator));

    return success;
}


struct PWMTelemInner {
    uint8_t pad:7;
    uint8_t targeting_enable:1;
    int16_t left_drive;
    int16_t right_drive;
    int16_t drive_distance;
} __attribute__((packed));
typedef TelemetryPacket<TLM_ID_PWM, PWMTelemInner> PWMTelemetry;

bool sendPWMTelem(bool targeting_enable, int16_t left_drive, int16_t right_drive,
                  int16_t drive_distance) {
    CHECK_ENABLED(TLM_ID_PWM);
    PWMTelemetry tlm;
    tlm.inner.targeting_enable = targeting_enable;
    tlm.inner.left_drive = left_drive;
    tlm.inner.right_drive = right_drive;
    tlm.inner.drive_distance = drive_distance;
    return Xbee.write((unsigned char *)&tlm, sizeof(tlm));
}


struct IMUTelemInner {
    int16_t a[3];
    int16_t g[3];
    int16_t t;
} __attribute__((packed));
typedef TelemetryPacket<TLM_ID_IMU, IMUTelemInner> IMUTelemetry;

bool sendIMUTelem(int16_t (&a)[3], int16_t (&g)[3], int16_t t)
{
    CHECK_ENABLED(TLM_ID_IMU);
    IMUTelemetry tlm;
    for(size_t i=0;i<3;i++) {
        tlm.inner.a[i] = a[i];
        tlm.inner.g[i] = g[i];
    }
    tlm.inner.t = t;
    return Xbee.write((unsigned char *)&tlm, sizeof(tlm));
}


struct DMPTelemInner {
    uint16_t fifoCount;
    uint8_t intStatus;
    float qw, qx, qy, qz;
} __attribute__((packed));
typedef TelemetryPacket<TLM_ID_DMP, DMPTelemInner> DMPTelemetry;

bool sendDMPTelem(size_t fifoCount, uint8_t intStatus, float w, float x, float y, float z)
{
    CHECK_ENABLED(TLM_ID_DMP);
    DMPTelemetry tlm;
    tlm.inner.fifoCount = fifoCount;
    tlm.inner.intStatus = intStatus;
    tlm.inner.qw = w;
    tlm.inner.qx = x;
    tlm.inner.qy = y;
    tlm.inner.qz = z;
    return Xbee.write((unsigned char *)&tlm, sizeof(tlm));
}


struct ORNTelemInner {
    uint8_t padding:3;
    uint8_t orientation:4;
    uint8_t stationary:1;
    int32_t best_accum;
    int32_t sum_angular_rate;
} __attribute__((packed));
typedef TelemetryPacket<TLM_ID_ORN, ORNTelemInner> ORNTelemetry;
bool sendORNTelem(bool stationary, uint8_t orientation, int32_t best_accum, int32_t sum_angular_rate)
{
    CHECK_ENABLED(TLM_ID_ORN);
    ORNTelemetry tlm;
    tlm.inner.stationary = stationary;
    tlm.inner.orientation = orientation;
    tlm.inner.best_accum = best_accum;
    tlm.inner.sum_angular_rate = sum_angular_rate;
    return Xbee.write((unsigned char *)&tlm, sizeof(tlm));
}


struct SelfRightTelemInner {
    uint8_t state;
} __attribute__((packed));
typedef TelemetryPacket<TLM_ID_SRT, SelfRightTelemInner> SRTTelemetry;
bool sendSelfRightTelem(uint8_t state) {
    CHECK_ENABLED(TLM_ID_SRT);
    SRTTelemetry tlm;
    tlm.inner.state = state;
    return Xbee.write((unsigned char *)&tlm, sizeof(tlm));
}

struct DriveTelemInner {
    int16_t wheel_voltages[4];
    int16_t weapons_voltage;
} __attribute__((packed));
typedef TelemetryPacket<TLM_ID_DRV, DriveTelemInner> DRVTelemetry;
bool sendDriveTelem(int16_t const (&vwheel)[4], int16_t vweapon) {
    CHECK_ENABLED(TLM_ID_DRV);
    DRVTelemetry tlm;
    memcpy(tlm.inner.wheel_voltages, vwheel, sizeof(vwheel));
    tlm.inner.weapons_voltage = vweapon;
    return Xbee.write((unsigned char *)&tlm, sizeof(tlm));
}

struct TrackingTelemetryInner {
    int16_t detection_x;
    int16_t detection_y;
    int32_t filtered_x;
    int32_t filtered_vx;
    int32_t filtered_y;
    int32_t filtered_vy;
    int32_t rx;
    int32_t ry;
    int32_t best_distance;
} __attribute__((packed));
typedef TelemetryPacket<TLM_ID_TRK, TrackingTelemetryInner> TRKTelemetry;
bool sendTrackingTelemetry(int16_t detection_x,
                           int16_t detection_y,
                           int32_t filtered_x,
                           int32_t filtered_vx,
                           int32_t filtered_y,
                           int32_t filtered_vy,
                           int32_t rx,
                           int32_t ry,
                           int32_t best_distance) {
    CHECK_ENABLED(TLM_ID_TRK);
    TRKTelemetry tlm;
    tlm.inner.detection_x = detection_x;
    tlm.inner.detection_y = detection_y;
    tlm.inner.filtered_x = filtered_x;
    tlm.inner.filtered_vx = filtered_vx;
    tlm.inner.filtered_y = filtered_y;
    tlm.inner.filtered_vy = filtered_vy;
    tlm.inner.rx = rx;
    tlm.inner.ry = ry;
    tlm.inner.best_distance = best_distance;
    return Xbee.write((unsigned char *)&tlm, sizeof(tlm));
}
