#include "Arduino.h"
#include "telem.h"
#include "xbee.h"
#include "leddar_io.h"
#include "autofire.h"
#include "pins.h"

enum TelemetryPacketId {
    TLM_ID_HS=1,
    TLM_ID_LEDDAR=2,
    TLM_ID_SNS=10,
    TLM_ID_SYS=11,
    TLM_ID_SBS=12,
    TLM_ID_DBGM=13,
    TLM_ID_SWG=14
};

const uint16_t TLM_TERMINATOR=0x6666;

template <uint8_t packet_id, typename packet_inner> struct TelemetryPacket{
    uint8_t pkt_id;
    packet_inner inner;
    uint16_t terminator;
    TelemetryPacket() : pkt_id(packet_id), terminator(TLM_TERMINATOR) {};
} __attribute__((packed));

struct SystemTelemetryInner {
    uint32_t loop_speed;
    uint16_t leddar_overrun;
    uint16_t leddar_crc_error;
    uint16_t sbus_overrun;
} __attribute__((packed));
typedef TelemetryPacket<TLM_ID_SYS, SystemTelemetryInner> SystemTelemetry;

bool sendSystemTelem(uint32_t loop_speed, uint16_t leddar_overrun,
                     uint16_t leddar_crc_error, uint16_t sbus_overrun){
    SystemTelemetry tlm;
    tlm.inner.loop_speed = loop_speed;
    tlm.inner.leddar_overrun = leddar_overrun;
    tlm.inner.leddar_crc_error = leddar_crc_error;
    tlm.inner.sbus_overrun = sbus_overrun;
    return Xbee.write((unsigned char *)&tlm, sizeof(tlm));
}


struct SensorTelemetryInner {
    uint16_t pressure;
    uint16_t angle;
} __attribute__((packed));
typedef TelemetryPacket<TLM_ID_SNS, SensorTelemetryInner> SensorTelemetry;

bool sendSensorTelem(int16_t pressure, uint16_t angle){
    SensorTelemetry tlm;
    tlm.inner.pressure = pressure;
    tlm.inner.angle = angle;
    return Xbee.write((unsigned char *)&tlm, sizeof(tlm));
}


struct SBusTelemetryInner {
    uint16_t bitfield;
} __attribute__((packed));
typedef TelemetryPacket<TLM_ID_SBS, SBusTelemetryInner> SBusTelemetry;

bool sendSbusTelem(uint16_t cmd_bitfield){
    SBusTelemetry tlm;
    tlm.inner.bitfield = cmd_bitfield;
    return Xbee.write((unsigned char *)&tlm, sizeof(tlm));
}

const size_t MAX_DEBUG_MSG_LENGTH=128;
bool sendDebugMessageTelem(const char *msg){
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
#ifdef HARD_WIRED
    sendDebugMessageTelem(msg.c_str());
#endif
}

struct LeddarTelemetryInner {
    uint16_t state;
    uint16_t count;
    const uint64_t segments = 0xEFCDAB8967452301;
    uint16_t beam[16];
} __attribute__((packed));
typedef TelemetryPacket<TLM_ID_LEDDAR, LeddarTelemetryInner> LeddarTelemetry;

bool sendLeddarTelem(Detection* detections, unsigned int count, LeddarState state){
  Detection min_detections[16];
  getMinDetections(count, detections, min_detections);

  LeddarTelemetry tlm;

  tlm.inner.state = state;
  tlm.inner.count = count;
  for (uint8_t i = 0; i < 16; i++){
      tlm.inner.beam[i] = min_detections[i].Distance;
  }
  return Xbee.write((unsigned char *)&tlm, sizeof(tlm));
}

/*
extern DMASerial& Debug;

void sendSwingData(uint16_t datapoints_collected, 
                   uint16_t* angle_data, 
                   int16_t* pressure_data, 
                   uint16_t data_collect_timestep, 
                   uint16_t throw_close_timestep, 
                   uint16_t vent_open_timestep, 
                   uint16_t throw_close_angle, 
                   uint16_t start_angle) {
    for (uint16_t i = 0; i < datapoints_collected; i++) {
        String timestep_data = String("");
        timestep_data += "d\t";
        timestep_data += angle_data[i];
        timestep_data += "\t";
        timestep_data += pressure_data[i];
        Debug.println(timestep_data);
        delay(2);
    }

    Debug.print("timestep\t");
    delay(50);
    Debug.println(data_collect_timestep);
    delay(50);
    Debug.print("tc_tstep\t");
    delay(50);
    Debug.println(throw_close_timestep);
    delay(50);
    Debug.print("vo_tstep\t");
    delay(50);
    Debug.println(vent_open_timestep);
    delay(50);
    Debug.print("tc_angle\t");
    delay(50);
    Debug.println(throw_close_angle);
    delay(50);
    Debug.print("st_angle\t");
    delay(50);
    Debug.println(start_angle);
    delay(50);
}
*/

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
        success &= Xbee.enqueue((uint8_t *)angle_data, sizeof(uint16_t)*125, NULL, NULL);
        success &= Xbee.enqueue((uint8_t *)pressure_data, sizeof(uint16_t)*125, NULL, NULL);
    }
    success &= Xbee.write((uint8_t *)&tlm.terminator, sizeof(tlm.terminator));

    return success;
}
