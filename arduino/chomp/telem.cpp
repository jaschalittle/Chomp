#include "Arduino.h"
#include "telem.h"
#include "xbee.h"
#include "leddar_io.h"
#include "autofire.h"
#include "pins.h"

bool sendHealthSensorTelem(uint32_t loop_speed, uint16_t cmd_bitfield, int16_t pressure, uint16_t angle, uint16_t leddar_overrun, uint16_t sbus_overrun){
  const uint16_t packet_len = 17;
  const uint8_t start = 0x01;
  const uint16_t ending = 0x6666;
  char sensor_data[packet_len] = {0};
  uint8_t offset = 0;
  memcpy(sensor_data, &start, sizeof(uint8_t));
  offset += sizeof(uint8_t);
  memcpy(sensor_data + offset, &loop_speed, sizeof(uint32_t));
  offset += sizeof(uint32_t);
  memcpy(sensor_data + offset, &cmd_bitfield, sizeof(uint16_t));
  offset += sizeof(uint16_t);
  memcpy(sensor_data + offset, &pressure, sizeof(int16_t));
  offset += sizeof(int16_t);
  memcpy(sensor_data + offset, &angle, sizeof(uint16_t));
  offset += sizeof(uint16_t);
  memcpy(sensor_data + offset, &leddar_overrun, sizeof(uint16_t));
  offset += sizeof(uint16_t);
  memcpy(sensor_data + offset, &sbus_overrun, sizeof(uint16_t));
  offset += sizeof(uint16_t);
  memcpy(sensor_data + offset, &ending, sizeof(uint16_t));
  bool result = xbeeBufferData(sensor_data, packet_len);
  return result;
}

bool sendLeddarTelem(Detection* detections, unsigned int count, LeddarState state){
  Detection min_detections[16];
  getMinDetections(count, detections, min_detections);

  const uint16_t packet_len = 45;
  const uint8_t start = 0x02;
  const uint16_t ending = 0x6666;
  char sensor_data[packet_len] = {0};
  uint8_t offset = 0;
  memcpy(sensor_data, &start, sizeof(uint8_t));
  offset += sizeof(uint8_t);
  memcpy(sensor_data + offset, &state, sizeof(int16_t));
  offset += sizeof(int16_t);
  const uint64_t segments = 0xEFCDAB8967452301;
  memcpy(sensor_data + offset, &segments, sizeof(uint64_t));
  offset += sizeof(uint64_t);
  for (uint16_t i = 0; i < 16; i++){
      memcpy(sensor_data + offset, &min_detections[i].Distance, sizeof(uint16_t));
      offset += sizeof(uint16_t);
  }
  memcpy(sensor_data + offset, &ending, sizeof(uint16_t));
  bool result = xbeeBufferData(sensor_data, packet_len);
  return result;
}

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
