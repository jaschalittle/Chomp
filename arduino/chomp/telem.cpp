#include "Arduino.h"
#include "telem.h"
#include "xbee.h"
#include "leddar_io.h"
#include "autofire.h"
#include "pins.h"

void send_sensor_telem(uint32_t loop_speed, int16_t pressure, uint16_t angle){
  const uint16_t packet_len = 11;
  const uint8_t start = 0x01;
  const uint16_t ending = 0x6666;
  char sensor_data[packet_len] = {0};
  uint8_t offset = 0;
  memcpy(sensor_data, &start, sizeof(uint8_t));
  offset += sizeof(uint8_t);
  memcpy(sensor_data + offset, &loop_speed, sizeof(uint32_t));
  offset += sizeof(uint32_t);
  memcpy(sensor_data + offset, &pressure, sizeof(int16_t));
  offset += sizeof(int16_t);
  memcpy(sensor_data + offset, &angle, sizeof(uint16_t));
  offset += sizeof(uint16_t);
  memcpy(sensor_data + offset, &ending, sizeof(uint16_t));
  xbeeBufferData(sensor_data, packet_len);
}

void sendLeddarTelem(Detection* detections, unsigned int count, LeddarState state){
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
  uint16_t j = 0;
  for (uint16_t i = 0; i < 16; i++){
      uint16_t dist = 1000;
      if (detections[j].Segment == i){
        dist = detections[j].Distance;
        while (j < count && detections[j].Segment == i){j++;}
      }
      memcpy(sensor_data + offset, &dist, sizeof(uint16_t));
      offset += sizeof(uint16_t);
  }
  memcpy(sensor_data + offset, &ending, sizeof(uint16_t));
  xbeeBufferData(sensor_data, packet_len);
}

