#include "Arduino.h"
#include "telem.h"
#include "xbee.h"
#include "leddar_io.h"
#include "autofire.h"
#include "pins.h"

bool cts(){
  return (digitalRead(XBEE_CTS) == LOW);
}

// I think int is same as short in avrland
void write_int(char* packet, int16_t n){
  for (uint8_t i = 0; i < 4; i++){
    packet[i] = 0xFF & n >> (i*8); 
  }
}

void write_ulong(char* packet, uint32_t n){
  for (uint8_t i = 0; i < 4; i++){
    packet[i] = 0xFF & n >> (i*8); 
  }
}

void write_short(char* packet, int16_t n){
  for (uint8_t i = 0; i < 2; i++){
    packet[i] = 0xFF & n >> (i*8); 
  }
}

void write_float(char* packet, float n){
  // get access to the float as a byte-array:
  uint8_t * data = (uint8_t *) &n;
   for (uint8_t i = 0; i < 4; i++){
    packet[i] = data[i];
  }
}

void write_terminator(char* packet){
  packet[0] = 0x55;
  packet[1] = 0x55;
}

void send_sensor_telem(uint32_t loop_speed, float pressure){
  const uint8_t LEN = 11;
  char packet[LEN];
  write_terminator(packet);
  packet[3] = 1; // packet id
  
  write_ulong(packet+3, loop_speed);
  write_int(packet+7, (int)pressure);
  if(cts()){
    Xbee.write(packet, LEN);
  }
  
}

void sendLeddarTelem(Detection* detections, unsigned int count, LeddarState state){
//  write_short(terminator);
//  char packet_id = 2;
//  Xbee.write(packet_id);
//  write_int(state);
//  uint8_t j=0;
//  for (int i = 0; i < 16; i++){
////    Xbee.print(detections[i].Segment);
////    Xbee.print("/");
////    Xbee.print(detections[i].Distance);
////    Xbee.print(" " );
//    //while(detections[j].Segment < i) { j++; }
//    if(detections[j].Segment > i){
//      write_short(0);
//    }else{
//      write_short((short)detections[j].Distance);
//      j++;
//    }
//    delay(5);
//  }
//  //Xbee.print("\r\n");
}

