#include "Arduino.h"
#include "telem.h"
#include "xbee.h"
#include "leddar_wrapper.h"

void write_int(int n){
  // Big endian
  for (int i = 3; i >= 0; i--){
    Xbee.write(0xFF & n >> (i*8)); 
  }
}

void write_ulong(unsigned long n){
  // Big endian
  for (int i = 3; i >= 0; i--){
    Xbee.write(0xFF & n >> (i*8)); 
  }
}

void write_short(short n){
  // Big endian
  for (int i = 1; i >= 0; i--){
    Xbee.write(0xFF & n >> (i*8)); 
  }
}

void write_float(float n){
  // get access to the float as a byte-array:
  byte * data = (byte *) &n;
  // Big endian
  for (int i = 3; i >= 0; i--){
    Xbee.write(data[i]); 
  }
}

short terminator = 0x5555;
void send_sensor_telem(unsigned long loop_speed, float pressure){
  write_short(terminator);
  char packet_id = 1;
  Xbee.write(packet_id);
  write_ulong(loop_speed);
  write_int((int)pressure);
}

void send_leddar_telem(Detection* detections, unsigned int count, LeddarState state){
  write_short(terminator);
  char packet_id = 2;
  Xbee.write(packet_id);
  write_int(state);
  int j=0;
  for (int i = 0; i < 16; i++){
    if(detections[j].Segment > i){
      write_int(-1);
    }else{
      write_int(detections[j].Distance);
      j++;
    }
  }
}
