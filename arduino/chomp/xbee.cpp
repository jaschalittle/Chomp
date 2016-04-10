#include "Arduino.h"
#include "xbee.h"
#include "pins.h"

static bool xbee_enabled = false;
void xbeeInit(){
  Xbee.begin(57600);
  pinMode(XBEE_CTS, INPUT);
  xbee_enabled = true;
}

#define BUFSIZE 256
char buf[BUFSIZE];
unsigned char head = 0;
unsigned char tail = 0;
unsigned int space(){
  if (head >= tail){
    return BUFSIZE - (head - tail);
  }
  return BUFSIZE - (BUFSIZE - tail + head);
}

void xbeeBufferData(char* data, unsigned int len){
//     Serial.print(head);
//     Serial.print(":");
//     Serial.print(tail);
//     Serial.print("\r\n");
   if ( len >= space()){
    return;
   }
   uint8_t firstCopyLen = min(len, BUFSIZE - head);
   memcpy(buf + head, data, firstCopyLen);
   head = (head + firstCopyLen) % BUFSIZE;
   // We only wrote up til the end of the ring
   if ( len > firstCopyLen){
     uint8_t secondCopyLen = len - firstCopyLen;
     memcpy(buf + head, data+firstCopyLen, secondCopyLen);
     head = (head + secondCopyLen) % BUFSIZE;
   }
}

void xbeePushData(){
    if (xbee_enabled){
      uint8_t len = 0;
      if (head < tail){
        len = min(BUFSIZE - tail, 16);
      } else {
        len = min(head - tail, 16);
      }
      Xbee.write(buf + tail, len);
      tail = (tail + len) % BUFSIZE;
    }
}

