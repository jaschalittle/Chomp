#include "Arduino.h"
#include "leddar_wrapper.h"
#include "xbee.h"

HardwareSerial & LeddarSerial = Serial;

void leddar_wrapper_init(){
  LeddarSerial.begin(115200);
}

bool CRC16(byte *aBuffer, byte aLength, bool aCheck) 
{
  byte lCRCHi = 0xFF; // high byte of CRC initialized
  byte lCRCLo = 0xFF; // low byte of CRC initialized
  int i;
  
  for (i = 0; i<aLength; ++i) 
  {
    int lIndex = lCRCLo ^ aBuffer[i]; // calculate the CRC
    lCRCLo = lCRCHi ^ CRC_HI[lIndex];
    lCRCHi = CRC_LO[lIndex];
  }
  
  if (aCheck) 
  {
    return ( aBuffer[aLength] == lCRCLo ) && ( aBuffer[aLength+1] == lCRCHi );
  }
  else 
  {
    aBuffer[aLength] = lCRCLo;
    aBuffer[aLength+1] = lCRCHi;
    return true;
  }
}
unsigned int len = 0;
unsigned char receivedData[256] = {0};
void request_detections(){
  unsigned int i = 0;
  unsigned char sendData[4] = {0};
  //clear serial buffer
  while (Serial.available())
  {
    Serial.read();
  }
  len = 0;
  memset(receivedData, 0, 256);
  
  //send message on uart
  sendData[0] = 0x01; //SlaveAddress;
  sendData[1] = 0x41;
  CRC16(sendData, 2, false);
  for (i = 0; i<4; i++)
  {
    Serial.write(sendData[i]);
  }
  Serial.flush();
}

bool buffer_detections(){
  unsigned int crc = 0xFFFF;
  unsigned long startTime = millis();
  
  unsigned int count = Serial.available();
  if (count > 0){
    Serial.readBytes(receivedData+len, count);
    len += count;
  }
  if (len > 3){
    char detection_count = receivedData[2];
    unsigned int target_len = detection_count * 5 + 11;
    if (target_len == len){
      return true; 
    }
  }
  return false;
}

Detection Detections[50];
unsigned int parse_detections(){
  if (!CRC16(receivedData, len-2, true)){
    return 0;
  }
  unsigned int detection_count = receivedData[2];
  // Parse out detection info
  for ( unsigned int i = 0; i < detection_count; i++){
    unsigned int offset = i * 5 + 3;
    Detections[i].Distance = ((unsigned int)receivedData[offset+1])*256 + receivedData[offset];
    Detections[i].Amplitude = ((float)receivedData[offset+3])*4+ ((float)receivedData[offset+2])/64;
    Detections[i].Segment = receivedData[offset+4]/16;
//    Xbee.print(Detections[i].Segment);
//    Xbee.print("/");
//    Xbee.print(Detections[i].Distance);
//    Xbee.print(" ");
  }
//  Xbee.print("\r\n");

  return detection_count;
}

Detection* get_detections(){
  return Detections;
}

#define CENTER_ZONE_MIN 4
#define CENTER_ZONE_MAX 12

#define FACE_OFFSET 20
#define ARM_THRESHOLD 150
#define FIRE_THRESHOLD 90
#define CONTIG_THRESHOLD 2
LeddarState get_state(unsigned int detections){

  int last_detected_segment = 0;
  int contiguous = 1;
  for (int i = 0; i < detections; i++){
    if (Detections[i].Segment < CENTER_ZONE_MAX && Detections[i].Segment > CENTER_ZONE_MIN &&
        Detections[i].Distance - FACE_OFFSET < FIRE_THRESHOLD){
      //Xbee.write("Hit!\r\n");
      if (Detections[i].Segment - last_detected_segment == 1){
        contiguous += 1;
      } else {
        contiguous = 1;
      }
      last_detected_segment = Detections[i].Segment;
    }
  }
  
  if (contiguous >= CONTIG_THRESHOLD){
    return HIT_ZONE;
  }
  
  last_detected_segment = 0;
  contiguous = 1;
  for (int i = 0; i < detections; i++){
    if (Detections[i].Segment < CENTER_ZONE_MAX && Detections[i].Segment > CENTER_ZONE_MIN &&
        Detections[i].Distance - FACE_OFFSET < ARM_THRESHOLD){
      //Xbee.write("Hit!\r\n");
      if (Detections[i].Segment - last_detected_segment == 1){
        contiguous += 1;
      } else {
        contiguous = 1;
      }
      last_detected_segment = Detections[i].Segment;
    }
  }

  if (contiguous >= CONTIG_THRESHOLD){
    return ARM_ZONE;
  }
  //Xbee.write("Nothing to see here\r\n");
  return FAR_ZONE;
}

