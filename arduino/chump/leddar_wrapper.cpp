#include "Arduino.h"
#include "leddar_wrapper.h"
#include "xbee.h"

HardwareSerial & LeddarSerial = Serial2;

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
  while (Serial2.available())
  {
    Serial2.read();
  }
  len = 0;
  memset(receivedData, 0, 256);
  
  //send message on uart
  sendData[0] = 0x01; //SlaveAddress;
  sendData[1] = 0x41;
  CRC16(sendData, 2, false);
  for (i = 0; i<4; i++)
  {
    Serial2.write(sendData[i]);
  }
  Serial2.flush();
}

bool buffer_detections(){
  unsigned int crc = 0xFFFF;
  unsigned long startTime = millis();
  
  unsigned int count = Serial2.available();
  if (count > 0){
    Serial2.readBytes(receivedData+len, count);
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
//    Xbee.print("\t");
//    Serial3.print(Detections[i].Segment);
//    Serial3.print("/");
//    Serial3.print(Detections[i].Distance);
//    Serial3.print("\t");
  }
//  Xbee.print("\r\n");
//  Serial3.print("\n");
  return detection_count;
}

int get_state(unsigned int detections){
  int arm_threshold = 100;
  int fire_threshold = 60;
  for (int i = 0; i < detections; i++){
    if (Detections[i].Distance < fire_threshold){
      Xbee.write("Hit!\r\n");
      return HIT_ZONE;
    }
  }
  for (int i = 0; i < detections; i++){
    if (Detections[i].Distance < arm_threshold){
      Xbee.write("Arming!\r\n");
      return ARM_ZONE;
    }
  }
  Xbee.write("Nothing to see here\r\n");
  return FAR_ZONE;
}

Detection get_min_detection (unsigned int num_detections) {
  Detection min_detection = Detections[0];
  for (unsigned int i = 1; i < num_detections; i++){
    if (Detections[i].Distance < min_detection.Distance) {
      min_detection = Detections[i];
    }
  }
//  Serial.print(min_detection.Distance);
  return min_detection;
}

Object_call call_nearest_obj (unsigned int num_detections) {
  float min_distance = Detections[(num_detections - 1)].Distance;
  Detection min_detection = Detections[(num_detections - 1)];
  float right_edge = 16.0;
  float left_edge = 0.0;

  // loop is backwards because Leddar is mounted upside down!
  for (int i = num_detections - 2; i >= 0; i--) {
    int delta = Detections[i].Distance - min_distance;
    if (delta < -90) {
      left_edge = (float) (15 - Detections[i].Segment);
      min_distance = Detections[i].Distance;
      right_edge = (float) (16 - i);
    } else if (delta > 90) {
      right_edge = (float) (16 - Detections[i].Segment);
    } else {
      if (Detections[i].Segment == 15) {
        left_edge = 0.0;
      }
    }
  }
  Object_call Nearest_obj;
  Nearest_obj.Distance = min_distance;
  Nearest_obj.Angle = (left_edge + right_edge) / 2 - 8;
  return Nearest_obj;
}

