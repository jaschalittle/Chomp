#include "Arduino.h"
#include "leddar_io.h"
#include "xbee.h"


void leddarWrapperInit(){
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
void requestDetections(){
  unsigned int i = 0;
  unsigned char sendData[4] = {0};
  //clear serial buffer
  while (LeddarSerial.available())
  {
    LeddarSerial.read();
  }
  len = 0;
  memset(receivedData, 0, 256);
  
  //send message on uart
  sendData[0] = 0x01; //SlaveAddress;
  sendData[1] = 0x41;
  CRC16(sendData, 2, false);
  for (i = 0; i<4; i++)
  {
    LeddarSerial.write(sendData[i]);
  }
  LeddarSerial.flush();
}

bool bufferDetections(){
  unsigned int crc = 0xFFFF;
  unsigned long startTime = millis();
  
  unsigned int count = LeddarSerial.available();
  if (count > 0){
    LeddarSerial.readBytes(receivedData+len, count);
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
unsigned int parseDetections(){
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

Detection* getDetections(){
  return Detections;
}

