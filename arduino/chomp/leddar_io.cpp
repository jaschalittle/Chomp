#include "Arduino.h"
#include "leddar_io.h"
#include "xbee.h"
#include "pins.h"


#define AMPLITUDE_THRESHOLDING_RANGE 125
#define LEDDAR_AMPLITUDE_THRESHOLD 50

void leddarWrapperInit(){
  LeddarSerial.begin(115200);
}

bool CRC16(uint8_t *aBuffer, uint8_t aLength, bool aCheck) 
{
  uint8_t lCRCHi = 0xFF; // high byte of CRC initialized
  uint8_t lCRCLo = 0xFF; // low byte of CRC initialized
  
  for (uint16_t i = 0; i<aLength; ++i) 
  {
    uint16_t lIndex = lCRCLo ^ aBuffer[i]; // calculate the CRC
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

uint16_t len = 0;
uint8_t receivedData[256] = {0};
void requestDetections(){
  uint16_t i = 0;
  uint8_t sendData[4] = {0};
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
  // this hangs if LeddarSerial not begun. could it hang in other scenarios?
  LeddarSerial.flush();
}

bool bufferDetections(){
  uint16_t count = LeddarSerial.available();
  if (count > 0){
    LeddarSerial.readBytes(receivedData+len, count);
    len += count;
  }
  if (len > 3){
    uint8_t detection_count = receivedData[2];
    uint16_t target_len = detection_count * 5 + 11;
    if (target_len == len){
      return true; 
    }
  }
  return false;
}

Detection Detections[50];
uint8_t parseDetections(){
  if (!CRC16(receivedData, len-2, true)){
    return 0;
  }
  uint16_t detection_count = receivedData[2];
  // Parse out detection info
  for ( uint16_t i = 0; i < detection_count; i++){
    uint16_t offset = i * 5 + 3;
    uint8_t segment = 15 - (receivedData[offset+4]/16);
    uint16_t amplitude = ((float)receivedData[offset+3])*4+ ((float)receivedData[offset+2])/64;
    uint16_t distance = ((uint16_t)receivedData[offset+1])*256 + receivedData[offset];
    
    // filter near detections that are of insufficient amplitude
    if (distance > AMPLITUDE_THRESHOLDING_RANGE || amplitude > LEDDAR_AMPLITUDE_THRESHOLD) {
      Detections[detection_count - 1 - i].Distance = distance;
      Detections[detection_count - 1 - i].Amplitude = amplitude;
      Detections[detection_count - 1 - i].Segment = segment; // flip the segment ID since we're upside down
    }
  }
  
  return detection_count;
}

Detection* getDetections(){
  return Detections;
}

#define MIN_OBJECT_DISTANCE 30
// outputMinDetections should be a default-constructed array of size 16
void getMinDetections(uint8_t detection_count, Detection* inputDetections, Detection* outputMinDetections){
  for (uint8_t i = 0; i < detection_count; i++) {
    if (inputDetections[i].Distance < outputMinDetections[inputDetections[i].Segment].Distance && inputDetections[i].Distance > MIN_OBJECT_DISTANCE) {
      outputMinDetections[inputDetections[i].Segment] = inputDetections[i];
    }
  }
}

