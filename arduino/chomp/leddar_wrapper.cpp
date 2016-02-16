#include "Arduino.h"
#include <Leddar.h>
#include "leddar_wrapper.h"

HardwareSerial & LeddarSerial = Serial;
Leddar16 leddar(115200,1);

void leddar_wrapper_init(){
  LeddarSerial.begin(115200);
  leddar.init();
}

int poll_leddar(){
  int fire_threshold = 60;
  char detections = leddar.getDetections();
  if (detections >= 0){
    for (int i = 0; i < leddar.NbDet; i++){
      if (leddar.Detections[i].Distance < fire_threshold){
        return HIT_ZONE;
      }
      //leddar.Detections[i].Amplitude;
    }
  }
  return FAR_ZONE;
}

void fire(){
  pinMode(27, OUTPUT);
  digitalWrite(27, HIGH);
//  Xbee.write("FIRE!\t");
//  Xbee.write(0x07);
//  Xbee.flush();
  delay(50);
  digitalWrite(27, LOW);
}
