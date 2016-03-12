#include "Arduino.h"
#include "autofire.h"
#include "leddar_io.h"

#define CENTER_ZONE_ARM_MIN 4
#define CENTER_ZONE_ARM_MAX 12

#define CENTER_ZONE_FIRE_MIN 6
#define CENTER_ZONE_FIRE_MAX 10

#define FACE_OFFSET 20
#define ARM_THRESHOLD 150
#define FIRE_THRESHOLD 70
#define CONTIG_THRESHOLD 2
LeddarState get_state(unsigned int detection_count, Detection* detections){

  int last_detected_segment = 0;
  int contiguous = 1;
  for (int i = 0; i < detection_count; i++){
    if (detections[i].Segment < CENTER_ZONE_FIRE_MAX && detections[i].Segment > CENTER_ZONE_FIRE_MIN &&
        detections[i].Distance - FACE_OFFSET < FIRE_THRESHOLD){
//      Serial.print(detections[i].Segment);
//      Serial.write("/");
//      Serial.print(detections[i].Distance);
//      Serial.write(" ");
      if (detections[i].Segment - last_detected_segment == 1){
        contiguous += 1;
      } else {
        contiguous = 1;
      }
      last_detected_segment = detections[i].Segment;
    }
  }
//  Serial.write("\r\n");
  if (contiguous >= CONTIG_THRESHOLD){
    return HIT_ZONE;
  }
  
  last_detected_segment = 0;
  contiguous = 1;
  for (int i = 0; i < detection_count; i++){
    if (detections[i].Segment < CENTER_ZONE_ARM_MAX && detections[i].Segment > CENTER_ZONE_ARM_MIN &&
        detections[i].Distance - FACE_OFFSET < ARM_THRESHOLD){
      //Xbee.write("Hit!\r\n");
      if (detections[i].Segment - last_detected_segment == 1){
        contiguous += 1;
      } else {
        contiguous = 1;
      }
      last_detected_segment = detections[i].Segment;
    }
  }

  if (contiguous >= CONTIG_THRESHOLD){
    return ARM_ZONE;
  }
  //Xbee.write("Nothing to see here\r\n");
  return FAR_ZONE;
}

