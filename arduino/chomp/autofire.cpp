#include "Arduino.h"
#include "autofire.h"
#include "leddar_io.h"

#define CENTER_ZONE_MIN 4
#define CENTER_ZONE_MAX 12

#define FACE_OFFSET 20
#define ARM_THRESHOLD 150
#define FIRE_THRESHOLD 90
#define CONTIG_THRESHOLD 2
LeddarState get_state(unsigned int detection_count, Detection* detections){

  int last_detected_segment = 0;
  int contiguous = 1;
  for (int i = 0; i < detection_count; i++){
    if (detections[i].Segment < CENTER_ZONE_MAX && detections[i].Segment > CENTER_ZONE_MIN &&
        detections[i].Distance - FACE_OFFSET < FIRE_THRESHOLD){
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
    return HIT_ZONE;
  }
  
  last_detected_segment = 0;
  contiguous = 1;
  for (int i = 0; i < detection_count; i++){
    if (detections[i].Segment < CENTER_ZONE_MAX && detections[i].Segment > CENTER_ZONE_MIN &&
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

