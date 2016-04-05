#include "Arduino.h"
#include "autofire.h"
#include "leddar_io.h"

// Inclusive range
#define CENTER_ZONE_ARM_MIN 0
#define CENTER_ZONE_ARM_MAX 15

// Exclusive range: 6, 7, 8, 9
#define CENTER_ZONE_FIRE_MIN 5
#define CENTER_ZONE_FIRE_MAX 10

#define FACE_OFFSET 20
#define ARM_THRESHOLD 150
#define FIRE_THRESHOLD 70
#define CONTIG_THRESHOLD 2
LeddarState getState(unsigned int detection_count, Detection* detections){
  int last_detected_segment = 0;
  int contiguous = 1;
  for (int i = 0; i < detection_count; i++){
    if (detections[i].Segment < CENTER_ZONE_FIRE_MAX && detections[i].Segment > CENTER_ZONE_FIRE_MIN &&
        detections[i].Distance - FACE_OFFSET < FIRE_THRESHOLD){
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
    if (detections[i].Segment <= CENTER_ZONE_ARM_MAX && detections[i].Segment >= CENTER_ZONE_ARM_MIN &&
        detections[i].Distance - FACE_OFFSET < ARM_THRESHOLD){
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
  return FAR_ZONE;
}

struct HitBox
{
  int16_t xmin;
  int16_t xmax;
  int16_t ymin;
  int16_t ymax;
  HitBox( int16_t _xmin, int16_t _xmax, int16_t _ymin, int16_t _ymax ) : xmin(_xmin), xmax(_xmax), ymin(_ymin), ymax(_ymax) { }
};

bool within(int16_t x, int16_t y, HitBox hb){
  return (x <= hb.xmax && x >= hb.xmin && y <= hb.ymax && y >= hb.ymin); 
}

// Box-test predicted x/y for arm/fire regions
LeddarState getStatePredictive(int16_t target_x_after_leadtime, int16_t target_y_after_leadtime){
  const HitBox arm = { -50, 50, 60, 200 }; 
  const HitBox hit = { -25, 25, 60, 100 };
  if ( within(target_x_after_leadtime, target_y_after_leadtime, hit)){
    return HIT_ZONE;
  }
  if ( within(target_x_after_leadtime, target_y_after_leadtime, arm)){
    return ARM_ZONE;
  }
  return FAR_ZONE;
}

