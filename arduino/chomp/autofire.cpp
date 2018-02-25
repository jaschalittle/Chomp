#include "Arduino.h"
#include "autofire.h"
#include "leddar_io.h"
#include "telem.h"
#include "pins.h"

// Inclusive range
#define CENTER_ZONE_ARM_MIN 0
#define CENTER_ZONE_ARM_MAX 15

// inclusive start, exclusive end
#define CENTER_ZONE_FIRE_MIN 6
#define CENTER_ZONE_FIRE_MAX 10

#define FACE_OFFSET 20
#define ARM_THRESHOLD 150
// 30-90, 60 cm neutral
// #define FIRE_THRESHOLD 70
#define CONTIG_THRESHOLD 2

int APPROACH_THRESHOLDS[LEDDAR_SEGMENTS] = { 76, 76, 75, 74, 73, 72, 71, 70, 70, 71, 72, 73, 74, 75, 76, 76 };
static const size_t center_segment = (LEDDAR_SEGMENTS/2);
#define APPROACH_BUFFER_SIZE 4

float bufferVelocity(int8_t* buf, int8_t curs, bool assert_pos){
  int16_t delta_sum = 0;
#ifdef HARD_WIRED
  String msg("{ ");
  for(int i = curs; i < curs + APPROACH_BUFFER_SIZE; i++)
  {
    msg += buf[i % APPROACH_BUFFER_SIZE];
    msg += ", ";
  }
  msg += " }";
  debug_print(msg);
#endif
  for(int i = curs + 1; i < curs + APPROACH_BUFFER_SIZE; i++){
    int16_t delta = buf[i % APPROACH_BUFFER_SIZE] - buf[(i - 1) % APPROACH_BUFFER_SIZE];
    delta_sum += delta;
    if (abs(delta) >= 3) { return 0.0; } // too fast for our blood
    if (assert_pos){
      if (delta < 0) { return 0.0; }
    } else {
      if (delta > 0) { return 0.0; }
    }
  }
  return delta_sum/(float)(APPROACH_BUFFER_SIZE-1);
}

static int8_t left_buffer[APPROACH_BUFFER_SIZE];
static int8_t left_buffer_cursor = 0;
bool leftApproach(const Detection (&detections)[LEDDAR_SEGMENTS]){
  int16_t last_detected_segment = -1;
  int16_t inner_contiguous_edge = -1;
  for (uint8_t i = 0; i < LEDDAR_SEGMENTS && detections[i].Segment < center_segment; i++){
    if (detections[i].Distance - FACE_OFFSET < APPROACH_THRESHOLDS[detections[i].Segment]){
      if (detections[i].Segment - last_detected_segment == 1){
        inner_contiguous_edge = detections[i].Segment;
      } else { break; }
      last_detected_segment = detections[i].Segment;
    }
  }

  left_buffer[left_buffer_cursor] = inner_contiguous_edge;
  left_buffer_cursor = (left_buffer_cursor + 1) % APPROACH_BUFFER_SIZE;

  if (inner_contiguous_edge != -1){
    float vel = bufferVelocity(left_buffer, left_buffer_cursor, true);
    int16_t predicted_seg = inner_contiguous_edge + (int16_t)(vel*10.0);
    bool fire = predicted_seg >= CENTER_ZONE_FIRE_MAX;
#ifdef HARD_WIRED
    String msg("Left inner edge: ");
    msg += inner_contiguous_edge;
    msg += "\t";
    msg += vel;
    msg += "\t";
    msg += predicted_seg;
    debug_print(msg);
    if (fire){debug_print(" FIRE!");}
#endif
    return fire;
  }
  return false;
}

static int8_t right_buffer[APPROACH_BUFFER_SIZE];
static int8_t right_buffer_cursor = 0;
bool rightApproach(const Detection (&detections)[LEDDAR_SEGMENTS]){
  int16_t last_detected_segment = LEDDAR_SEGMENTS;
  int16_t inner_contiguous_edge = LEDDAR_SEGMENTS;
  for (int i = LEDDAR_SEGMENTS - 1; i >=0 && detections[i].Segment >= center_segment; i--){
    if (detections[i].Distance - FACE_OFFSET < APPROACH_THRESHOLDS[detections[i].Segment]){
      if (last_detected_segment - detections[i].Segment  == 1){
        inner_contiguous_edge = detections[i].Segment;
      } else { break; }
      last_detected_segment = detections[i].Segment;
    }
  }

  right_buffer[right_buffer_cursor] = inner_contiguous_edge;
  right_buffer_cursor = (right_buffer_cursor + 1) % APPROACH_BUFFER_SIZE;

  if (inner_contiguous_edge != LEDDAR_SEGMENTS){
    float vel = bufferVelocity(right_buffer, right_buffer_cursor, false);
    int16_t predicted_seg = inner_contiguous_edge + (int16_t)(vel*10.0);
    bool fire = predicted_seg <= CENTER_ZONE_FIRE_MIN;
#ifdef HARD_WIRED
    String msg("Right inner edge: ");
    msg += inner_contiguous_edge;
    msg += "\t";
    msg += vel;
    msg += "\t";
    msg += predicted_seg;
    debug_print(msg);
    if (fire){debug_print(" FIRE!");}
#endif
    return fire;
  }
  return false;
}

LeddarState getState(const Detection (&min_detections)[LEDDAR_SEGMENTS], int16_t fire_threshold){
  int last_detected_segment = -1; // Off the left edge
  int contiguous = 0;

  // For firing, require that the entire center zone be occupied
  for (int i = CENTER_ZONE_FIRE_MIN; i < CENTER_ZONE_FIRE_MAX; i++){
    if (min_detections[i].Distance - FACE_OFFSET < fire_threshold){
      contiguous += 1;
    } else {
      break;
    }
  }
  if (contiguous == CENTER_ZONE_FIRE_MAX - CENTER_ZONE_FIRE_MIN){
    return HIT_ZONE;
  }

  if (leftApproach(min_detections) || rightApproach(min_detections)){
    return PREDICTIVE_HIT_ZONE;
  }

  // For arming, only require that we detect a multi-segment object in the arm zone somewhere
  last_detected_segment = -1;
  contiguous = 0;
  for (int i = 0; i < LEDDAR_SEGMENTS; i++){
    if ( min_detections[i].Distance - FACE_OFFSET < ARM_THRESHOLD){
      if (min_detections[i].Segment - last_detected_segment == 1){
        contiguous += 1;
        if (contiguous >= CONTIG_THRESHOLD){
          return ARM_ZONE;
        }
      } else {
        contiguous = 1;
      }
      last_detected_segment = min_detections[i].Segment;
    }
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

