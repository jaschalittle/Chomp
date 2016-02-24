#include "rc.h"
#include "leddar_wrapper.h"
#include "xbee.h"
#include "pwm.h"

void setup() {
  xbee_init();
  leddar_wrapper_init();
  attachRCInterrupts();
  request_detections();
  pwm_setup();
  Serial.begin(115200);
}


char previous_rc_bitfield = 0;
static float aileron_duty = pwm_neutral;
static float elevator_duty = pwm_neutral;
static float throttle_duty = pwm_neutral;
unsigned long last_request_time = micros();
// new loop for chump driving should get pwm values set by interrupts to send out pwm to output pins, which will go to motor controllers. 
// setup function needs to put these at appropriate neutral values. need some global enable too to make sure that radio contact 
// is ensured! set another pin to self drive mode, and that mode can do something like spin left for 1s, then right for 1s, and so on 
// for demonstration
void loop() {

  unsigned long start_time = micros();
  if (micros() - last_request_time > 1000000){
    last_request_time = micros();
    request_detections();
  }

  aileron_duty = get_aileron();
  elevator_duty = get_elevator();
  throttle_duty = get_throttle();
  float l_tread_mix = pwm_neutral - (pwm_neutral - aileron_duty) - (pwm_neutral - elevator_duty);
  float r_tread_mix = pwm_neutral - (pwm_neutral - aileron_duty) + (pwm_neutral - elevator_duty);
  
  bool complete = buffer_detections();
  if (complete){
    unsigned int detection_count = parse_detections();
//    Xbee.print(micros() - last_request_time);
//    Xbee.print("\r\n");
    last_request_time = micros();
//    int current_leddar_state = get_state(detection_count);
    request_detections();
    Detection min_detection = get_min_detection(detection_count);
//    Serial.print(min_detection.Distance);
//    Serial.print("\t");
//    Serial.print(min_detection.Segment);
//    Serial.print("\t");
//    Serial.print("\t");
//    Serial.print("\n");
    if (min_detection.Segment & 0b00001000) {
      r_tread_mix += 0.005;
    } else { 
      l_tread_mix += 0.005;
    }
//    Serial.print(l_tread_mix);
//    Serial.print("\t");
//    Serial.print(r_tread_mix);
//    Serial.print("\t");
//    Serial.print(l_tread_duty);
//    Serial.print("\t");
//    Serial.print(r_tread_duty);
//    Serial.print("\t");
//    Serial.print("\n");
  }
  Serial.print(l_tread_mix, 5);
  Serial.print("\t");
  Serial.print(r_tread_mix, 5);
  Serial.print("\t");
  Serial.print(aileron_duty, 5);
  Serial.print("\t");
  Serial.print(elevator_duty, 5);
  Serial.print("\t");
  Serial.print(throttle_duty, 5);
  Serial.print("\n");
  pwm_duty_L(l_tread_mix);
  pwm_duty_R(r_tread_mix);
}
