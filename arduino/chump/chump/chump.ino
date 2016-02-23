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
unsigned long last_request_time = micros();
// new loop for chump driving should use global pwm values set by interrupts to send out pwm to output pins, which will go to motors. 
// setup function needs to put these at appropriate neutral values. need some global enable too to make sure that radio contact 
// is ensured! set another pin to self drive mode, and that mode can do something like spin left for 1s, then right for 1s, and so on
void loop() {

  unsigned long start_time = micros();
  if (micros() - last_request_time > 1000000){
    last_request_time = micros();
    request_detections();
  }
  
  float Lmix = 0.075 - (0.075 - L_TREAD_pwm_val / 20000.0) + (0.075 - R_TREAD_pwm_val / 20000.0);
  float Rmix = 0.075 - (0.075 - L_TREAD_pwm_val / 20000.0) - (0.075 - R_TREAD_pwm_val / 20000.0);
  
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
//    if (min_detection.Segment & 0b00001000) {
//      Rmix += 0.005;
//    } else { 
//      Lmix += 0.005;
//    }
    Serial.print(Lmix);
    Serial.print("\t");
    Serial.print(Rmix);
    Serial.print("\t");
    Serial.print(L_TREAD_pwm_val);
    Serial.print("\t");
    Serial.print(R_TREAD_pwm_val);
    Serial.print("\t");
    Serial.print("\n");
  }
//  Serial.print(Lmix);
//  Serial.print("\t");
//  Serial.print(Rmix);
//  Serial.print("\t");
//  Serial.print(L_TREAD_pwm_val);
//  Serial.print("\t");
//  Serial.print(R_TREAD_pwm_val);
//  Serial.print("\t");
//  Serial.print("\n");
  pwm_duty_L(Lmix);
  pwm_duty_R(Rmix);
//  Ch1 (ailerons) is L, Ch2 (elevators) is R
}
