#include "rc.h"
#include "leddar_wrapper.h"
#include "xbee.h"
#include "pwm.h"

void setup() {
  xbee_init();
  leddar_wrapper_init();
  attachRCInterrupts();
  request_detections();
  pwm_output_setup();
  Serial.begin(115200);
}


char previous_rc_bitfield = 0;
static float aileron_duty = pwm_neutral;
static float elevator_duty = pwm_neutral;
static float throttle_duty = pwm_neutral;
unsigned long last_request_time = micros();
static float steer_bias = 0.0; // positive turns right, negative turns left
Object_call Nearest_obj;

// new loop for chump driving should get pwm values set by interrupts to send out pwm to output pins, which will go to motor controllers. 
// setup function needs to put these at appropriate neutral values. need some global enable too to make sure that radio contact 
// is ensured! set another pin to self drive mode, and that mode can do something like spin left for 1s, then right for 1s, and so on 
// for demonstration
void loop() {

  unsigned long start_time = micros();
  if (micros() - last_request_time > 1000000) {
    last_request_time = micros();
    request_detections();
  }
  
  bool complete = buffer_detections();
  if (complete) {
    unsigned int detection_count = parse_detections();
    last_request_time = micros();
//    int current_leddar_state = get_state(detection_count);
    request_detections();
    Detection min_detection = get_min_detection(detection_count);
    if (min_detection.Segment < 7) {
      steer_bias = 0.01;
    } else if (min_detection.Segment > 8) { 
      steer_bias = -0.01;
    } else {
      steer_bias = 0.0;
    }
//    steer_bias = 0.005 * (8 - (unsigned int) min_detection.Segment)
    Nearest_obj = call_nearest_obj(detection_count);
  }

  aileron_duty = get_aileron();
  elevator_duty = get_elevator();
  throttle_duty = get_throttle();
  float l_tread_mix = pwm_neutral - (pwm_neutral - aileron_duty) - (pwm_neutral - elevator_duty);
  float r_tread_mix = pwm_neutral - (pwm_neutral - aileron_duty) + (pwm_neutral - elevator_duty);

  if (throttle_duty > 0.075) {
    // centering behavior ultra basic
//    l_tread_mix += steer_bias;
//    r_tread_mix += steer_bias;

    // slightly better centering
    steer_bias = Nearest_obj.Angle * 0.005;
    l_tread_mix += steer_bias;
    r_tread_mix += steer_bias;
    
    // spinning behavior
//    pwm_duty_L(l_tread_mix - 0.02);
//    pwm_duty_R(r_tread_mix - 0.02);
//    delay(1000);
  }
  pwm_duty_L(l_tread_mix);
  pwm_duty_R(r_tread_mix);
}

//    Serial.print(min_detection.Distance);
//    Serial.print("\t");
//    Serial.print(min_detection.Segment);
//    Serial.print("\t");
//    Serial.print("\t");
//    Serial.print("\n");



