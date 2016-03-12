#include "Arduino.h"
#include "chump_main.h"
#include "rc.h"
#include "leddar_io.h"
#include "autofire.h"
#include "chump_targeting.h"
#include "sensors.h"
#include "xbee.h"
#include "pwm_drive.h"
#include "telem.h"
#include "pins.h"

void chump_setup() {
  xbee_init();
  leddar_wrapper_init();
  attachRCInterrupts();
  request_detections();
  pwm_output_setup();
  Serial.begin(115200);
  Serial3.begin(100000);
  Serial3.setTimeout(10);
//  Serial3.begin(57600);
}

static char previous_rc_bitfield = 0;
static float left_rc_duty = pwm_neutral;
static float right_rc_duty = pwm_neutral;
static unsigned long last_request_time = micros();
static float steer_bias = 0.0; // positive turns right, negative turns left
Object Nearest_obj;
static float target_angle = 0.0;

// new loop for chump driving should get pwm values set by interrupts to send out pwm to output pins, which will go to motor controllers. 
// setup function needs to put these at appropriate neutral values. need some global enable too to make sure that radio contact 
// is ensured! set another pin to self drive mode, and that mode can do something like spin left for 1s, then right for 1s, and so on 
// for demonstration
unsigned long last_loop_begin = micros();

void chump_loop() {
//  Serial.println(micros() - last_loop_begin);
//  last_loop_begin = micros();

  unsigned long start_time = micros();
  if (micros() - last_request_time > 1000000) {
    last_request_time = micros();
    request_detections();
  }
  
  // check if there is new Leddar data
  bool complete = buffer_detections();
  if (complete) {
    unsigned int detection_count = parse_detections();
    last_request_time = micros();
    int current_leddar_state = get_state(detection_count, get_detections());
    request_detections();
    steer_bias = PidSteer(detection_count, get_detections());
  }

  // should this be renamed to weapons_rc for clarity?
  bool rc_complete = bufferSbusData();
  if (rc_complete) {
    parse_sbus();
  }

  left_rc_duty = get_left_rc();
  right_rc_duty = get_right_rc();

  // right side value needs to be reversed for chump because of motor configuration. might differ on chomp.
  float l_tread_mix = left_rc_duty;
  float r_tread_mix = pwm_neutral + (pwm_neutral - right_rc_duty);

  bool targeting_enable = get_targeting_enable() > 0.09;
  if (targeting_enable) {
    l_tread_mix += steer_bias;
    r_tread_mix += steer_bias;
  }

  pwm_duty_L(l_tread_mix);
  pwm_duty_R(r_tread_mix);
}





