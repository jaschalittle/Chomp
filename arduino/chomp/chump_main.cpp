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

static HardwareSerial& Debug = Serial;
static HardwareSerial& Sbus = Serial3;

void chumpSetup() {
  leddarWrapperInit();
  attachRCInterrupts();
  requestDetections();
  pwmOutputSetup();
  Debug.begin(115200);
  Sbus.begin(100000);
  Sbus.setTimeout(10);
}

static char previous_rc_bitfield = 0;
static float left_rc_duty = PWM_NEUTRAL;
static float right_rc_duty = PWM_NEUTRAL;
static unsigned long last_request_time = micros();
static float steer_bias = 0.0; // positive turns right, negative turns left
// Object Nearest_obj;
// static float target_angle = 0.0;

// new loop for chump driving should get pwm values set by interrupts to send out pwm to output pins, which will go to motor controllers. 
// setup function needs to put these at appropriate neutral values. need some global enable too to make sure that radio contact 
// is ensured! set another pin to self drive mode, and that mode can do something like spin left for 1s, then right for 1s, and so on 
// for demonstration
unsigned long last_loop_begin = micros();

void chumpLoop() {
//  Serial.println(micros() - last_loop_begin);
//  last_loop_begin = micros();

  unsigned long start_time = micros();
  if (micros() - last_request_time > 1000000) {
    last_request_time = micros();
    requestDetections();
  }
  
  // check if there is new Leddar data
  bool complete = bufferDetections();
  if (complete) {
    unsigned int detection_count = parseDetections();
    last_request_time = micros();
    int current_leddar_state = getState(detection_count, getDetections());
    requestDetections();
    steer_bias = pidSteer(detection_count, getDetections());
  }

  // should this be renamed to weapons_rc for clarity?
  bool rc_complete = bufferSbusData();
  if (rc_complete) {
    parseSbus();
  }

  left_rc_duty = getLeftRc();
  right_rc_duty = getRightRc();

  // right side value needs to be reversed for chump because of motor configuration. might differ on chomp.
  float l_tread_mix = left_rc_duty;
  float r_tread_mix = PWM_NEUTRAL + (PWM_NEUTRAL - right_rc_duty);

  bool targetingEnable = getTargetingEnable() > 0.09;
//   float tuning_offset = 0.005;
  if (targetingEnable) {
    l_tread_mix += steer_bias;
    r_tread_mix += steer_bias;
    Debug.print(steer_bias);
    Debug.println();
//     l_tread_mix += tuning_offset;
//     r_tread_mix -= tuning_offset;
//   } else {
//     l_tread_mix -= tuning_offset;
//     r_tread_mix += tuning_offset;
  }

  pwmDutyL(l_tread_mix);
  pwmDutyR(r_tread_mix);
  Debug.print(l_tread_mix, 5);
  Debug.print(" ");
  Debug.print(r_tread_mix, 5);
  Debug.println();
}





