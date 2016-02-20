#include "rc.h"
#include "leddar_wrapper.h"
#include "sensors.h"
#include "xbee.h"
#include "telem.h"

void setup() {
  xbee_init();
  leddar_wrapper_init();
  attachRCInterrupts();
  request_detections();
}

bool enabled(){
  return WEAPONS_ENABLE_pwm_value > WEAPONS_ENABLE_threshold;
}

char process_rc_bools(){
  char bitfield = 0;
  if ( WEAPONS_ENABLE_pwm_value > WEAPONS_ENABLE_threshold ){
    bitfield |= WEAPONS_ENABLE_BIT;
  }
  if ( FLAME_CTRL_pwm_value > FLAME_CTRL_threshold){
    bitfield |= FLAME_CTRL_BIT;
  }
  return bitfield;
}

void retract(){
  Xbee.write("oy");
}
void fire(){
  if (enabled()){
    Xbee.write("yo");
    retract();
  }
}

void flame_start(){
  if (enabled()){
    Xbee.write("burrrrn");
  }
}

void flame_end(){
  Xbee.write("sssssss");
}

int previous_leddar_state = FAR_ZONE;
char previous_rc_bitfield = 0;
unsigned long last_request_time = micros();
void loop() {
  unsigned long start_time = micros();
  if (micros() - last_request_time > 1000000){
    last_request_time = micros();
    request_detections();
    //Xbee.print("Requesting\r\n");
  }
  bool complete = buffer_detections();
  if (complete){
    unsigned int detection_count = parse_detections();
//    Xbee.print(micros() - last_request_time);
//    Xbee.print("\r\n");
    last_request_time = micros();
    int current_leddar_state = get_state(detection_count);
    switch (current_leddar_state){
      case FAR_ZONE:
      case ARM_ZONE:
        previous_leddar_state = current_leddar_state;
        break;
      case HIT_ZONE:
        if (previous_leddar_state == ARM_ZONE) {
          fire(/*hammer intensity*/);
        } else {
          previous_leddar_state = ARM_ZONE; // Going from far to hit counts as arming
        }
        break;
    }
    request_detections();
  }
  
  // React to RC state changes
  char current_rc_bitfield = process_rc_bools();
  if ( previous_rc_bitfield != current_rc_bitfield ){
    char diff = previous_rc_bitfield ^ current_rc_bitfield;
    // Global enable -> disable
    if( (diff & WEAPONS_ENABLE_BIT) && !(current_rc_bitfield & WEAPONS_ENABLE_BIT)){
      flame_end();
    }
    // Flame on -> off
    if( (diff & FLAME_CTRL_BIT) && !(current_rc_bitfield & FLAME_CTRL_BIT) ){
      flame_end();
    }
    // Flame off -> on
    if( (diff & FLAME_CTRL_BIT) && (current_rc_bitfield & FLAME_CTRL_BIT) ){
      flame_start();
    }
  }
  previous_rc_bitfield = current_rc_bitfield;

  unsigned long loop_speed = micros() - start_time;
  // Read other sensors, to report out
  float pressure = readMLHPressure();
  
  send_sensor_telem(loop_speed, pressure);
  delay(50);
  
}
