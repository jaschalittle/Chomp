#include "Arduino.h"
#include "rc.h"
#include "leddar_io.h"
#include "autofire.h"
#include "sensors.h"
#include "xbee.h"
#include "telem.h" 
#include "pins.h"



bool enabled(){
  return false;//WEAPONS_ENABLE_pwm_value > WEAPONS_ENABLE_threshold;
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

void phidget_test(){
  pinMode(14, OUTPUT);
  digitalWrite(14, HIGH);
  digitalWrite(21, HIGH);
  delay(1000);
  digitalWrite(14, LOW);
  digitalWrite(21, LOW);
  delay(1000);
}

void valve_reset(){
  digitalWrite(ENABLE_VALVE_DO, LOW);
  digitalWrite(THROW_VALVE_DO, LOW);
  digitalWrite(VENT_VALVE_DO, LOW);
  digitalWrite(RETRACT_VALVE_DO, LOW);
}

void fire_test(){
  digitalWrite(GREEN, HIGH);
  digitalWrite(ENABLE_VALVE_DO, HIGH);
  delay(1000);
  
  // Seal vent (which is normally closed)
  digitalWrite(VENT_VALVE_DO, LOW);
  delay(10);
  // Open fire
  digitalWrite(THROW_VALVE_DO, HIGH);
  digitalWrite(RED, HIGH);
  delay(100);
  // Close fire
  digitalWrite(THROW_VALVE_DO, LOW);
  // Open vent
  digitalWrite(VENT_VALVE_DO, HIGH);
  digitalWrite(RED, LOW);
  delay(500);
  // Open retract
  digitalWrite(RETRACT_VALVE_DO, HIGH);
  delay(100);
  // Close retract
  digitalWrite(RETRACT_VALVE_DO, LOW);
  
  digitalWrite(GREEN, LOW);
  digitalWrite(ENABLE_VALVE_DO, LOW);
}

void chomp_setup() {
  xbee_init();
  leddar_wrapper_init();
  attachRCInterrupts();
  valve_reset();
  pinMode(ENABLE_VALVE_DO, OUTPUT);
  pinMode(THROW_VALVE_DO, OUTPUT);
  pinMode(VENT_VALVE_DO, OUTPUT);
  pinMode(RETRACT_VALVE_DO, OUTPUT);
  fire_test();
}

int previous_leddar_state = FAR_ZONE;
char previous_rc_bitfield = 0;
unsigned long last_request_time = micros();
unsigned long last_telem_time = micros();
void chomp_loop() {
  unsigned long start_time = micros();
  if (micros() - last_request_time > 1000000){
    last_request_time = micros();
    request_detections();
//    Xbee.print("Requesting\r\n");
  }
  bool complete = buffer_detections();
  if (complete){
    unsigned int detection_count = parse_detections();
//    Xbee.print(micros() - last_request_time);
//    Xbee.print("\r\n");
    last_request_time = micros();
    LeddarState current_leddar_state = get_state(detection_count, get_detections());
    switch (current_leddar_state){
      case FAR_ZONE:
        digitalWrite(21, LOW);
        digitalWrite(22, LOW);
        previous_leddar_state = current_leddar_state;
        break;
      case ARM_ZONE:
        digitalWrite(22, LOW);
        digitalWrite(21, HIGH);
        previous_leddar_state = current_leddar_state;
        break;
      case HIT_ZONE:
        if (previous_leddar_state == ARM_ZONE) {
          digitalWrite(22, HIGH);
          fire(/*hammer intensity*/);
        } else {
          digitalWrite(21, HIGH);
          previous_leddar_state = ARM_ZONE; // Going from far to hit counts as arming
        }
        break;
    }
    send_leddar_telem(get_detections(), detection_count, current_leddar_state);
    request_detections();
  }

  // React to RC state changes
  char current_rc_bitfield = get_rc_bitfield();
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
  float angle = readAngle();

  if (micros() - last_telem_time > 15000){
    send_sensor_telem(loop_speed, pressure);
    last_telem_time = micros();
  }
}
