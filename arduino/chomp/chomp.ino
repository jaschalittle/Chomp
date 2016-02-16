#include <Leddar.h>
#include "rc.h"
#include "leddar_wrapper.h"
#include "sensors.h"

// Xbee configuration notes:
// Followed tutorial here: https://eewiki.net/display/Wireless/XBee+Wireless+Communication+Setup
// Xbee SN 13A200 40BEFC5C is set to Coordinator AT, and DH/DL programmed to the SN of the Router AT
// Xbee SN 13A200 40B9D1B1 is set to Router AT, and DH/DL programmed to the SN of the Coordinator AT
// They're talking on PAN ID 2001 (A Space Odyssey)
HardwareSerial & Xbee = Serial2;

bool VERBOSE = false;

void setup() {
  Xbee.begin(9600);
  leddar_wrapper_init();
  attachRCInterrupts();
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

void fire(){
  if (enabled()){
    Xbee.write("yo");
    retract();
  }
}

void retract(){
  Xbee.write("oy");
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

void loop() {
  int start_time = micros();
  int current_leddar_state = poll_leddar();
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
  int loop_time = micros() - start_time;
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
  
  Xbee.print(loop_time);
  Xbee.print("\r\n");
}
