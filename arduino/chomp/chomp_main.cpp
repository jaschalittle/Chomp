#include "Arduino.h"
#include "chomp_main.h"
#include "rc.h"
#include "leddar_io.h"
#include "autofire.h"
#include "sensors.h"
#include "xbee.h"
#include "telem.h" 
#include "pins.h"
#include "drive.h"
#include "SoftwareSerial.h"


bool weaponsEnabled(char bitfield){
  return bitfield & WEAPONS_ENABLE_BIT;
}

static bool hammer_fired = 0;
void retract(char bitfield){
  if (weaponsEnabled(bitfield) && hammer_fired){
    hammer_fired = 0;
    Debug.write("Retract\r\n");
    // Open vent
    digitalWrite(VENT_VALVE_DO, HIGH);
    delay(10);
    // Open retract
    digitalWrite(RETRACT_VALVE_DO, HIGH);
    delay(300);
    // Close retract
    digitalWrite(RETRACT_VALVE_DO, LOW);
    delay(300); // time to finish retract before it can fire again
  }
}

void fire(char bitfield){
  if (weaponsEnabled(bitfield) && !hammer_fired){
    hammer_fired = 1;
    // Debug.write("Fire!\r\n");
    // Seal vent (which is normally closed)
    digitalWrite(VENT_VALVE_DO, LOW);
    delay(10);
    // Open fire
    digitalWrite(THROW_VALVE_DO, HIGH);
    delay(300);
    // Close fire
    digitalWrite(THROW_VALVE_DO, LOW);
    delay(500);
    // Open vent
    digitalWrite(VENT_VALVE_DO, HIGH);
  }
}

void flameStart(char bitfield){
  if (weaponsEnabled(bitfield)){
    Debug.write("Flame!\r\n");
  }
}

void flameEnd(){
  Debug.write("Flame off\r\n");
}

void valveReset(){
  digitalWrite(ENABLE_VALVE_DO, LOW);
  digitalWrite(THROW_VALVE_DO, LOW);
  digitalWrite(VENT_VALVE_DO, LOW);
  digitalWrite(RETRACT_VALVE_DO, LOW);
}

void chompSetup() {
  Debug.begin(115200);
  Sbus.begin(100000);
  leddarWrapperInit();
  attachRCInterrupts();
  valveReset();
  pinMode(GREEN, OUTPUT);
  pinMode(RED, OUTPUT);
  pinMode(ENABLE_VALVE_DO, OUTPUT);
  pinMode(THROW_VALVE_DO, OUTPUT);
  pinMode(VENT_VALVE_DO, OUTPUT);
  pinMode(RETRACT_VALVE_DO, OUTPUT);
  digitalWrite(ENABLE_VALVE_DO, HIGH);
  digitalWrite(VENT_VALVE_DO, HIGH);
  delay(10);
}

static int previous_leddar_state = FAR_ZONE;
static char previous_rc_bitfield = 0;
static unsigned long last_request_time = micros();
static unsigned long last_telem_time = micros();
static int16_t left_drive_value = 0;
static int16_t right_drive_value = 0;
void chompLoop() {
  unsigned long start_time = micros();
  if (micros() - last_request_time > 1000000){
    last_request_time = micros();
    requestDetections();
    Debug.write("Request\r\n");
  }
  bool complete = bufferDetections();
  if (complete){
    unsigned int detection_count = parseDetections();
    last_request_time = micros();
    LeddarState current_leddar_state = getState(detection_count, getDetections());
    switch (current_leddar_state){
      case FAR_ZONE:
        digitalWrite(RED, LOW);
        digitalWrite(GREEN, LOW);
        previous_leddar_state = current_leddar_state;
        break;
      case ARM_ZONE:
        digitalWrite(RED, LOW);
        digitalWrite(GREEN, HIGH);
        previous_leddar_state = current_leddar_state;
        break;
      case HIT_ZONE:
        if (previous_leddar_state == ARM_ZONE) {
          digitalWrite(RED, HIGH);
          driveL(0);
          driveR(0);
          fire(previous_rc_bitfield /*hammer intensity*/); // TODO - think about whether using previous bitfield is safe here
        } else {
          digitalWrite(GREEN, HIGH);
          previous_leddar_state = ARM_ZONE; // Going from far to hit counts as arming
        }
        break;
    }
    // sendLeddarTelem(getDetections(), detection_count, current_leddar_state);
    requestDetections();
  }

  if (bufferSbusData()){
    parseSbus();
    // React to RC state changes
    char current_rc_bitfield = getRcBitfield();
    if ( previous_rc_bitfield != current_rc_bitfield ){
      char diff = previous_rc_bitfield ^ current_rc_bitfield;
      // Global enable -> disable
      if( (diff & WEAPONS_ENABLE_BIT) && !(current_rc_bitfield & WEAPONS_ENABLE_BIT)){
        flameEnd();
      }
      // Flame on -> off
      if( (diff & FLAME_CTRL_BIT) && !(current_rc_bitfield & FLAME_CTRL_BIT) ){
        flameEnd();
      }
      // Flame off -> on
      if( (diff & FLAME_CTRL_BIT) && (current_rc_bitfield & FLAME_CTRL_BIT) ){
        flameStart(current_rc_bitfield);
      }
      // Manual hammer fire
      if( (diff & HAMMER_FIRE_BIT) && (current_rc_bitfield & HAMMER_FIRE_BIT)){
        fire(current_rc_bitfield); // checks enable internally
      }
      if( (diff & HAMMER_RETRACT_BIT) && (current_rc_bitfield & HAMMER_RETRACT_BIT)){
        retract(current_rc_bitfield); // checks enable internally
      }
    }
    previous_rc_bitfield = current_rc_bitfield;
  }
  left_drive_value = getLeftRc();
  right_drive_value = getRightRc();
  float l_tread_mix = left_drive_value;
  float r_tread_mix = -right_drive_value;
  driveL(l_tread_mix);
  driveR(r_tread_mix);
  
  unsigned long loop_speed = micros() - start_time;
  // Read other sensors, to report out
//   float pressure = readMlhPressure();
//   float angle = readAngle();

  if (micros() - last_telem_time > 200000){
//    send_sensor_telem(loop_speed, pressure);
    last_telem_time = micros();
  }
}
