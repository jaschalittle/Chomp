#include "Arduino.h"
#include "chomp_main.h"
#include "rc.h"
#include "leddar_io.h"
#include "autofire.h"
#include "sensors.h"
#include "xbee.h"
#include "telem.h" 
#include "pins.h"

static HardwareSerial& Debug = Serial;
static HardwareSerial& Sbus = Serial3;

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
    digitalWrite(RED, LOW);
    delay(10);
    // Open retract
    digitalWrite(RETRACT_VALVE_DO, HIGH);
    delay(500);
    // Close retract
    digitalWrite(RETRACT_VALVE_DO, LOW);
  }
}

void fire(char bitfield){
  if (weaponsEnabled(bitfield) && !hammer_fired){
    hammer_fired = 1;
    Debug.write("Fire!\r\n");

    // Seal vent (which is normally closed)
    digitalWrite(VENT_VALVE_DO, LOW);
    delay(10);
    // Open fire
    digitalWrite(THROW_VALVE_DO, HIGH);
    digitalWrite(RED, HIGH);
    delay(100);
    // Close fire
    digitalWrite(THROW_VALVE_DO, LOW);
    delay(1000);
    
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
  pinMode(ENABLE_VALVE_DO, OUTPUT);
  pinMode(THROW_VALVE_DO, OUTPUT);
  pinMode(VENT_VALVE_DO, OUTPUT);
  pinMode(RETRACT_VALVE_DO, OUTPUT);
  digitalWrite(GREEN, HIGH);
  digitalWrite(ENABLE_VALVE_DO, HIGH);
  digitalWrite(VENT_VALVE_DO, HIGH);
  delay(10);

}

static int previous_leddar_state = FAR_ZONE;
static char previous_rc_bitfield = 0;
static unsigned long last_request_time = micros();
static unsigned long last_telem_time = micros();
void chompLoop() {
  unsigned long start_time = micros();
  if (micros() - last_request_time > 1000000){
    last_request_time = micros();
    requestDetections();
  }
  bool complete = bufferDetections();
  if (complete){
    unsigned int detection_count = parseDetections();
    last_request_time = micros();
    LeddarState current_leddar_state = getState(detection_count, getDetections());
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
//          digitalWrite(22, HIGH);
//          Debug.write("Leddar! ");
          //fire(previous_rc_bitfield /*hammer intensity*/); // TODO - think about whether using previous bitfield is safe here
        } else {
          digitalWrite(21, HIGH);
          previous_leddar_state = ARM_ZONE; // Going from far to hit counts as arming
        }
        break;
    }
    sendLeddarTelem(getDetections(), detection_count, current_leddar_state);
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
  unsigned long loop_speed = micros() - start_time;
  // Read other sensors, to report out
  float pressure = readMlhPressure();
  float angle = readAngle();

  if (micros() - last_telem_time > 200000){
//    send_sensor_telem(loop_speed, pressure);
    last_telem_time = micros();
  }
}
