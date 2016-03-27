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
#include "weapons.h"

// SAFETY CODE ----------------------------------------------------
void safeState(){
  valveReset();
  flameEnd();
  // magnets off 
}

static volatile int WEAPONS_ENABLE_pwm_val = 500; // disabled
static volatile int WEAPONS_ENABLE_prev_time = 0;
#define WEAPONS_ENABLE_THRESHOLD 1500
void weaponsEnableFalling();
void weaponsEnableRising(){
  attachInterrupt(WEAPONS_ENABLE, weaponsEnableFalling, FALLING );
  WEAPONS_ENABLE_prev_time = micros();
}

void weaponsEnableFalling(){
  attachInterrupt(WEAPONS_ENABLE, weaponsEnableRising, RISING);
  WEAPONS_ENABLE_pwm_val = micros() - WEAPONS_ENABLE_prev_time;
  if ( WEAPONS_ENABLE_pwm_val < WEAPONS_ENABLE_THRESHOLD){
    safeState();
    g_enabled = false;
  } else {
    g_enabled = true;
  }
}
// -----------------------------------------------------------------

void chompSetup() {
    attachInterrupt(WEAPONS_ENABLE, weaponsEnableRising, RISING);
    // xbeeInit();
    Debug.begin(115200);
    Sbus.begin(100000);
    leddarWrapperInit();
    attachRCInterrupts();
    valveSetup();
    pinMode(GREEN, OUTPUT);
    pinMode(RED, OUTPUT);
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
        // Debug.write("Request\r\n");
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
                    drive(0, 0);
                //   fire(previous_rc_bitfield /*hammer intensity*/); // TODO - think about whether using previous bitfield is safe here

                    if (autofireEnabled(previous_rc_bitfield)){
                        // I think if serial connection to SBUS is lost, such that no SBUS failsafe received, this will remain stale and could be unsafe
                        fire(previous_rc_bitfield /*hammer intensity*/); // TODO - think about whether using previous bitfield is safe here
                    }
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
        char diff = previous_rc_bitfield ^ current_rc_bitfield;
        if (diff) {
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
    drive(l_tread_mix, r_tread_mix);

    unsigned long loop_speed = micros() - start_time;
    // Read other sensors, to report out
    //float pressure = readMlhPressure();
    float angle = readAngle();

    if (micros() - last_telem_time > 200000){
        // send_sensor_telem(loop_speed, pressure);
        // last_telem_time = micros();
    }
}
