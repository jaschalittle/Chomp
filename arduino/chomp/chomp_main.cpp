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
#include "utils.h"
#include "chump_targeting.h"
#include <avr/wdt.h>


// SAFETY CODE ----------------------------------------------------
void safeState(){
    valveSafe();
    flameSafe();
    magnetSafe();
}

void enableState(){
   valveEnable();
//    flameEnable(); 
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
        if (!g_enabled) {
            g_enabled = true;
            enableState();
        }  
    }
}
// -----------------------------------------------------------------

void chompSetup() {
    // Come up safely
    safeState();
    attachInterrupt(WEAPONS_ENABLE, weaponsEnableRising, RISING);
    wdt_enable(WDTO_4S);
#ifdef HARD_WIRED
    Debug.begin(115200);
    Debug.println("STARTUP");
#else
    xbeeInit();
#endif    
    Sbus.begin(100000);
    driveSetup();
    leddarWrapperInit();
    sensorSetup();
    attachRCInterrupts();
}

static int16_t previous_leddar_state = FAR_ZONE;
static int8_t previous_rc_bitfield = 0;
static uint16_t hammer_intensity = 0;
static uint32_t last_request_time = micros();
static uint32_t last_telem_time = micros();
static uint32_t last_leddar_telem_time = micros();
static int16_t left_drive_value = 0;
static int16_t right_drive_value = 0;
static int16_t steer_bias = 0; // positive turns right, negative turns left

void chompLoop() {
    uint32_t start_time = micros();
    if (micros() - last_request_time > 100000UL){
        last_request_time = micros();
        requestDetections();
    }
    if (bufferDetections()){
        uint8_t detection_count = parseDetections();
        last_request_time = micros();
        LeddarState current_leddar_state = getState(detection_count, getDetections());
        switch (current_leddar_state){
            case FAR_ZONE:
                previous_leddar_state = current_leddar_state;
                break;
            case ARM_ZONE:
                previous_leddar_state = current_leddar_state;
                break;
            case HIT_ZONE:
                if (previous_leddar_state == ARM_ZONE) {
                    if (autofireEnabled(previous_rc_bitfield)){
                        fire( hammer_intensity );
                    }
                } else {
                    previous_leddar_state = ARM_ZONE; // Going from far to hit counts as arming
                }
                break;
        }
        // Send subsampled leddar telem
        if (micros() - last_leddar_telem_time > 100000L){
          bool success = sendLeddarTelem(getDetections(), detection_count, current_leddar_state);
          if (success){
            last_leddar_telem_time = micros();
          }
        }      
        requestDetections();
    }

    if (bufferSbusData()){
        bool in_failsafe_state = parseSbus();
        if (!in_failsafe_state) {
            wdt_reset();
            // React to RC state changes
            char current_rc_bitfield = getRcBitfield();
            char diff = previous_rc_bitfield ^ current_rc_bitfield;
            hammer_intensity = getHammerIntensity();
            if (diff) {
                // Flame on -> off
                if( (diff & FLAME_CTRL_BIT) && !(current_rc_bitfield & FLAME_CTRL_BIT) ){
                    flameEnd();
                }
                // Flame off -> on
                if( (diff & FLAME_CTRL_BIT) && (current_rc_bitfield & FLAME_CTRL_BIT) ){
                    flameStart();
                }
                // Manual hammer fire
                if( (diff & HAMMER_FIRE_BIT) && (current_rc_bitfield & HAMMER_FIRE_BIT)){
                    if (current_rc_bitfield & DANGER_CTRL_BIT){
                      no_angle_fire(hammer_intensity);
                    } else {
                      fire(hammer_intensity);
                    }
                }
                if( (diff & HAMMER_RETRACT_BIT) && (current_rc_bitfield & HAMMER_RETRACT_BIT)){
                  if (current_rc_bitfield & DANGER_CTRL_BIT){
                    no_angle_retract();
                  } else {
                    retract();
                  }
                }
                if( (diff & GENTLE_HAM_F_BIT) && (current_rc_bitfield & GENTLE_HAM_F_BIT)) {
                    gentleFire();
                }
                if( (diff & GENTLE_HAM_R_BIT) && !(current_rc_bitfield & GENTLE_HAM_R_BIT)) {
                    retract();
                }
                if( (diff & MAG_CTRL_BIT) && (current_rc_bitfield & MAG_CTRL_BIT)){
                    magOn();
                }
                if( (diff & MAG_CTRL_BIT) && !(current_rc_bitfield & MAG_CTRL_BIT)){
                    magOff();
                }
            }
            previous_rc_bitfield = current_rc_bitfield;
        }
    }
    
    left_drive_value = getLeftRc();
    right_drive_value = getRightRc();
    
    // ideally, this would only be called if there is fresh RC input. otherwise, it is ugly for drive prediction stuff
    // don't spam motor controllers -- only send drive command every 10 ms
    // drive always called now to log drive command history. only commands roboteqs if getTargetingEnable()
    if (newRc()) {
        drive(left_drive_value - steer_bias, right_drive_value - steer_bias, getTargetingEnable());
        // drive(-drive_command, drive_command, getTargetingEnable());
    }

   if (micros() - last_telem_time > 50000L){
      uint32_t loop_speed = micros() - start_time;
      int16_t pressure = 0;
      readMlhPressure(&pressure);
      uint16_t angle = 0;
      readAngle(&angle);
      bool success = sendSensorTelem(loop_speed, pressure, angle);
      if (success){
        last_telem_time = micros();
      }
    }
   xbeePushData();
}
