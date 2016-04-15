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
    // wdt_enable(WDTO_4S);
    // xbeeInit();
    Debug.begin(115200);
    Sbus.begin(100000);
    DriveSerial.begin(115200);
    leddarWrapperInit();
    attachRCInterrupts();
    sensorSetup();
}

static int16_t previous_leddar_state = FAR_ZONE;
static int8_t previous_rc_bitfield = 0;
static uint32_t last_request_time = micros();
static uint32_t last_telem_time = micros();
static int16_t left_drive_value = 0;
static int16_t right_drive_value = 0;
static bool targeting_enabled = false;
void chompLoop() {
    uint32_t start_time = micros();
    if (micros() - last_request_time > 1000000){
        last_request_time = micros();
        requestDetections();
        // Debug.write("Request\r\n");
    }
    bool complete = bufferDetections();
    if (complete){
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
                    drive(0, 0);
                    if (autofireEnabled(previous_rc_bitfield)){
                        fire();
                    }
                } else {
                    previous_leddar_state = ARM_ZONE; // Going from far to hit counts as arming
                }
                break;
        }
        // sendLeddarTelem(getDetections(), detection_count, current_leddar_state);
        requestDetections();
    }

    if (bufferSbusData()){
        wdt_reset();
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
                flameStart();
            }
            // Manual hammer fire
            if( (diff & HAMMER_FIRE_BIT) && (current_rc_bitfield & HAMMER_FIRE_BIT)){
                fire_test();
            }
            if( (diff & HAMMER_RETRACT_BIT) && (current_rc_bitfield & HAMMER_RETRACT_BIT)){
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
    left_drive_value = getLeftRc();
    right_drive_value = getRightRc();
    // targeting_enabled = getTargetingEnable();
    int16_t l_tread_mix = left_drive_value;
    int16_t r_tread_mix = -right_drive_value;
    drive(l_tread_mix, r_tread_mix);

    unsigned long loop_speed = micros() - start_time;
    // Read other sensors, to report out
    // uint16_t pressure;
    // bool pressure_read_ok = readMlhPressure(&pressure);
    // uint16_t angle;
    // bool angle_read_ok = readAngle(&angle);
    // Debug.println(angle);

    if (micros() - last_telem_time > 200000){
        // send_sensor_telem(loop_speed, pressure);
        // last_telem_time = micros();
    }
}

void electricalCheckouts(){
  Debug.print("Clicking valves\r\n");
  int pulse_time = 3000;
  Debug.print("Enable\r\n");
  delay(5000);
  Debug.print("now\r\n");
  safeDigitalWrite(ENABLE_VALVE_DO, HIGH);
  delay(pulse_time);
  safeDigitalWrite(ENABLE_VALVE_DO, LOW);
  
  Debug.print("Retract\r\n");
  delay(5000);
  Debug.print("now\r\n");
  safeDigitalWrite(RETRACT_VALVE_DO, HIGH);
  delay(pulse_time);
  safeDigitalWrite(RETRACT_VALVE_DO, LOW);

  Debug.print("Vent\r\n");
  delay(5000);
  Debug.print("now\r\n");
  safeDigitalWrite(VENT_VALVE_DO, HIGH);
  delay(pulse_time);
  safeDigitalWrite(VENT_VALVE_DO, LOW);

  Debug.print("Throw\r\n");
  delay(5000);
  Debug.print("now\r\n");
  safeDigitalWrite(THROW_VALVE_DO, HIGH);
  delay(pulse_time);
  safeDigitalWrite(THROW_VALVE_DO, LOW);

  Debug.print("Clicking magnets\r\n");
  Debug.print("Magnet 1\r\n");
  delay(5000);
  Debug.print("now\r\n");
  safeDigitalWrite(MAG1_DO, HIGH);
  delay(pulse_time);
  safeDigitalWrite(MAG1_DO, LOW);

  Debug.print("Magnet 2\r\n");
  delay(5000);
  Debug.print("now\r\n");
  safeDigitalWrite(MAG2_DO, HIGH);
  delay(pulse_time);
  safeDigitalWrite(MAG2_DO, LOW);

  Debug.print("Clicking flamethrower\r\n");
  Debug.print("Igniter\r\n");
  delay(5000);
  Debug.print("now\r\n");
  safeDigitalWrite(IGNITER_DO, HIGH);
  delay(pulse_time);
  safeDigitalWrite(IGNITER_DO, LOW);

  Debug.print("Propane valve\r\n");
  delay(5000);
  Debug.print("now\r\n");
  safeDigitalWrite(PROPANE_DO, HIGH);
  delay(1000);
  safeDigitalWrite(PROPANE_DO, LOW);
  
  Debug.print("All done\r\n");
  safeState();
}
