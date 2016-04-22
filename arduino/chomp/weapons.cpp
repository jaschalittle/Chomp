#include "Arduino.h"
#include "weapons.h"
#include "rc.h"
#include "sensors.h"
#include "pins.h"
#include "utils.h"
#include "telem.h"
#include <avr/wdt.h>

uint8_t MAX_SAFE_ANGLE = 65;
uint8_t HAMMER_INTENSITIES_ANGLE[9] = { 3, 5, 10, 15, 20, 30, 40, 50, 65 };

uint8_t MAX_SAFE_TIME = 125;
//                                     3   5   10  15  20  30  40  50  65
uint8_t HAMMER_INTENSITIES_TIME[9] = { 25, 35, 60, 70, 80, 95, 105, 115, 125 };

#define RELATIVE_TO_FORWARD 221  // offset of axle anfle from 180 when hammer forward on floor. actual angle read 221
#define RELATIVE_TO_VERTICAL 32  // offset of axle angle from 90 when hammer arms vertical. actual angle read 122
#define RELATIVE_TO_BACK 25  // offset of axle angle from 0 when hammer back on floor. actual angle read 25

bool weaponsEnabled(){
    return g_enabled;
}

bool autofireEnabled(char bitfield){
    return bitfield & AUTO_HAMMER_ENABLE_BIT;
}

// RETRACT CONSTANTS
#define RETRACT_BEGIN_VEL_MAX 45.0f
static const uint32_t RETRACT_TIMEOUT = 4000 * 1000L;  // in microseconds
static const uint16_t RETRACT_COMPLETE_ANGLE = 10 + RELATIVE_TO_BACK;  // angle read  angle 53 off ground good on 4-09

// HAMMER DATA BUFFERS
#define MAX_DATAPOINTS 500
static uint16_t angle_data[MAX_DATAPOINTS];
static int16_t pressure_data[MAX_DATAPOINTS];

// HAMMER THROW CONSTANTS
// #define THROW_CLOSE_ANGLE_DIFF 3  // angle distance between throw open and throw close
#define VENT_OPEN_ANGLE 175
#define DATA_COLLECT_TIMESTEP 2000  // timestep for data logging, in microseconds
static const uint32_t SWING_TIMEOUT = 1000 * 1000L;  // in microseconds
static const uint16_t THROW_BEGIN_ANGLE_MIN = RELATIVE_TO_BACK - 5;
static const uint16_t THROW_BEGIN_ANGLE_MAX = RELATIVE_TO_BACK + 10;
static const uint16_t THROW_COMPLETE_ANGLE = RELATIVE_TO_FORWARD;

void retract( bool check_velocity ){
    uint16_t angle;
    uint32_t sensor_read_time;
    uint32_t delay_time;
    uint32_t retract_time;

    bool velocity_ok = true;
    if (check_velocity){
      float angular_velocity;
      bool velocity_read_ok = angularVelocity(&angular_velocity);
      velocity_ok = velocity_read_ok && abs(angular_velocity) < RETRACT_BEGIN_VEL_MAX;
    }

    bool angle_read_ok = readAngle(&angle);
    // Only retract if hammer is forward and not moving
    if (weaponsEnabled() && angle_read_ok && angle > RETRACT_COMPLETE_ANGLE && velocity_ok) {
        retract_time = micros();
        // Should already be low, but just in case
        safeDigitalWrite(VENT_VALVE_DO, LOW);
        while (micros() - retract_time < RETRACT_TIMEOUT && angle > RETRACT_COMPLETE_ANGLE) {
            sensor_read_time = micros();
            readAngle(&angle);
            DriveSerial.println("@05!G 100");  // start motor to aid meshing
            safeDigitalWrite(RETRACT_VALVE_DO, HIGH);
            DriveSerial.println("@05!G 1000");
            // Ensure that loop step takes 1 ms or more (without this it takes quite a bit less)
            sensor_read_time = micros() - sensor_read_time;
            delay_time = 10000 - sensor_read_time;
            if (delay_time > 0) {
                delayMicroseconds(delay_time);
            }
        }
        safeDigitalWrite(RETRACT_VALVE_DO, LOW);
        DriveSerial.println("@05!G 0");
    }
}

// Helper to end a swing in case of timeout or hammer obstruction (zero velocity)
void endSwing( bool& throw_open, bool& vent_closed, uint16_t& throw_close_timestep, uint16_t& vent_open_timestep, uint16_t timestep ){
  if (throw_open) {
    throw_close_timestep = timestep;
  }
  safeDigitalWrite(THROW_VALVE_DO, LOW);
  throw_open = false;
  delay(10);
  if (vent_closed) {
    vent_open_timestep = timestep;
  }
  safeDigitalWrite(VENT_VALVE_DO, LOW);
  vent_closed = false;  
}

void fire( uint16_t hammer_intensity, bool flame_pulse, bool mag_pulse ){
    uint32_t fire_time;
    uint32_t swing_length = 0;
    uint32_t sensor_read_time;
    uint16_t throw_close_timestep = 0;
    uint16_t vent_open_timestep = 0;
    uint16_t datapoints_collected = 0;
    uint16_t timestep = 0;
    bool vent_closed = false;
    bool throw_open = false;
    uint32_t delay_time;
    uint16_t angle;
    uint16_t start_angle;
    int16_t pressure;
    bool pressure_read_ok;

    bool angle_read_ok = readAngle(&angle);
    if (weaponsEnabled() && angle_read_ok){
        start_angle = angle;
        // Just in case a bug causes us to fall out of the hammer intensities array, do a last minute
        // sanity check before we actually command a throw.
        uint16_t throw_close_angle_diff = min(MAX_SAFE_ANGLE, HAMMER_INTENSITIES_ANGLE[hammer_intensity]);
        uint16_t throw_close_angle = start_angle + throw_close_angle_diff;
        if (angle > THROW_BEGIN_ANGLE_MIN && angle < THROW_BEGIN_ANGLE_MAX) {
            
            if (mag_pulse){
                magOn();
            }
            if (flame_pulse){
                flameStart();
            }
            // Seal vent (which is normally open)
            safeDigitalWrite(VENT_VALVE_DO, HIGH);
            vent_closed = true;
            // can we actually determine vent close time?
            delay(10);
            
            // Open throw valve
            safeDigitalWrite(THROW_VALVE_DO, HIGH);
            throw_open = true;
            fire_time = micros();
            // Wait until hammer swing complete, up to timeout
            while (swing_length < SWING_TIMEOUT) {
                sensor_read_time = micros();
                angle_read_ok = readAngle(&angle);
                pressure_read_ok = readMlhPressure(&pressure);

                if (throw_open && angle > throw_close_angle) {
                    throw_close_timestep = timestep;
                    safeDigitalWrite(THROW_VALVE_DO, LOW);
                    throw_open = false;
                }
                if (vent_closed && angle > VENT_OPEN_ANGLE) {
                    vent_open_timestep = timestep;
                    safeDigitalWrite(VENT_VALVE_DO, LOW);
                    vent_closed = false;
                }
                if (datapoints_collected < MAX_DATAPOINTS){
                    angle_data[datapoints_collected] = angle;
                    pressure_data[datapoints_collected] = pressure;
                    datapoints_collected++;
                }
                                
                // Once past our throw close angle, start checking velocity 
                if (angle > throw_close_angle) {
                    float angular_velocity;
                    bool velocity_read_ok = angularVelocityBuffered(&angular_velocity, angle_data, datapoints_collected, DATA_COLLECT_TIMESTEP/1000);
                    if (velocity_read_ok && abs(angular_velocity) < RETRACT_BEGIN_VEL_MAX) {
                        // If the swing hasn't already ended, end it now
                        endSwing(throw_open, vent_closed, throw_close_timestep, vent_open_timestep, timestep);
                        // Since our final velocity is low enough, auto-retract
                        retract( /*check_velocity*/ false );
                        break; // exit the while loop
                    }
                }

                // Ensure that loop step takes 1 ms or more (without this it takes quite a bit less)
                sensor_read_time = micros() - sensor_read_time;
                delay_time = DATA_COLLECT_TIMESTEP - sensor_read_time;
                if (delay_time > 0) {
                    delayMicroseconds(delay_time);
                }
                timestep++;
                swing_length = micros() - fire_time;
            } // while
            // If the swing hasn't already ended, end it now
            endSwing(throw_open, vent_closed, throw_close_timestep, vent_open_timestep, timestep);
            
            if (mag_pulse){
                magOff();
            }
            if (flame_pulse) {
                flameEnd();
            }
        } else {
            noAngleFire(/* hammer intensity */1, false, false);
            return;
        }
        
#ifdef HARD_WIRED
        sendSwingData(datapoints_collected,
                      angle_data,
                      pressure_data,
                      DATA_COLLECT_TIMESTEP,
                      throw_close_timestep,
                      vent_open_timestep,
                      throw_close_angle,
                      start_angle);
#endif
    }
}

const uint8_t NO_ANGLE_SWING_DURATION = 185; // total estimated time in ms of a swing (to calculate vent time)
void noAngleFire( uint16_t hammer_intensity, bool flame_pulse, bool mag_pulse ){
    if (weaponsEnabled()){
        if (mag_pulse){
            magOn();
        }
        if (flame_pulse){
            flameStart();
        }
        uint8_t throw_duration = min(MAX_SAFE_TIME, HAMMER_INTENSITIES_TIME[hammer_intensity]);
        // Seal vent valve
        safeDigitalWrite(VENT_VALVE_DO, HIGH);
        // can we actually determine vent close time?
        delay(10);
        
        // Open throw valve
        safeDigitalWrite(THROW_VALVE_DO, HIGH);
        delay(throw_duration);
        safeDigitalWrite(THROW_VALVE_DO, LOW);

        // Wait the estimated remaining time in the swing and then vent
        delay(NO_ANGLE_SWING_DURATION - throw_duration);
        safeDigitalWrite(VENT_VALVE_DO, LOW);

        if (mag_pulse){
            magOff();
        }
        if (flame_pulse) {
            flameEnd();
        }
        
        // Stay in this function for the full second to make sure we're not moving, before we allow the user to possibly retract
        delay( 500 - NO_ANGLE_SWING_DURATION );
    }
}

// use retract motor to gently move hammer to forward position to safe it for approach
#define GENTLE_THROW_BEGIN_ANGLE_MIN 20
#define GENTLE_THROW_STOP_ANGLE 142
#define GENTLE_THROW_COMPLETE_ANGLE 142
#define GENTLE_THROW_TIMEOUT 500000L
void gentleFire(){
    // Make sure we're vented
    safeDigitalWrite(VENT_VALVE_DO, LOW);
    safeDigitalWrite(RETRACT_VALVE_DO, HIGH);
    delay(50);
    DriveSerial.println("@05!G -500");
    uint32_t inter_sbus_time = micros();
    while ((micros() - inter_sbus_time) < 30000UL) {
        if (bufferSbusData()){
            bool error = parseSbus();
            if (!error) {
                wdt_reset();
                char current_rc_bitfield = getRcBitfield();
                if (!(current_rc_bitfield & GENTLE_HAM_F_BIT)){
                    break;
                }
                delay(5);
                DriveSerial.println("@05!G -500");
                inter_sbus_time = micros();
            // continue retracting
            } else {
                break;
            }
        }
    }
    safeDigitalWrite(RETRACT_VALVE_DO, LOW);
    DriveSerial.println("@05!G 0");
}

void gentleRetract( RCBitfield cmd_bit ){
    // Make sure we're vented
    safeDigitalWrite(VENT_VALVE_DO, LOW);
    safeDigitalWrite(RETRACT_VALVE_DO, HIGH);
    delay(50);
    DriveSerial.println("@05!G 1000");
    uint32_t inter_sbus_time = micros();
    while (micros() - inter_sbus_time < 30000UL){
        if (bufferSbusData()){
            bool error = parseSbus();
            if (!error) {
                wdt_reset();
                char current_rc_bitfield = getRcBitfield();
                if (!(current_rc_bitfield & cmd_bit)){
                    break;
                }
                delay(5);
                DriveSerial.println("@05!G 1000");
                inter_sbus_time = micros();
            // continue retracting
            } else {
                break;
            }
      }
    }
    safeDigitalWrite(RETRACT_VALVE_DO, LOW);
    DriveSerial.println("@05!G 0");
}

void flameStart(){
    safeDigitalWrite(PROPANE_DO, HIGH);
}

void flameEnd(){
    // seems like this shouldn't require enable, even though disable should close valve itself
    digitalWrite(PROPANE_DO, LOW);
}

void magOn(){
    if (weaponsEnabled()){
        safeDigitalWrite(MAG1_DO, HIGH);
        safeDigitalWrite(MAG2_DO, HIGH);
    }
}

void magOff(){
    digitalWrite(MAG1_DO, LOW);
    digitalWrite(MAG2_DO, LOW);
}

void valveSafe(){
    // Safing code deliberately does not use safeDigitalWrite since it should always go through.
    digitalWrite(ENABLE_VALVE_DO, LOW);
    digitalWrite(THROW_VALVE_DO, LOW);
    digitalWrite(VENT_VALVE_DO, LOW);
    digitalWrite(RETRACT_VALVE_DO, LOW);
    pinMode(ENABLE_VALVE_DO, OUTPUT);
    pinMode(THROW_VALVE_DO, OUTPUT);
    pinMode(VENT_VALVE_DO, OUTPUT);
    pinMode(RETRACT_VALVE_DO, OUTPUT);
}

void valveEnable(){
    // Assumes safe() has already been called beforehand, to set pin modes.
    safeDigitalWrite(ENABLE_VALVE_DO, HIGH);
}

void flameSafe(){
    digitalWrite(IGNITER_DO, LOW);
    digitalWrite(PROPANE_DO, LOW);
    pinMode(IGNITER_DO, OUTPUT);
    pinMode(PROPANE_DO, OUTPUT);
}

void flameEnable(){
    // Assumes safe() has already been called beforehand, to set pin modes.
    safeDigitalWrite(IGNITER_DO, HIGH);
}
void magnetSafe(){
    digitalWrite(MAG1_DO, LOW);
    digitalWrite(MAG2_DO, LOW);
    pinMode(MAG1_DO, OUTPUT);
    pinMode(MAG2_DO, OUTPUT);
}

