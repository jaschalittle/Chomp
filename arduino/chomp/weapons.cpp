#include "Arduino.h"
#include "weapons.h"
#include "rc.h"
#include "sensors.h"
#include "pins.h"
#include "utils.h"

bool weaponsEnabled(){
    return g_enabled;
}

bool autofireEnabled(char bitfield){
    return bitfield & AUTO_HAMMER_ENABLE_BIT;
}

static const uint32_t RETRACT_TIMEOUT = 300 * 1000;  // microseconds
static const uint16_t RETRACT_BEGIN_ANGLE_MIN = 90;
static const uint16_t RETRACT_BEGIN_VEL_MAX = 5;
static const uint16_t RETRACT_COMPLETE_ANGLE = 10;

void retract(){
    int16_t start_angle;
    bool angle_read_ok = readAngle(&start_angle);
    int16_t angle = start_angle;
    int16_t angular_velocity = 0;
    bool velocity_read_ok = angularVelocity(&angular_velocity);
    uint32_t retract_time;
    // Do we want to check cylinder pressure here?
    // Consider inferring hammer velocity here and requiring that it be below some threshold
    // Only retract if hammer is forward
    if (weaponsEnabled() && angle > RETRACT_BEGIN_ANGLE_MIN && abs(angular_velocity) < RETRACT_BEGIN_VEL_MAX) {
        // Debug.write("Retract\r\n");
        // Open vent
        safeDigitalWrite(VENT_VALVE_DO, LOW);
        delay(10);
        // Open retract valve
        safeDigitalWrite(RETRACT_VALVE_DO, HIGH);
        retract_time = micros();
        angle_read_ok = readAngle(&angle);
        // Keep valve open lesser of 40 degrees from start and 500 ms
        while (micros() - retract_time < RETRACT_TIMEOUT && angle > (start_angle - 40)) {
            angle_read_ok = readAngle(&angle);
        }
        // Close retract valve
        safeDigitalWrite(RETRACT_VALVE_DO, LOW);
        retract_time = micros();
        // Wait until sooner of angle < 10 deg off floor or 300 ms
        while (micros() - retract_time < RETRACT_TIMEOUT && angle > RETRACT_COMPLETE_ANGLE) {
            angle_read_ok = readAngle(&angle);
        }
    }
}

// for collecting hammer swing data
static const uint16_t MAX_DATAPOINTS = 1000;
static int16_t angle_data[MAX_DATAPOINTS];
static int16_t pressure_data[MAX_DATAPOINTS];

// I wanted to multiply to minimize potential for errors, and also seemed like these constants could be outside?
static const uint32_t SWING_TIMEOUT = 5000000;  // microseconds
static const uint32_t DATA_COLLECT_TIMESTEP = 5000;  // timestep for data logging, in microseconds
static const uint16_t THROW_BEGIN_ANGLE_MIN = 5;
static const uint16_t THROW_BEGIN_ANGLE_MAX = 30;
static const uint16_t THROW_CLOSE_ANGLE = 90;
static const uint16_t VENT_OPEN_ANGLE = 160;
static const uint16_t THROW_COMPLETE_ANGLE = 190;
static const uint16_t ACCEL_TIME = 50000;
static const uint16_t THROW_COMPLETE_VELOCITY = 0;

void fire(){
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
    int16_t angle;
    bool angle_read_ok;
    int16_t pressure;
    bool pressure_read_ok;
    int16_t angular_velocity;
    bool velocity_read_ok;
    
    if (weaponsEnabled()){
        bool angle_read_ok = readAngle(&angle);
        // if angle data isn't coming back, abort
        while (!angle_read_ok) {
            uint8_t start_attempts = 1;
            if (start_attempts < 5) {
                angle_read_ok = readAngle(&angle);
                start_attempts++;
            } else {
                return;
            }
        }
        if (angle > THROW_BEGIN_ANGLE_MIN && angle < THROW_BEGIN_ANGLE_MAX) {
            // Debug.write("Fire!\r\n");
            // Seal vent (which is normally open)
            safeDigitalWrite(VENT_VALVE_DO, HIGH);
            vent_closed = true;
            // can we actually determine vent close time?
            delay(10);
            
            // Open throw valve
            safeDigitalWrite(THROW_VALVE_DO, HIGH);
            throw_open = true;
            fire_time = micros();
            // In fighting form, should probably just turn on flamethrower here
            // Wait until hammer swing complete, up to 1 second
            while (swing_length < SWING_TIMEOUT && angle < THROW_COMPLETE_ANGLE) {
                sensor_read_time = micros();
                angle_read_ok = readAngle(&angle);
                pressure_read_ok = readMlhPressure(&pressure);
                // Keep throw valve open until 5 degrees
                if (throw_open && angle > THROW_CLOSE_ANGLE) {
                    throw_close_timestep = timestep;
                    safeDigitalWrite(THROW_VALVE_DO, LOW);
                    throw_open = false;
                }
                if (vent_closed && angle > VENT_OPEN_ANGLE) {
                    vent_open_timestep = timestep;
                    safeDigitalWrite(VENT_VALVE_DO, LOW);
                    vent_closed = false;
                }
                // close throw, open vent if hammer velocity below threshold after 50 ms initial acceleration time
                if (swing_length > ACCEL_TIME) {
                    velocity_read_ok = angularVelocityBuffered(&angular_velocity, angle_data, datapoints_collected);
                    if (velocity_read_ok && abs(angular_velocity) < THROW_COMPLETE_VELOCITY) {
                        if (throw_open) {throw_close_timestep = timestep;}
                        safeDigitalWrite(THROW_VALVE_DO, LOW);
                        throw_open = false;
                        delay(10);
                        if (vent_closed) {vent_open_timestep = timestep;}
                        safeDigitalWrite(VENT_VALVE_DO, LOW);
                        vent_closed = false;
                    }
                }
                if (datapoints_collected < MAX_DATAPOINTS){
                    angle_data[datapoints_collected] = angle;
                    pressure_data[datapoints_collected] = pressure;
                    datapoints_collected++;
                }
                // Ensure that loop step takes 1 ms or more (without this it takes quite a bit less)
                sensor_read_time = micros() - sensor_read_time;
                delay_time = DATA_COLLECT_TIMESTEP - sensor_read_time;
                if (delay_time > 0) {
                    delayMicroseconds(delay_time);
                }
                timestep++;
                swing_length = micros() - fire_time;
            }
            // Close throw valve after 1 second even if target angle not achieved
            throw_close_timestep = timestep;
            safeDigitalWrite(THROW_VALVE_DO, LOW);
            delay(10);
            // Open vent valve after 1 second even if target angle not achieved
            vent_open_timestep = timestep;
            safeDigitalWrite(VENT_VALVE_DO, LOW);
            
            // Send buffered throw data over serial
            for (uint16_t i = 0; i < datapoints_collected; i++) {
                Debug.print(angle_data[i], DEC);
                Debug.print("\t");
                Debug.println(pressure_data[i]);
            }
            Debug.print("timestep\t");
            Debug.println(DATA_COLLECT_TIMESTEP);
            Debug.print("throw_close_timestep\t");
            Debug.println(throw_close_timestep);
            Debug.print("vent_open_timestep\t");
            Debug.println(vent_open_timestep);
        }
    }
}

void flameStart(){
    if (weaponsEnabled()){
        safeDigitalWrite(PROPANE_DO, HIGH);
    }
}

void flameEnd(){
    // seems like this shouldn't require enable, even though change to disable should close valve itself
    digitalWrite(PROPANE_DO, LOW);
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

