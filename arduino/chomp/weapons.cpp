#include "Arduino.h"
#include "weapons.h"
#include "rc.h"
#include "sensors.h"
#include "pins.h"
#include "utils.h"

// for collecting hammer swing data
static const int MAX_DATAPOINTS = 1000;
static int16_t angle_data[MAX_DATAPOINTS];
static int16_t pressure_data[MAX_DATAPOINTS];

bool weaponsEnabled(){
    return g_enabled;
}

bool autofireEnabled(char bitfield){
    return bitfield & AUTO_HAMMER_ENABLE_BIT;
}

void retract(char bitfield){
    float start_angle = readAngle();
    float angle = start_angle;
    float angular_velocity = angularVelocity();
    // Do we want to check cylinder pressure here?
    // Consider inferring hammer velocity here and requiring that it be below some threshold
    // Only retract if hammer is forward
    if (weaponsEnabled() && angle > 90.0 && angular_velocity < 5.0) {
        // Debug.write("Retract\r\n");
        // Open vent
        safeDigitalWrite(VENT_VALVE_DO, LOW);
        delay(10);
        // Open retract valve
        safeDigitalWrite(RETRACT_VALVE_DO, HIGH);
        long retract_time = micros();
        angle = readAngle();
        // Keep valve open lesser of 40 degrees from start and 500 ms
        while (micros() - retract_time < 500000 && angle > (start_angle - 40)) {
            angle = readAngle();
        }
        // Close retract valve
        safeDigitalWrite(RETRACT_VALVE_DO, LOW);
        retract_time = micros();
        // Wait until sooner of angle < 10 deg off floor or 300 ms
        while (micros() - retract_time < 300000 && angle > 10.0) {
            angle = readAngle();
        }
    }
}

void fire(char bitfield){
    if (weaponsEnabled()){
        float angle = readAngle();
        if (angle < 10.0) {
            // In fighting form, should probably just turn on flamethrower here
            // Debug.write("Fire!\r\n");
            // Seal vent (which is normally closed)
            safeDigitalWrite(VENT_VALVE_DO, HIGH);
            bool vent_closed = true;
            delay(10);
            // Open throw valve
            safeDigitalWrite(THROW_VALVE_DO, HIGH);
            bool throw_open = true;
            long fire_time = micros();
            long read_time;
            int throw_close_time;
            int vent_open_time;
            float pressure;
            int datapoints_collected = 0;
            // Wait until hammer swing complete, up to 1 second
            while ((micros() - fire_time) < 1000000 && angle < 190.0) {
                read_time = micros();
                angle = readAngle();
                pressure = readMlhPressure();
                // Keep throw valve open until 5 degrees
                if (throw_open && angle > 5.0) {
                    throw_close_time = datapoints_collected;
                    safeDigitalWrite(THROW_VALVE_DO, LOW);
                    throw_open = false;
                }
                if (vent_closed && angle > 160.0) {
                    vent_open_time = datapoints_collected;
                    safeDigitalWrite(VENT_VALVE_DO, LOW);
                    vent_closed = false;
                }
                if (datapoints_collected < MAX_DATAPOINTS){
                  angle_data[datapoints_collected] = angle;
                  pressure_data[datapoints_collected] = pressure;
                  datapoints_collected++;
                }
                // Ensure that loop step takes 1 ms or more (without this it takes quite a bit less)
                read_time = micros() - read_time;
                int delay_time = 1000 - read_time;
                if (delay_time > 0) {
                    delayMicroseconds(delay_time);
                }
            }
            // Close throw valve after 1 second even if target angle not achieved
            safeDigitalWrite(THROW_VALVE_DO, LOW);
            delay(10);
            // Open vent valve after 1 second even if target angle not achieved
            safeDigitalWrite(VENT_VALVE_DO, LOW);
            
            // Send buffered throw data over serial
            for (int i = 0; i < datapoints_collected; i++) {
                Debug.print(angle_data[i], DEC);
                Debug.print("\t");
                Debug.println(pressure_data[i]);
            }
            Debug.print("throw_close_time\t");
            Debug.println(throw_close_time);
            Debug.print("vent_open_time\t");
            Debug.println(vent_open_time);
        }
    }
}

void valveReset(){
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

void valveSetup() {
    valveReset();
    pinMode(ANGLE_AI, INPUT);
    safeDigitalWrite(ENABLE_VALVE_DO, HIGH);
    safeDigitalWrite(VENT_VALVE_DO, LOW);
}

void flameStart(char bitfield){
    if (weaponsEnabled()){
        // Debug.write("Flame!\r\n");
    }
}

void flameEnd(){
    // Debug.write("Flame off\r\n");
}
