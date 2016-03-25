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


// for collecting hammer swing data
const int datapoints = 1000;
int16_t angle_data[datapoints];
int16_t pressure_data[datapoints];

bool weaponsEnabled(char bitfield){
    return bitfield & WEAPONS_ENABLE_BIT;
}

bool autofireEnabled(char bitfield){
    return bitfield & AUTO_HAMMER_ENABLE_BIT;
}

float angularVelocity () {
    // This function could filter low angle values and ignore them for summing. If we only rail to 0V, we could still get a velocity.
    float angle_traversed = 0.0;
    float last_angle = readAngle();
    long read_time = micros();
    // Take 50 readings. This should be 5-10 ms.
    for (int i = 0; i < 50; i++) {
        float new_angle = readAngle();
        angle_traversed += abs(new_angle - last_angle);
        last_angle = new_angle;
    }
    read_time = micros() - read_time / 1000;    // convert to milliseconds
    float angular_velocity = angle_traversed / read_time * 1000; // degrees per second
}

void retract(char bitfield){
    float angle = readAngle();
    float angular_velocity = angularVelocity();
    // Do we want to check cylinder pressure here?
    // Consider inferring hammer velocity here and requiring that it be below some threshold
    // Only retract if hammer is forward
    if (weaponsEnabled(bitfield) && angle > 90.0 && angular_velocity < 5.0) {
        // Debug.write("Retract\r\n");
        // Open vent
        digitalWrite(VENT_VALVE_DO, HIGH);
        delay(10);
        // Open retract valve
        digitalWrite(RETRACT_VALVE_DO, HIGH);
        long retract_time = micros();
        angle = readAngle();
        // Keep valve open lesser of 40 degrees and 500 ms
        while (micros() - retract_time < 500000 && angle > 140.0) {
            angle = readAngle();
        }
        // Close retract valve
        digitalWrite(RETRACT_VALVE_DO, LOW);
        retract_time = micros();
        // Wait until sooner of angle < 10 deg off floor or 300 ms
        while (micros() - retract_time < 300000 && angle > 10.0) {
            angle = readAngle();
        }
    }
}

void fire(char bitfield){
    if (weaponsEnabled(bitfield)){
        float angle = readAngle();
        if (angle < 10.0) {
            // In fighting form, should probably just turn on flamethrower here
            // Debug.write("Fire!\r\n");
            // Seal vent (which is normally closed)
            digitalWrite(VENT_VALVE_DO, LOW);
            bool vent_closed = true;
            delay(10);
            // Open throw valve
            digitalWrite(THROW_VALVE_DO, HIGH);
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
                    digitalWrite(THROW_VALVE_DO, LOW);
                    throw_open = false;
                }
                if (vent_closed && angle > 160.0) {
                    vent_open_time = datapoints_collected;
                    digitalWrite(VENT_VALVE_DO, HIGH);
                    vent_closed = false;
                }
                angle_data[datapoints_collected] = angle;
                pressure_data[datapoints_collected] = pressure;
                datapoints_collected++;
                // Ensure that loop step takes 1 ms or more (without this it takes quite a bit less)
                read_time = micros() - read_time;
                int delay_time = 1000 - read_time;
                if (delay_time > 0) {
                    delayMicroseconds(delay_time);
                }
            }
            // Close throw valve after 1 second even if target angle not achieved
            digitalWrite(THROW_VALVE_DO, LOW);
            delay(10);
            // Open vent valve after 1 second even if target angle not achieved
            digitalWrite(VENT_VALVE_DO, HIGH);
            
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

void flameStart(char bitfield){
    if (weaponsEnabled(bitfield)){
        // Debug.write("Flame!\r\n");
    }
}

void flameEnd(){
    // Debug.write("Flame off\r\n");
}

void valveReset(){
    digitalWrite(ENABLE_VALVE_DO, LOW);
    digitalWrite(THROW_VALVE_DO, LOW);
    digitalWrite(VENT_VALVE_DO, LOW);
    digitalWrite(RETRACT_VALVE_DO, LOW);
}

void valveSetup() {
    valveReset();
    pinMode(ENABLE_VALVE_DO, OUTPUT);
    pinMode(THROW_VALVE_DO, OUTPUT);
    pinMode(VENT_VALVE_DO, OUTPUT);
    pinMode(RETRACT_VALVE_DO, OUTPUT);
    pinMode(ANGLE_AI, INPUT);
    digitalWrite(ENABLE_VALVE_DO, HIGH);
    digitalWrite(VENT_VALVE_DO, HIGH);
}

void chompSetup() {
    // xbeeInit();
    Debug.begin(115200);
    Sbus.begin(100000);
    leddarWrapperInit();
    attachRCInterrupts();
    valveSetup();
    pinMode(GREEN, OUTPUT);
    pinMode(RED, OUTPUT);
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
            
            // Global enable -> disable
            // if (diff & WEAPONS_ENABLE_BIT & previous_rc_bitfield) {
            if( (diff & WEAPONS_ENABLE_BIT) && !(current_rc_bitfield & WEAPONS_ENABLE_BIT)){
                // TODO: Do we want to shut enable valve here too?
                flameEnd();
            }
            // Flame on -> off
            // if (diff & FLAME_CTRL_BIT & previous_rc_bitfield) {
            if( (diff & FLAME_CTRL_BIT) && !(current_rc_bitfield & FLAME_CTRL_BIT) ){
                flameEnd();
            }
            // Flame off -> on
            // if (diff & FLAME_CTRL_BIT & current_rc_bitfield) {
            if( (diff & FLAME_CTRL_BIT) && (current_rc_bitfield & FLAME_CTRL_BIT) ){
                flameStart(current_rc_bitfield);
            }
            // Manual hammer fire
            // if (diff & HAMMER_FIRE_BIT & current_rc_bitfield) {
            if( (diff & HAMMER_FIRE_BIT) && (current_rc_bitfield & HAMMER_FIRE_BIT)){
                fire(current_rc_bitfield); // checks enable internally
            }
            // if (diff & HAMMER_RETRACT_BIT & current_rc_bitfield) {
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
