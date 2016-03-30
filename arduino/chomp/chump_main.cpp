#include "Arduino.h"
#include "chump_main.h"
#include "rc.h"
#include "leddar_io.h"
#include "autofire.h"
#include "chump_targeting.h"
#include "sensors.h"
#include "xbee.h"
#include "drive.h"
#include "telem.h"
#include "pins.h"
#include "weapons.h"

// for correlating drive control to speed
void printMiddleDistance(unsigned int num_detections, Detection* detections) {
    int segment7_dist = 10000;
    int segment8_dist = 10000;
    for (uint8_t i = 0; i < num_detections; i++) {
        if (detections[i].Segment == 7 && detections[i].Distance < segment7_dist) {
            segment7_dist = detections[i].Distance;
        } else if (detections[i].Segment == 8 && detections[i].Distance < segment8_dist) {
            segment8_dist = detections[i].Distance;
        }
    }
    if (segment7_dist == 10000 && segment8_dist == 10000) {
        Xbee.print("n/a");
        Xbee.print("\t");
    } else if (segment7_dist == 10000) {
        Xbee.print(segment8_dist);
        Xbee.print("\t");
    } else if (segment8_dist == 10000) {
        Xbee.print(segment7_dist);
        Xbee.print("\t");
    } else {
        Xbee.print((segment7_dist + segment8_dist) / 2);
        Xbee.print("\t");
    }
}

void chumpSetup() {
    // canSetup();
    leddarWrapperInit();
    attachRCInterrupts();
    requestDetections();
    Debug.begin(115200);
    Sbus.begin(100000);
    Sbus.setTimeout(10);
    DriveSerial.begin(115200);
    // xbeeInit();
    valveEnable();
}

static int previous_leddar_state = FAR_ZONE;
static char previous_rc_bitfield = 0;
static unsigned long last_request_time = micros();
static unsigned long last_telem_time = micros();
static int16_t left_drive_value = 0;
static int16_t right_drive_value = 0;
static bool targeting_enabled = false;
static int16_t steer_bias = 0; // positive turns right, negative turns left
unsigned long last_loop_begin = micros();

void chumpLoop() {
    // Debug.println(micros() - last_loop_begin);
    // last_loop_begin = micros();

    unsigned long start_time = micros();
    if (micros() - last_request_time > 1000000) {
        last_request_time = micros();
        requestDetections();
    }

    // check if there is new Leddar data
    bool complete = bufferDetections();
    if (complete) {
        unsigned int detection_count = parseDetections();
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
    steer_bias = pidSteer(detection_count, getDetections(), 600);   // 600 cm ~ 20 ft

    // code for correlating distance to RC input
    // if (targeting_enabled && left_drive_value <= 600) {
        // left_drive_value += 50;
        // right_drive_value += 50;
        // printMiddleDistance(detection_count, getDetections());
        // Debug.print(left_drive_value);
        // Debug.print("\t");
        // Debug.print(right_drive_value);
        // Debug.println();
    // }
    }

    // should this be renamed to weapons_rc for clarity?
    if (bufferSbusData()){
        parseSbus();
        // React to RC state changes
        char current_rc_bitfield = getRcBitfield();
        char diff = previous_rc_bitfield ^ current_rc_bitfield;
        if (diff) {
            // Flame on -> off
            // if (diff & FLAME_CTRL_BIT & previous_rc_bitfield) {
            if( (diff & FLAME_CTRL_BIT) && !(current_rc_bitfield & FLAME_CTRL_BIT) ){
                // flameEnd();
            }
            // Flame off -> on
            // if (diff & FLAME_CTRL_BIT & current_rc_bitfield) {
            if( (diff & FLAME_CTRL_BIT) && (current_rc_bitfield & FLAME_CTRL_BIT) ){
                // flameStart(current_rc_bitfield);
            }
            // Manual hammer fire
            // if (diff & HAMMER_FIRE_BIT & current_rc_bitfield) {
            if( (diff & HAMMER_FIRE_BIT) && (current_rc_bitfield & HAMMER_FIRE_BIT)){
                fire();
            }
            // if (diff & HAMMER_RETRACT_BIT & current_rc_bitfield) {
            if( (diff & HAMMER_RETRACT_BIT) && (current_rc_bitfield & HAMMER_RETRACT_BIT)){
                retract();
            }
        }
        previous_rc_bitfield = current_rc_bitfield;
    }

    // left_drive_value = getLeftRc();
    // right_drive_value = getRightRc();
    drive(left_drive_value - steer_bias, right_drive_value - steer_bias);

    targeting_enabled = getTargetingEnable();
    if (targeting_enabled) {
        drive(left_drive_value - steer_bias, right_drive_value - steer_bias);
        // Debug.print(steer_bias);
        // Debug.println();
    }
}
