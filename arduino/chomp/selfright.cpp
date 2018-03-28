#include "Arduino.h"
#include "pins.h"
#include "weapons.h" // weaponsEnabled
#include "utils.h"   // safeDigitalWrite
#include "imu.h"
#include "sensors.h"
#include "telem.h"


extern HardwareSerial& DriveSerial;

void selfRightLeft(){
    if (weaponsEnabled()){
        safeDigitalWrite(SELF_RIGHT_LEFT_DO, HIGH);
        safeDigitalWrite(SELF_RIGHT_RIGHT_DO, LOW);
    }
}


void selfRightRight(){
    if (weaponsEnabled()){
        safeDigitalWrite(SELF_RIGHT_LEFT_DO, LOW);
        safeDigitalWrite(SELF_RIGHT_RIGHT_DO, HIGH);
    }
}


// static void selfRightBoth(){
//     if (weaponsEnabled()){
//         safeDigitalWrite(SELF_RIGHT_LEFT_DO, HIGH);
//         safeDigitalWrite(SELF_RIGHT_RIGHT_DO, HIGH);
//     }
// }


void selfRightOff(){
    digitalWrite(SELF_RIGHT_LEFT_DO, LOW);
    digitalWrite(SELF_RIGHT_RIGHT_DO, LOW);
}


void selfRightSafe(){
    digitalWrite(SELF_RIGHT_LEFT_DO, LOW);
    digitalWrite(SELF_RIGHT_RIGHT_DO, LOW);
    pinMode(SELF_RIGHT_LEFT_DO, OUTPUT);
    pinMode(SELF_RIGHT_RIGHT_DO, OUTPUT);
}


enum SelfRightState {
    UPRIGHT,
    WAIT_HAMMER_FORWARD,
    EXTEND,
    LOCK_OUT,
    WAIT_HAMMER_RETRACT,
    WAIT_UPRIGHT
};

enum Orientation checked_orientation;
static enum SelfRightState self_right_state = UPRIGHT;
String hammer_command;
uint16_t min_hammer_forward_angle = 166;
uint16_t max_hammer_forward_angle = 252;
uint16_t min_hammer_back_angle = 10;
uint16_t max_hammer_back_angle = 20;
uint32_t max_hammer_move_duration = 2000000L;
uint32_t hammer_move_start;
uint32_t max_reorient_duration = 3000000L;
uint32_t reorient_start;


// state tests
static bool hammerIsForward() {
    uint16_t hammer_position = getAngle();
    return (hammer_position > min_hammer_forward_angle &&
            hammer_position < max_hammer_forward_angle);
}


static bool hammerIsRetracted(void) {
    return (getAngle() <= RETRACT_COMPLETE_ANGLE);
 }


static bool hammerIsBack(void) {
    return (min_hammer_back_angle<getAngle() &&
            getAngle() < max_hammer_back_angle);
}

// actions
static void startHammerForward(void) {
    if(hammerIsForward())
        return;

    int16_t hammer_position = getAngle();
    // negative when motion should occur in fire direction,
    // positive when hammer is past min_hammer_angle
    int16_t fire_distance = hammer_position - min_hammer_forward_angle;
    // negative when motion should occur in retract direction,
    // positive when hammer is before max_hammer_angle
    int16_t retract_distance = max_hammer_forward_angle - hammer_position;

    hammer_move_start = micros();
    if(retract_distance<fire_distance) {
        // positive speed moves the hammer in the retract direction
        hammer_command = startElectricHammerMove(1000);
    } else {
        // negative speed moves the hammer in the fire direction
        hammer_command = startElectricHammerMove(-1000);
    }
}


static void startHammerRetract(void)
{
    hammer_move_start = micros();
    hammer_command = startElectricHammerMove(1000);
}


static enum SelfRightState checkHammerRetracted(const enum SelfRightState state)
{
    enum SelfRightState result = state;
    if(hammerIsRetracted()) {
        stopElectricHammerMove();
        result = UPRIGHT;
    } else if((micros() - hammer_move_start)>max_hammer_move_duration) {
        stopElectricHammerMove();
        result = UPRIGHT;
    } else {
        DriveSerial.println(hammer_command);
    }
    return result;
}

static enum SelfRightState doExtend(const enum SelfRightState state)
{
    if(checked_orientation == ORN_LEFT) {
        selfRightLeft();
    } else if(checked_orientation == ORN_RIGHT) {
        selfRightRight();
    }
    reorient_start = micros();
    return WAIT_UPRIGHT;
}


static enum SelfRightState checkOrientation(const enum SelfRightState state)
{
    enum SelfRightState result = state;
    checked_orientation = getOrientation();
    if((checked_orientation == ORN_LEFT) ||
       (checked_orientation == ORN_RIGHT)) {
        if(hammerIsBack()) {
            result = EXTEND;
        } else {
            startHammerForward();
            result = WAIT_HAMMER_FORWARD;
        }
    }
    return result;
}


static enum SelfRightState checkHammerForward(const enum SelfRightState state)
{
    enum SelfRightState result=state;
    if(getOrientation()==ORN_UPRIGHT) {
        startHammerRetract();
        selfRightSafe();
        result = WAIT_HAMMER_RETRACT;
    } else if(hammerIsForward() ||
              (micros() - hammer_move_start > max_hammer_move_duration)) {
        stopElectricHammerMove();
        result = EXTEND;
    } else {
        DriveSerial.println(hammer_command);
    }
    return result;
}


static enum SelfRightState checkUpright(const enum SelfRightState state)
{
    enum SelfRightState result=state;
    if(getOrientation() == ORN_UPRIGHT) {
        selfRightSafe();
        startHammerRetract();
        result = WAIT_HAMMER_RETRACT;
    } else if((micros() - reorient_start) > max_reorient_duration) {
        selfRightSafe();
        result = LOCK_OUT;
    }
    return result;
}


void autoSelfRight(bool enabled) {
    if(!weaponsEnabled() || !enabled) {
        if(self_right_state != UPRIGHT) {
            selfRightSafe();
        }
        if(self_right_state == WAIT_HAMMER_FORWARD ||
           self_right_state == WAIT_HAMMER_RETRACT) {
            stopElectricHammerMove();
        }
        self_right_state = UPRIGHT;
        return;
    }
    switch(self_right_state) {
        case UPRIGHT:
            self_right_state = checkOrientation(self_right_state);
            break;
        case WAIT_HAMMER_FORWARD:
            self_right_state = checkHammerForward(self_right_state);
            break;
        case EXTEND:
            self_right_state = doExtend(self_right_state);
            break;
        case LOCK_OUT:
            if(getOrientation() == ORN_UPRIGHT) {
                self_right_state = UPRIGHT;
            }
            break;
        case WAIT_UPRIGHT:
            self_right_state = checkUpright(self_right_state);
            break;
        case WAIT_HAMMER_RETRACT:
            self_right_state = checkHammerRetracted(self_right_state);
            break;
        default:
            self_right_state = UPRIGHT;
            break;
    }
}


void telemetrySelfRight() {
    sendSelfRightTelem(self_right_state);
}
