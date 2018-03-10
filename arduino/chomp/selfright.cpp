#include "Arduino.h"
#include "pins.h"
#include "weapons.h" // weaponsEnabled
#include "utils.h"   // safeDigitalWrite
#include "imu.h"
#include "sensors.h"
#include "telem.h"


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


/* static void selfRightBoth(){
    if (weaponsEnabled()){
        safeDigitalWrite(SELF_RIGHT_LEFT_DO, HIGH);
        safeDigitalWrite(SELF_RIGHT_RIGHT_DO, HIGH);
    }
}*/


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
    MOVE_HAMMER_FORWARD,
    WAIT_STABLE,
    LOCK_OUT,
    HAMMER_RETRACT,
    WAIT_UPRIGHT
};

static enum SelfRightState self_right_state = UPRIGHT;
String hammer_command;
uint16_t min_hammer_angle = 166;
uint16_t max_hammer_angle = 252;
uint32_t max_hammer_move_duration = 2000000L;
uint32_t hammer_move_start;
uint32_t max_reorient_duration = 5000000L;
uint32_t reorient_start;


// state tests
static bool hammerIsForward() {
    uint16_t hammer_position = getAngle();
    return (hammer_position > min_hammer_angle &&
            hammer_position < max_hammer_angle);
}


static bool hammerIsRetracted(void) {
    return (getAngle() <= RETRACT_COMPLETE_ANGLE);
}


// actions
static void startHammerForward(void) {
    if(hammerIsForward())
        return;

    int16_t hammer_position = getAngle();
    // negative when motion should occur in fire direction,
    // positive when hammer is past min_hammer_angle
    int16_t fire_distance = hammer_position - min_hammer_angle;
    // negative when motion should occur in retract direction,
    // positive when hammer is before max_hammer_angle
    int16_t retract_distance = max_hammer_angle - hammer_position;

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


// state functions
static enum SelfRightState checkOrientation(const enum SelfRightState state) {
    enum SelfRightState result=state;
    switch(getOrientation()) {
        case ORN_LEFT:
        case ORN_RIGHT:
        case ORN_TOP_LEFT:
        case ORN_TOP_RIGHT:
        case ORN_TOP:
        case ORN_FRONT:
        case ORN_TAIL:
            startHammerForward();
            result = MOVE_HAMMER_FORWARD;
            break;
        case ORN_UPRIGHT:
        case ORN_UNKNOWN:
        default:
            break;
    }
    return result;
}


static enum SelfRightState checkHammerForward(const enum SelfRightState state)
{
    enum SelfRightState result=state;
    if(hammerIsForward()) {
        stopElectricHammerMove();
        result = WAIT_STABLE;
    } else if(micros() - hammer_move_start > max_hammer_move_duration) {
        stopElectricHammerMove();
        selfRightOff();
        result = LOCK_OUT;
    } else {
        DriveSerial.println(hammer_command);
    }
    return result;
}


static enum SelfRightState checkStable(const enum SelfRightState state)
{
    enum SelfRightState result=state;
    switch(getOrientation()) {
        case ORN_UPRIGHT:
            startHammerRetract();
            result = HAMMER_RETRACT;
            break;
        case ORN_LEFT:
        case ORN_TOP_LEFT:
            selfRightLeft();
            reorient_start = micros();
            result = WAIT_UPRIGHT;
            break;
        case ORN_RIGHT:
        case ORN_TOP_RIGHT:
            selfRightRight();
            reorient_start = micros();
            result = WAIT_UPRIGHT;
            break;
        case ORN_TOP:
        case ORN_FRONT:
        case ORN_TAIL:
            selfRightOff();
            result = LOCK_OUT;
            break;
        case ORN_UNKNOWN:
        default:
            break;
    }
 
    return result;
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


static enum SelfRightState checkUpright(const enum SelfRightState state)
{
    enum SelfRightState result = state;
    if(getOrientation() == ORN_UPRIGHT) {
        selfRightOff();
        startHammerRetract();
        result = HAMMER_RETRACT;
    } else if((micros() - reorient_start)>max_reorient_duration) {
        selfRightOff();
        result = LOCK_OUT;
    }
    return result;
}


void autoSelfRight(void) {
    if(!weaponsEnabled()) {
        selfRightSafe();
        if(self_right_state == MOVE_HAMMER_FORWARD ||
           self_right_state == HAMMER_RETRACT) {
            stopElectricHammerMove();
        }
        self_right_state = UPRIGHT;
        return;
    }
    switch(self_right_state) {
        case UPRIGHT:
            self_right_state = checkOrientation(self_right_state);
            break;
        case MOVE_HAMMER_FORWARD:
            self_right_state = checkHammerForward(self_right_state);
            break;
        case WAIT_STABLE:
            self_right_state = checkStable(self_right_state);
            break;
        case LOCK_OUT:
            if(getOrientation() == ORN_UPRIGHT) {
                self_right_state = UPRIGHT;
            }
            break;
        case WAIT_UPRIGHT:
            self_right_state = checkUpright(self_right_state);
            break;
        case HAMMER_RETRACT:
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
