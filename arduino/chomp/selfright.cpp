#include "Arduino.h"
#include "pins.h"
#include "weapons.h" // weaponsEnabled
#include "utils.h"   // safeDigitalWrite
#include "imu.h"
#include "sensors.h"

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

static enum SelfRightState state = UPRIGHT;
String hammer_command;
uint16_t min_hammer_angle = 166;
uint16_t max_hammer_angle = 252;
uint32_t max_hammer_move_duration = 2000000L;
uint32_t hammer_move_start;

static bool hammerIsForward() {
    uint16_t hammer_position = getAngle();
    return (hammer_position > min_hammer_angle &&
            hammer_position < max_hammer_angle);
}

static void hammerForward(void) {
    if(hammerIsForward())
        return;

    int16_t hammer_position = getAngle();
    int16_t forward_distance = abs(hammer_position - min_hammer_angle);
    int16_t reverse_distance = abs(hammer_position - max_hammer_angle);

    hammer_move_start = micros();
    if(reverse_distance<forward_distance) {
        // positive speed moves the hammer in the retract direction
        hammer_command = startElectricHammerMove(1000);
    } else {
        // negative speed moves the hammer in the fire direction
        hammer_command = startElectricHammerMove(-1000);
    }
}


static enum SelfRightState checkOrientation(void) {
    enum SelfRightState result=UPRIGHT;
    switch(getOrientation()) {
        case ORN_LEFT:
        case ORN_RIGHT:
        case ORN_TOP_LEFT:
        case ORN_TOP_RIGHT:
        case ORN_TOP:
        case ORN_FRONT:
        case ORN_TAIL:
            hammerForward();
            result = MOVE_HAMMER_FORWARD;
            break;
        case ORN_UPRIGHT:
        case ORN_UNKNOWN:
        default:
            break;
    }
    return result;
}

static enum SelfRightState checkHammerForward(void)
{
    enum SelfRightState result=UPRIGHT;
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


static void startHammerRetract(void)
{
    hammer_move_start = micros();
    hammer_command = startElectricHammerMove(-1000);
}

static enum SelfRightState checkStable(void)
{
    enum SelfRightState result=UPRIGHT;
    switch(getOrientation()) {
        case ORN_UPRIGHT:
            startHammerRetract();
            result = HAMMER_RETRACT;
            break;
        case ORN_LEFT:
        case ORN_TOP_LEFT:
            selfRightLeft();
            result = WAIT_UPRIGHT;
            break;
        case ORN_RIGHT:
        case ORN_TOP_RIGHT:
            selfRightRight();
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


static bool hammerIsRetracted(void) {
    return (((micros() - hammer_move_start) > max_hammer_move_duration) ||
            (getAngle() <= RETRACT_COMPLETE_ANGLE));
}

void autoSelfRight(void) {
    if(!weaponsEnabled()) {
        selfRightSafe();
        return;
    }
    switch(state) {
        case UPRIGHT:
            state = checkOrientation();
            break;
        case MOVE_HAMMER_FORWARD:
            state = checkHammerForward();
            break;
        case WAIT_STABLE:
            state = checkStable();
            break;
        case LOCK_OUT:
            if(getOrientation() == ORN_UPRIGHT) {
                state = UPRIGHT;
            }
            break;
        case WAIT_UPRIGHT:
            if(getOrientation() == ORN_UPRIGHT) {
                selfRightOff();
                startHammerRetract();
                state = HAMMER_RETRACT;
            }
            break;
        case HAMMER_RETRACT:
            if(hammerIsRetracted()) {
                stopElectricHammerMove();
                state = UPRIGHT;
            }
            break;
        default:
            state = UPRIGHT;
            break;
    }
}
