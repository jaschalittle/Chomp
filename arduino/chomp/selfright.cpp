#include "Arduino.h"
#include "pins.h"
#include "weapons.h" // weaponsEnabled
#include "utils.h"   // safeDigitalWrite
#include "imu.h"
#include "sensors.h"
#include "telem.h"


extern HardwareSerial& DriveSerial;

static void saveSelfRightParameters();

void selfRightExtendLeft(){
    if (weaponsEnabled()){
        safeDigitalWrite(SELF_RIGHT_LEFT_EXTEND_DO, HIGH);
    }
}

void selfRightRetractLeft(){
    if(weaponsEnabled()){
        safeDigitalWrite(SELF_RIGHT_LEFT_RETRACT_DO, HIGH);
    }
}


void selfRightExtendRight(){
    if (weaponsEnabled()){
        safeDigitalWrite(SELF_RIGHT_RIGHT_EXTEND_DO, HIGH);
    }
}

void selfRightRetractRight(){
    if (weaponsEnabled()){
        safeDigitalWrite(SELF_RIGHT_RIGHT_RETRACT_DO, HIGH);
    }
}

void selfRightExtendBoth(){
     if (weaponsEnabled()){
         safeDigitalWrite(SELF_RIGHT_LEFT_EXTEND_DO, HIGH);
         safeDigitalWrite(SELF_RIGHT_RIGHT_EXTEND_DO, HIGH);
     }
}

void selfRightRetractBoth(){
     if (weaponsEnabled()){
         safeDigitalWrite(SELF_RIGHT_LEFT_RETRACT_DO, HIGH);
         safeDigitalWrite(SELF_RIGHT_RIGHT_RETRACT_DO, HIGH);
     }
}

void selfRightOff(){
    digitalWrite(SELF_RIGHT_LEFT_EXTEND_DO, LOW);
    digitalWrite(SELF_RIGHT_RIGHT_EXTEND_DO, LOW);
    digitalWrite(SELF_RIGHT_LEFT_RETRACT_DO, LOW);
    digitalWrite(SELF_RIGHT_RIGHT_RETRACT_DO, LOW);
}


void selfRightSafe(){
    digitalWrite(SELF_RIGHT_LEFT_EXTEND_DO, LOW);
    digitalWrite(SELF_RIGHT_RIGHT_EXTEND_DO, LOW);
    digitalWrite(SELF_RIGHT_LEFT_RETRACT_DO, LOW);
    digitalWrite(SELF_RIGHT_RIGHT_RETRACT_DO, LOW);
    pinMode(SELF_RIGHT_LEFT_EXTEND_DO, OUTPUT);
    pinMode(SELF_RIGHT_RIGHT_EXTEND_DO, OUTPUT);
    pinMode(SELF_RIGHT_LEFT_RETRACT_DO, OUTPUT);
    pinMode(SELF_RIGHT_RIGHT_RETRACT_DO, OUTPUT);
}


enum SelfRightState {
    UPRIGHT,
    WAIT_HAMMER_POSITIONED,
    EXTEND,
    LOCK_OUT,
    WAIT_HAMMER_RETRACT,
    WAIT_UPRIGHT,
    WAIT_VENT,
    WAIT_LOCKOUT_VENT,
    WAIT_RETRACT,
    WAIT_LOCKOUT_RETRACT
};

enum Orientation checked_orientation;
static enum SelfRightState self_right_state = UPRIGHT;
String hammer_command;
struct SelfRightParams {
    uint16_t min_hammer_self_right_angle;
    uint16_t max_hammer_self_right_angle;
    uint32_t max_hammer_move_duration;
    uint32_t max_reorient_duration;
    uint32_t min_retract_duration;
    uint32_t min_vent_duration;
} __attribute__((packed));

static struct SelfRightParams EEMEM saved_params = {
    .min_hammer_self_right_angle = 30,
    .max_hammer_self_right_angle = 40,
    .max_hammer_move_duration = 2000000L,
    .max_reorient_duration = 3000000L,
    .min_retract_duration = 1000000L,
    .min_vent_duration = 1000000L
};

static struct SelfRightParams params;
uint32_t hammer_move_start;
uint32_t reorient_start;
uint32_t retract_start;
uint32_t extend_vent_start;

void setSelfRightParameters(
        uint16_t p_min_hammer_self_right_angle,
        uint16_t p_max_hammer_self_right_angle,
        uint32_t p_max_hammer_move_duration,
        uint32_t p_max_reorient_duration,
        uint32_t p_min_retract_duration,
        uint32_t p_min_vent_duration
        )
{
     params.min_hammer_self_right_angle = p_min_hammer_self_right_angle;
     params.max_hammer_self_right_angle = p_max_hammer_self_right_angle;
     params.max_hammer_move_duration = p_max_hammer_move_duration;
     params.max_reorient_duration = p_max_reorient_duration;
     params.min_retract_duration = p_min_retract_duration;
     params.min_vent_duration = p_min_vent_duration;
     saveSelfRightParameters();
}


// state tests
static bool hammerIsRetracted(void) {
    return (getAngle() <= RETRACT_COMPLETE_ANGLE);
}

static bool hammerSelfRightPositionAchieved(int16_t min, int16_t max)
{
    return (min < (int16_t)getAngle() && (int16_t)getAngle() < max);
}

// actions
static void startHammerSelfRightPosition(int16_t min, int16_t max) {
    if(hammerSelfRightPositionAchieved(min, max))
        return;

    int16_t hammer_position = getAngle();
    // angle to travel in fire direction to get to min
    int16_t fire_distance = 360 - hammer_position + min;
    // angle to travel in retract direction to get to max
    //int16_t retract_distance = hammer_position - max;

    hammer_move_start = micros();
    if(fire_distance > 0 && fire_distance < 90) {
        // negative speed moves the hammer in the fire direction
        hammer_command = startElectricHammerMove(-1000);
    } else {
        // positive speed moves the hammer in the retract direction
        hammer_command = startElectricHammerMove(1000);
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
        result = WAIT_VENT;
    } else if((micros() - hammer_move_start)>params.max_hammer_move_duration) {
        stopElectricHammerMove();
        result = WAIT_VENT;
    } else {
        DriveSerial.println(hammer_command);
    }
    return result;
}

static enum SelfRightState doExtend(const enum SelfRightState state)
{
    (void)state;
    if((checked_orientation == ORN_LEFT) || (checked_orientation == ORN_TOP_LEFT)) {
        selfRightExtendLeft();
    } else if((checked_orientation == ORN_RIGHT) || (checked_orientation == ORN_TOP_RIGHT)) {
        selfRightExtendRight();
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
        if(hammerSelfRightPositionAchieved(params.min_hammer_self_right_angle,
                                           params.max_hammer_self_right_angle)) {
            result = EXTEND;
        } else {
            startHammerSelfRightPosition(params.min_hammer_self_right_angle,
                                         params.max_hammer_self_right_angle);
            result = WAIT_HAMMER_POSITIONED;
        }
    }
    return result;
}

static enum SelfRightState checkHammerPositioned(const enum SelfRightState state)
{
    enum SelfRightState result=state;
    if(getOrientation()==ORN_UPRIGHT) {
        startHammerRetract();
        selfRightSafe();
        result = WAIT_HAMMER_RETRACT;
    } else if(hammerSelfRightPositionAchieved(params.min_hammer_self_right_angle,
                                              params.max_hammer_self_right_angle) ||
              (micros() - hammer_move_start > params.max_hammer_move_duration)) {
        stopElectricHammerMove();
        safeDigitalWrite(VENT_VALVE_DO, HIGH);
        result = EXTEND;
    } else {
        DriveSerial.println(hammer_command);
    }
    return result;
}

void startBarRetract(void)
{
    if(checked_orientation == ORN_RIGHT)
    {
        selfRightRetractRight();
    }
    else if(checked_orientation == ORN_LEFT)
    {
        selfRightRetractLeft();
    }
    else
    {
        selfRightRetractBoth();
    }
    retract_start = micros();
}

static enum SelfRightState checkUpright(const enum SelfRightState state)
{
    enum SelfRightState result=state;
    if(getOrientation() == ORN_UPRIGHT) {
        // Make sure we're vented
        safeDigitalWrite(VENT_VALVE_DO, LOW);
        selfRightSafe();
        extend_vent_start = micros();
        startHammerRetract();
        result = WAIT_HAMMER_RETRACT;
    } else if((micros() - reorient_start) > params.max_reorient_duration) {
        safeDigitalWrite(VENT_VALVE_DO, LOW);
        selfRightSafe();
        extend_vent_start = micros();
        result = WAIT_LOCKOUT_VENT;
    }
    return result;
}

static enum SelfRightState waitVentExtend(const enum SelfRightState state)
{
    enum SelfRightState result=state;
    if((micros() - extend_vent_start) > params.min_vent_duration)
    {
        if(state == WAIT_VENT)
        {
            result = WAIT_RETRACT;
        }
        else if(state == WAIT_LOCKOUT_VENT)
        {
            result = WAIT_LOCKOUT_RETRACT;
        }
        startBarRetract();
    }
    return result;
}

static enum SelfRightState waitMinimumRetract(const enum SelfRightState state)
{
    enum SelfRightState result=state;
    if((micros() - retract_start) > params.min_retract_duration)
    {
        selfRightSafe();
        if(state == WAIT_RETRACT)
        {
            result = UPRIGHT;
        }
        else if(state == WAIT_LOCKOUT_RETRACT)
        {
            result = LOCK_OUT;
        }
    }
    return result;
}

void autoSelfRight(bool enabled) {
    if(!weaponsEnabled() || !enabled) {
        if(self_right_state != UPRIGHT) {
            selfRightSafe();
        }
        if(self_right_state == WAIT_HAMMER_POSITIONED ||
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
        case WAIT_HAMMER_POSITIONED:
            self_right_state = checkHammerPositioned(self_right_state);
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
        case WAIT_VENT:
        case WAIT_LOCKOUT_VENT:
            self_right_state = waitVentExtend(self_right_state);
            break;
        case WAIT_RETRACT:
        case WAIT_LOCKOUT_RETRACT:
            self_right_state = waitMinimumRetract(self_right_state);
            break;
        default:
            self_right_state = UPRIGHT;
            break;
    }
}

void telemetrySelfRight() {
    sendSelfRightTelem(self_right_state);
}

void saveSelfRightParameters() {
    eeprom_write_block(&params, &saved_params, sizeof(struct SelfRightParams));
}

void restoreSelfRightParameters() {
    eeprom_read_block(&params, &saved_params, sizeof(struct SelfRightParams));
}
