// Hook up all the RC interrupts. 
#include "Arduino.h"
#include "rc_pwm.h"
#include "pins.h"
#include "weapons.h"
#include "utils.h"
#include <util/atomic.h>

enum RCinterrupts {
    DRIVE_DISTANCE_int = digitalPinToInterrupt(DRIVE_DISTANCE_PIN),
    TARGETING_ENABLE_int = digitalPinToInterrupt(TARGETING_ENABLE_PIN),
};


static void attachRCInterrupts();
// values for converting Futaba 7C RC PWM to Roboteq drive control (-1000 to 1000)
// CH1 LEFT 1116-1932 1522 neutral CH2 RIGHT 1100-1920 1512 neutral
// Note - left and right rc have different polarities! When you throttle forwards,
// expect to see left rc go low and right rc go high.

// These constants are copied from the Roboteq manual
// https://www.roboteq.com/index.php/docman/motor-controllers-documents-and-files/documentation/user-manual/272-roboteq-controllers-user-manual-v17/file
// page 76
// We want to convert from PWM->percent speed with the same coefficients
// as the Roboteq so driving behaviour really doesn't depend on command source
#define LEFT_PWM_NEUTRAL 1500
#define LEFT_PWM_RANGE 500
#define RIGHT_PWM_NEUTRAL 1500
#define RIGHT_PWM_RANGE 500
// deadband is 50 wide, 5% either way of zero
#define LEFT_DEADBAND_MIN 1475
#define LEFT_DEADBAND_MAX 1525
#define RIGHT_DEADBAND_MIN 1475
#define RIGHT_DEADBAND_MAX 1525

// initialize PWM vals to neutral values
static volatile uint16_t LEFT_RC_pwm_val = 1500;
static volatile uint32_t LEFT_RC_prev_time = 0;
static volatile uint16_t RIGHT_RC_pwm_val = 1500;
static volatile uint32_t RIGHT_RC_prev_time = 0;
static volatile uint16_t TARGETING_ENABLE_pwm_val = 1500;
static volatile uint32_t TARGETING_ENABLE_prev_time = 0;
static volatile int DRIVE_DISTANCE_pwm_val = 1500;
static volatile int DRIVE_DISTANCE_prev_time = 0;

static volatile uint8_t PBLAST = 0; // 0 so that we detect rising interrupts first.
static volatile bool NEW_RC = false;

ISR(PCINT0_vect) {
    uint8_t PBNOW = PINB ^ PBLAST;
    PBLAST = PINB;
    uint8_t left_rc_bit = 1 << PINB6;
    uint8_t right_rc_bit = 1 << PINB7;
    
    // These can come in simultaneously so don't make this an if/else.
    if (PBNOW & left_rc_bit) {
        if (PINB & left_rc_bit) { // Rising
            LEFT_RC_prev_time = micros();
        }
        else {
            LEFT_RC_pwm_val = micros() - LEFT_RC_prev_time;
        }
    }
    if (PBNOW & right_rc_bit) {
        if (PINB & right_rc_bit) { // Rising
            RIGHT_RC_prev_time = micros();
        }
        else {
            RIGHT_RC_pwm_val = micros() - RIGHT_RC_prev_time;
        }
    }
}

static void TARGETING_ENABLE_falling(); // forward decl
static void TARGETING_ENABLE_rising() {
    attachInterrupt(TARGETING_ENABLE_int, TARGETING_ENABLE_falling, FALLING);
    TARGETING_ENABLE_prev_time = micros();
}
void TARGETING_ENABLE_falling() {
    attachInterrupt(TARGETING_ENABLE_int, TARGETING_ENABLE_rising, RISING);
    TARGETING_ENABLE_pwm_val = micros() - TARGETING_ENABLE_prev_time;
    NEW_RC = true;
}

static void DRIVE_DISTANCE_Falling();
static void DRIVE_DISTANCE_Rising(){
    attachInterrupt(DRIVE_DISTANCE_int, DRIVE_DISTANCE_Falling, FALLING );
    DRIVE_DISTANCE_prev_time = micros();
}
static void DRIVE_DISTANCE_Falling(){
    attachInterrupt(DRIVE_DISTANCE_int, DRIVE_DISTANCE_Rising, RISING);
    DRIVE_DISTANCE_pwm_val = micros() - DRIVE_DISTANCE_prev_time;
}

void rcInit() {
    attachRCInterrupts();
    attachInterrupt(DRIVE_DISTANCE_int, DRIVE_DISTANCE_Rising, RISING);
}

// Set up all RC interrupts
static void attachRCInterrupts(){
    pinMode(LEFT_RC_PIN, INPUT_PULLUP);
    pinMode(RIGHT_RC_PIN, INPUT_PULLUP);
    pinMode(TARGETING_ENABLE_PIN, INPUT_PULLUP);

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        attachInterrupt(TARGETING_ENABLE_int, TARGETING_ENABLE_rising, RISING);

        PCICR |= 0b00000001; // Enables Ports B Pin Change Interrupts
        PCMSK0 |= 0b11000000; // Mask interrupts to PCINT6 and PCINT7
    }
}

// test whether there is new unused RC drive command
bool newRc() {
    if (NEW_RC) {
        NEW_RC = false;
        return true;
    } else {
        return false;
    }
}

int16_t getLeftRc() {
    int16_t drive_value = 0;
    if (LEFT_RC_pwm_val > LEFT_DEADBAND_MAX || LEFT_RC_pwm_val < LEFT_DEADBAND_MIN) {
        drive_value = ((int16_t) LEFT_RC_pwm_val - LEFT_PWM_NEUTRAL) * 1000L / LEFT_PWM_RANGE;
    }
    return drive_value;
}

int16_t getRightRc() {
    int16_t drive_value = 0;
    if (RIGHT_RC_pwm_val > RIGHT_DEADBAND_MAX || RIGHT_RC_pwm_val < RIGHT_DEADBAND_MIN) {
        drive_value = ((int16_t) RIGHT_RC_pwm_val - RIGHT_PWM_NEUTRAL) * 1000L / RIGHT_PWM_RANGE;
    }
    return drive_value;
}

bool getTargetingEnable() {
    return TARGETING_ENABLE_pwm_val > 1700;
}

#define DRIVE_DISTANCE_PWM_MIN 1000
static int16_t max_drive_range=2000;
static int16_t min_drive_range=500;
static int16_t drive_range_scale=2;
int16_t getDriveDistance() {
    return clip((DRIVE_DISTANCE_pwm_val - DRIVE_DISTANCE_PWM_MIN)*drive_range_scale,
                0, max_drive_range) + min_drive_range;
}

