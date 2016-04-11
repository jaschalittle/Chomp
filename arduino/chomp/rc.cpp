// Hook up all the RC interrupts. 
#include "Arduino.h"
#include "rc.h"
#include "pins.h"


// initialize PWM vals to neutral values
static volatile uint16_t LEFT_RC_pwm_val = 1520;
static volatile uint32_t LEFT_RC_prev_time = 0;
static volatile uint16_t RIGHT_RC_pwm_val = 1520;
static volatile uint32_t RIGHT_RC_prev_time = 0;
static volatile uint16_t TARGETING_ENABLE_pwm_val = 1520;
static volatile uint32_t TARGETING_ENABLE_prev_time = 0;

static volatile bool NEW_RC = false;

// values for converting Futaba 7C RC PWM to Roboteq drive control (-1000 to 1000)
// CH1 922-2120 1522 neutral CH2 909-2106 1503 neutral
#define LEFT_PWM_NEUTRAL 1522
#define LEFT_PWM_RANGE 599
#define RIGHT_PWM_NEUTRAL 1503
#define RIGHT_PWM_RANGE 599
// deadband is 60 wide, 5%
#define LEFT_DEADBAND_MIN 1492
#define LEFT_DEADBAND_MAX 1552
#define RIGHT_DEADBAND_MIN 1473
#define RIGHT_DEADBAND_MAX 1533

// values for converting Futaba 9C RC PWM to Roboteq drive control (-1000 to 1000)
// CH1 922-2120 1522 neutral CH2 909-2106 1503 neutral
// #define LEFT_PWM_NEUTRAL 1506
// #define LEFT_PWM_RANGE 598
// #define RIGHT_PWM_NEUTRAL 1530
// #define RIGHT_PWM_RANGE 598
// // deadband is 60 wide, 5%
// #define LEFT_DEADBAND_MIN 1476
// #define LEFT_DEADBAND_MAX 1536
// #define RIGHT_DEADBAND_MIN 1500
// #define RIGHT_DEADBAND_MAX 1560

void LEFT_RC_rising();
void LEFT_RC_falling();
void RIGHT_RC_rising();
void RIGHT_RC_falling();
// void TARGETING_ENABLE_rising();
void TARGETING_ENABLE_falling();

void TARGETING_ENABLE_rising() {
    attachInterrupt(TARGETING_ENABLE, TARGETING_ENABLE_falling, FALLING);
    TARGETING_ENABLE_prev_time = micros();
}
void TARGETING_ENABLE_falling() {
    attachInterrupt(TARGETING_ENABLE, TARGETING_ENABLE_rising, RISING);
    TARGETING_ENABLE_pwm_val = micros() - TARGETING_ENABLE_prev_time;
    NEW_RC = true;
}

// Forgive me, I know not what I do.
#define CREATE_RISING_ISR( rc_interrupt )\
void rc_interrupt ## _rising() {\
  attachInterrupt(rc_interrupt, rc_interrupt ## _falling, FALLING);\
  rc_interrupt ## _prev_time = micros();\
}

#define CREATE_FALLING_ISR( rc_interrupt )\
void rc_interrupt ## _falling() {\
  attachInterrupt(rc_interrupt, rc_interrupt ## _rising, RISING);\
  rc_interrupt ## _pwm_val = micros() - rc_interrupt ## _prev_time;\
}

CREATE_FALLING_ISR(LEFT_RC);
CREATE_RISING_ISR(LEFT_RC);
CREATE_FALLING_ISR(RIGHT_RC);
CREATE_RISING_ISR(RIGHT_RC);

// Set up all RC interrupts
void attachRCInterrupts(){
    pinMode(FUTABA_CH1_PIN, INPUT_PULLUP);
    pinMode(FUTABA_CH2_PIN, INPUT_PULLUP);
    pinMode(FUTABA_CH5_PIN, INPUT_PULLUP);
    attachInterrupt(LEFT_RC, LEFT_RC_rising, RISING);
    attachInterrupt(RIGHT_RC, RIGHT_RC_rising, RISING);
    attachInterrupt(TARGETING_ENABLE, TARGETING_ENABLE_rising, RISING);
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

static uint8_t sbusData[25] = {0};
bool bufferSbusData() {
  // returns number of bytes available for reading from serial receive buffer, which is 64 bytes
  uint16_t count = Sbus.available();
  if (count == 25){
    Sbus.readBytes(sbusData, count);
    return true;
  } else if (count > 25) {
    uint8_t trash[64];
    Sbus.readBytes(trash, count);
    return false;
  }
  return false;
}

static uint16_t sbusChannels [17];  // could initialize this with failsafe values for extra safety
bool parseSbus(){
    bool failsafe = true;
    if (sbusData[0] == 0x0F && sbusData[24] == 0x00) {
        // perverse little endian-ish packet structure-- low bits come in first byte, remaining high bits
        // in next byte
        // NB: chars are promoted to shorts implicitly before bit shift operations
        sbusChannels[0]  = (sbusData[2]  << 8  | sbusData[1])                           & 0x07FF; // 8, 3
        sbusChannels[1]  = (sbusData[3]  << 5  | sbusData[2] >> 3)                      & 0x07FF; // 6, 5
        sbusChannels[2]  = (sbusData[5]  << 10 | sbusData[4] << 2 | sbusData[3] >> 6)   & 0x07FF; // 1, 8, 2
        sbusChannels[3]  = (sbusData[6]  << 7  | sbusData[5] >> 1)                      & 0x07FF; // 4, 7
        sbusChannels[4]  = (sbusData[7]  << 4  | sbusData[6] >> 4)                      & 0x07FF; // 7, 4
        sbusChannels[5]  = (sbusData[9]  << 9  | sbusData[8] << 1 | sbusData[7] >> 7)   & 0x07FF; // 2, 8, 1
        sbusChannels[6]  = (sbusData[10] << 6  | sbusData[9] >> 2)                      & 0x07FF; // 5, 6
        sbusChannels[7]  = (sbusData[11] << 3  | sbusData[10] >> 5)                     & 0x07FF; // 8, 3
        sbusChannels[8]  = (sbusData[13] << 8  | sbusData[12])                          & 0x07FF; // 3, 8
        sbusChannels[9]  = (sbusData[14] << 5  | sbusData[13] >> 3)                     & 0x07FF; // 6, 5
        sbusChannels[10] = (sbusData[16] << 10 | sbusData[15] << 2 | sbusData[14] >> 6) & 0x07FF; // 1, 8, 2
        sbusChannels[11] = (sbusData[17] << 7  | sbusData[16] >> 1)                     & 0x07FF; // 4, 7
        sbusChannels[12] = (sbusData[18] << 4  | sbusData[17] >> 4)                     & 0x07FF; // 7, 4
        sbusChannels[13] = (sbusData[20] << 9  | sbusData[19] << 1 | sbusData[18] >> 7) & 0x07FF; // 2, 8, 1
        sbusChannels[14] = (sbusData[21] << 6  | sbusData[20] >> 2)                     & 0x07FF; // 5, 6
        sbusChannels[15] = (sbusData[22] << 3  | sbusData[21] >> 5)                     & 0x07FF; // 8, 3
        failsafe = sbusData[23] & 0x0008;
    }
    return failsafe;
}

int16_t getLeftRc() {
    int16_t drive_value = 0;
    if (LEFT_RC_pwm_val > LEFT_DEADBAND_MAX || LEFT_RC_pwm_val < LEFT_DEADBAND_MIN) {
        drive_value = ((int16_t) LEFT_RC_pwm_val - LEFT_PWM_NEUTRAL) * 1000L / LEFT_PWM_RANGE;
    }
    // Debug.print(LEFT_RC_pwm_val); Debug.print("\t");
    return drive_value;
}

int16_t getRightRc() {
    int16_t drive_value = 0;
    if (RIGHT_RC_pwm_val > RIGHT_DEADBAND_MAX || RIGHT_RC_pwm_val < RIGHT_DEADBAND_MIN) {
        drive_value = ((int16_t) RIGHT_RC_pwm_val - RIGHT_PWM_NEUTRAL) * 1000L / RIGHT_PWM_RANGE;
    }
    // Debug.print(RIGHT_RC_pwm_val); Debug.println("\t");
    return drive_value;
}

bool getTargetingEnable() {
    // Debug.println(TARGETING_ENABLE_pwm_val);
    return TARGETING_ENABLE_pwm_val > 1700;
}

#define AUTO_HAMMER_THRESHOLD 1000 // (190 down - 1800 up)
#define HAMMER_FIRE_THRESHOLD 1500 // 900 neutral, 170 to 1800
#define HAMMER_RETRACT_THRESHOLD 500
#define FLAME_CTRL_THRESHOLD 500
#define MAG_CTRL_THRESHOLD 1500
#define DANGER_MODE_THRESHOLD 1500

uint8_t getRcBitfield() {
  uint8_t bitfield = 0;
  if ( sbusChannels[AUTO_HAMMER_ENABLE] > AUTO_HAMMER_THRESHOLD){
    bitfield |= AUTO_HAMMER_ENABLE_BIT;
  }
  if ( sbusChannels[HAMMER_CTRL] > HAMMER_FIRE_THRESHOLD){
    bitfield |= HAMMER_FIRE_BIT;
  }
  if ( sbusChannels[HAMMER_CTRL] < HAMMER_RETRACT_THRESHOLD){
    bitfield |= HAMMER_RETRACT_BIT;
  }
  if ( sbusChannels[FLAME_CTRL] > FLAME_CTRL_THRESHOLD){
    bitfield |= FLAME_CTRL_BIT;
  }
  if ( sbusChannels[MAG_CTRL] > MAG_CTRL_THRESHOLD){
    bitfield |= MAG_CTRL_BIT;
  }
  if ( sbusChannels[DANGER_MODE] > DANGER_MODE_THRESHOLD){
    bitfield |= DANGER_CTRL_BIT;
  }
  return bitfield;
}

// WARNING - this function assumes that you have successfully received an SBUS packet!
uint16_t getHammerIntensity(){
  return sbusChannels[INTENSITY];
}

