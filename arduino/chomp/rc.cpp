// Hook up all the RC interrupts. 
#include "Arduino.h"
#include "rc.h"
#include "pins.h"
#include <avr/interrupt.h>

// values for converting Futaba 7C RC PWM to Roboteq drive control (-1000 to 1000)
// CH1 LEFT 1116-1932 1522 neutral CH2 RIGHT 1100-1920 1512 neutral
// Note - left and right rc have different polarities! When you throttle forwards,
// expect to see left rc go low and right rc go high.

#define LEFT_PWM_NEUTRAL 1522
#define LEFT_PWM_RANGE 410
#define RIGHT_PWM_NEUTRAL 1512
#define RIGHT_PWM_RANGE 410
// deadband is 40 wide, 5%
#define LEFT_DEADBAND_MIN 1502
#define LEFT_DEADBAND_MAX 1542
#define RIGHT_DEADBAND_MIN 1492
#define RIGHT_DEADBAND_MAX 1532

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

// initialize PWM vals to neutral values
static volatile uint16_t LEFT_RC_pwm_val = 1520;
static volatile uint32_t LEFT_RC_prev_time = 0;
static volatile uint16_t RIGHT_RC_pwm_val = 1520;
static volatile uint32_t RIGHT_RC_prev_time = 0;
static volatile uint16_t TARGETING_ENABLE_pwm_val = 1520;
static volatile uint32_t TARGETING_ENABLE_prev_time = 0;


static volatile uint8_t PBLAST = 0; // TODO - what is the appropriate startup state?
static volatile bool NEW_RC = false;


ISR(PCINT0_vect)
{
    uint8_t PBNOW = PINB ^ PBLAST;
    PBLAST = PINB;
    uint8_t left_rc_bit = 1 << PINB6;
    uint8_t right_rc_bit = 1 << PINB7;
    
    // These can come in simultaneously so don't make this an if/else.
    if(PBNOW & left_rc_bit){
        if (PINB & left_rc_bit) { // Rising
            LEFT_RC_prev_time = micros();
        }
        else{
            LEFT_RC_pwm_val = micros() - LEFT_RC_prev_time;
        }
    }
    if(PBNOW & right_rc_bit){
        if (PINB & right_rc_bit){// Rising
            RIGHT_RC_prev_time = micros();
        }
        else{
            RIGHT_RC_pwm_val = micros() - RIGHT_RC_prev_time;
        }
    }
}

void TARGETING_ENABLE_falling(); // forward decl

void TARGETING_ENABLE_rising() {
    attachInterrupt(TARGETING_ENABLE, TARGETING_ENABLE_falling, FALLING);
    TARGETING_ENABLE_prev_time = micros();
}
void TARGETING_ENABLE_falling() {
    attachInterrupt(TARGETING_ENABLE, TARGETING_ENABLE_rising, RISING);
    TARGETING_ENABLE_pwm_val = micros() - TARGETING_ENABLE_prev_time;
    NEW_RC = true;
}

// Set up all RC interrupts
void attachRCInterrupts(){
    pinMode(LEFT_RC_PIN, INPUT_PULLUP);
    pinMode(RIGHT_RC_PIN, INPUT_PULLUP);
    pinMode(TARGETING_ENABLE_PIN, INPUT_PULLUP);
    
    cli();
    attachInterrupt(TARGETING_ENABLE, TARGETING_ENABLE_rising, RISING);

    PCICR |= 0b00000001; // Enables Ports B Pin Change Interrupts
    PCMSK0 |= 0b11000000; // Mask interrupts to PCINT6 and PCINT7
    sei();
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

uint16_t sbus_overrun = 0;
static uint8_t sbusData[25] = {0};
bool bufferSbusData() {
  // returns number of bytes available for reading from serial receive buffer, which is 64 bytes
  uint16_t count = Sbus.available();
  if (count == 25){
    Sbus.readBytes(sbusData, count);
    return true;
  } else if (count > 25) {
    uint8_t trash[64];
    while(count>0) {
        Sbus.readBytes(trash, min(64, count));
        count = Sbus.available();
    }
    sbus_overrun++;
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

#define AUTO_HAMMER_THRESHOLD 1000 // (190 down - 1800 up)
#define HAMMER_FIRE_THRESHOLD 1500 // 900 neutral, 170 to 1800
#define HAMMER_RETRACT_THRESHOLD 500

#define FLAME_PULSE_THRESHOLD 500
#define FLAME_CTRL_THRESHOLD 1500

#define GENTLE_HAM_F_THRESHOLD 500
#define GENTLE_HAM_R_THRESHOLD 1500

#define MAG_PULSE_THRESHOLD 500
#define MAG_CTRL_THRESHOLD 1500

#define DANGER_MODE_THRESHOLD 1500

uint16_t getRcBitfield() {
  uint16_t bitfield = 0;
  if ( sbusChannels[AUTO_HAMMER_ENABLE] > AUTO_HAMMER_THRESHOLD){
    bitfield |= AUTO_HAMMER_ENABLE_BIT;
  }
  if ( sbusChannels[HAMMER_CTRL] > HAMMER_FIRE_THRESHOLD){
    bitfield |= HAMMER_FIRE_BIT;
  }
  if ( sbusChannels[HAMMER_CTRL] < HAMMER_RETRACT_THRESHOLD){
    bitfield |= HAMMER_RETRACT_BIT;
  }
  
  // Full stick, flamethrower is on
  if ( sbusChannels[FLAME_CTRL] > FLAME_CTRL_THRESHOLD){
    bitfield |= FLAME_CTRL_BIT;
  }
  // Center position enables pulse mode
  if ( sbusChannels[FLAME_CTRL] > FLAME_PULSE_THRESHOLD && sbusChannels[FLAME_CTRL] < FLAME_CTRL_THRESHOLD ){
    bitfield |= FLAME_PULSE_BIT;
  }
  
  if ( sbusChannels[GENTLE_HAM_CTRL] < GENTLE_HAM_F_THRESHOLD){
    bitfield |= GENTLE_HAM_F_BIT;
  }
  if ( sbusChannels[GENTLE_HAM_CTRL] > GENTLE_HAM_R_THRESHOLD){
    bitfield |= GENTLE_HAM_R_BIT;
  }
  // Full stick, magnets are always on
  if ( sbusChannels[MAG_CTRL] > MAG_CTRL_THRESHOLD){
    bitfield |= MAG_CTRL_BIT;
  }
  // Center position, pulse when firing
  if ( sbusChannels[MAG_CTRL] > MAG_PULSE_THRESHOLD && sbusChannels[MAG_CTRL] < MAG_CTRL_THRESHOLD ){
    bitfield |= MAG_PULSE_BIT;
  }
  if ( sbusChannels[DANGER_MODE] > DANGER_MODE_THRESHOLD){
    bitfield |= DANGER_CTRL_BIT;
  }
  return bitfield;
}

// WARNING - this function assumes that you have successfully received an SBUS packet!
uint16_t getHammerIntensity(){
  uint16_t channel_val = sbusChannels[INTENSITY];
  if (channel_val < 172) { channel_val = 172; } else if (channel_val > 1811) { channel_val = 1811; }
  // Taranis throttle has been tuned for linearity, 9 steps on throttle lines. intensity is 0-based, 0-8.
  uint16_t intensity = (channel_val - 172 + 102) / 205;
  return intensity;
}

// 30-90, 60 cm neutral
uint16_t getRange() {
  uint16_t channel_val = sbusChannels[RANGE];
  if (channel_val < 172) { channel_val = 172; } else if (channel_val > 1811) { channel_val = 1811; }
  uint16_t range = (channel_val - 172) / 27 + 30;
  return range;
}