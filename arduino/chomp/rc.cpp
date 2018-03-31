// Hook up all the RC interrupts. 
#include "Arduino.h"
#include "rc.h"
#include "pins.h"
#include "weapons.h"
#include "utils.h"
#include <avr/interrupt.h>
#include <util/atomic.h>

enum RCinterrupts {
    DRIVE_DISTANCE = digitalPinToInterrupt(DRIVE_DISTANCE_PIN),
    TARGETING_ENABLE = digitalPinToInterrupt(TARGETING_ENABLE_PIN),
};


static uint16_t computeRCBitfield();
static void attachRCInterrupts();
static bool bufferSbusData();
static bool parseSbus();

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

extern HardwareSerial& Sbus;

// initialize PWM vals to neutral values
static volatile uint16_t LEFT_RC_pwm_val = 1500;
static volatile uint32_t LEFT_RC_prev_time = 0;
static volatile uint16_t RIGHT_RC_pwm_val = 1500;
static volatile uint32_t RIGHT_RC_prev_time = 0;
static volatile uint16_t TARGETING_ENABLE_pwm_val = 1500;
static volatile uint32_t TARGETING_ENABLE_prev_time = 0;
static volatile int DRIVE_DISTANCE_pwm_val = 1500;
static volatile int DRIVE_DISTANCE_prev_time = 0;

static uint32_t last_parse_time = 0;
static uint32_t radio_lost_timeout = 10000;

static volatile uint8_t PBLAST = 0; // 0 so that we detect rising interrupts first.
static volatile bool NEW_RC = false;

static uint16_t bitfield;
static uint16_t last_bitfield;

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
    attachInterrupt(TARGETING_ENABLE, TARGETING_ENABLE_falling, FALLING);
    TARGETING_ENABLE_prev_time = micros();
}
void TARGETING_ENABLE_falling() {
    attachInterrupt(TARGETING_ENABLE, TARGETING_ENABLE_rising, RISING);
    TARGETING_ENABLE_pwm_val = micros() - TARGETING_ENABLE_prev_time;
    NEW_RC = true;
}

static void DRIVE_DISTANCE_Falling();
static void DRIVE_DISTANCE_Rising(){
    attachInterrupt(WEAPONS_ENABLE, DRIVE_DISTANCE_Falling, FALLING );
    DRIVE_DISTANCE_prev_time = micros();
}
static void DRIVE_DISTANCE_Falling(){
    attachInterrupt(WEAPONS_ENABLE, DRIVE_DISTANCE_Rising, RISING);
    DRIVE_DISTANCE_pwm_val = micros() - DRIVE_DISTANCE_prev_time;
}

void rcInit() {
    Sbus.begin(100000);
    attachRCInterrupts();
    attachInterrupt(DRIVE_DISTANCE, DRIVE_DISTANCE_Rising, RISING);
    last_parse_time = micros();
}

// Set up all RC interrupts
static void attachRCInterrupts(){
    pinMode(LEFT_RC_PIN, INPUT_PULLUP);
    pinMode(RIGHT_RC_PIN, INPUT_PULLUP);
    pinMode(TARGETING_ENABLE_PIN, INPUT_PULLUP);

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        attachInterrupt(TARGETING_ENABLE, TARGETING_ENABLE_rising, RISING);

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

static void setWeaponsEnabled(bool state)
{
    if(state) {
        if (!g_enabled) {
            g_enabled = true;
            enableState();
        }
    } else {
        safeState();
        g_enabled = false;
    }
}

bool processSbusData(void) {
    bool ready = bufferSbusData();
    bool fail = false;
    if(ready) {
        fail = parseSbus();
        last_parse_time = micros();
        if(!fail) {
            computeRCBitfield();
        }
    }
    bool timeout = (micros() - last_parse_time) > radio_lost_timeout;
    bool radio_working = !(fail || timeout);
    if(!radio_working) {
        setWeaponsEnabled(false);
    }
    return radio_working;
}

uint16_t sbus_overrun = 0;
static uint8_t sbusData[25] = {0};
uint32_t last_sbus_time = 0;
uint8_t sbus_idx = 0;
static bool bufferSbusData() {
  // returns number of bytes available for reading from serial receive buffer, which is 64 bytes
  if(Sbus.available()>25) {
    while(Sbus.available()) {
        Sbus.read();
    }
    sbus_overrun++;
    return false;
  }
  while(Sbus.available()) {
    if(sbus_idx == 25) {
        sbus_overrun++;
        sbus_idx = 0;
    }
    uint8_t c = Sbus.read();
    last_sbus_time = micros();
    if(((sbus_idx == 0) && (c == 0x0f)) ||
        (sbus_idx>0 && sbus_idx<25)) {
        sbusData[sbus_idx++] = c;
    }
  }
  if(sbus_idx==25) {
      sbus_idx = 0;
      return true;
  }
  if((micros() - last_sbus_time)>1000) {
      last_sbus_time = micros();
      sbus_idx = 0;
  }
  return false;
}

static uint16_t sbusChannels [17];  // could initialize this with failsafe values for extra safety
static bool parseSbus(){
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
        failsafe = sbusData[23] & 0x08;
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

#define DRIVE_DISTANCE_PWM_MIN 1000
static int16_t max_drive_range=2000;
static int16_t min_drive_range=500;
static int16_t drive_range_scale=2;
int16_t getDriveDistance() {
    return clip((DRIVE_DISTANCE_pwm_val - DRIVE_DISTANCE_PWM_MIN)*drive_range_scale,
                0, max_drive_range) + min_drive_range;
}

#define WEAPONS_ENABLE_THRESHOLD 1450
#define AUTO_HAMMER_THRESHOLD 1000 // (190 down - 1800 up)
#define HAMMER_FIRE_THRESHOLD 1500 // 900 neutral, 170 to 1800
#define HAMMER_RETRACT_THRESHOLD 500

#define FLAME_PULSE_THRESHOLD 500
#define FLAME_CTRL_THRESHOLD 1500

#define GENTLE_HAM_F_THRESHOLD 500
#define GENTLE_HAM_R_THRESHOLD 1500

// #define MAG_PULSE_THRESHOLD 500
#define AUTO_SELF_RIGHT_THRESHOLD 1500
#define MANUAL_SELF_RIGHT_LEFT_THRESHOLD 500
#define MANUAL_SELF_RIGHT_RIGHT_THRESHOLD 1500

#define DANGER_MODE_THRESHOLD 1500

uint16_t getRcBitfield() {
    return bitfield;
}

uint16_t getRcBitfieldChanges() {
    uint16_t changes = bitfield ^ last_bitfield;
    last_bitfield = bitfield;
    return changes;
}

static uint16_t computeRCBitfield() {
  bitfield = 0;

  if(sbusChannels[WEAPONS_ENABLE] > WEAPONS_ENABLE_THRESHOLD) {
    bitfield |= WEAPONS_ENABLE_BIT;
  }
  setWeaponsEnabled(bitfield&WEAPONS_ENABLE_BIT);

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
  if ( sbusChannels[AUTO_SELF_RIGHT] > AUTO_SELF_RIGHT_THRESHOLD){
    bitfield |= AUTO_SELF_RIGHT_BIT;
  }
  if ( sbusChannels[DANGER_MODE] > DANGER_MODE_THRESHOLD){
    bitfield |= DANGER_CTRL_BIT;
  }
  if (sbusChannels[MANUAL_SELF_RIGHT] < MANUAL_SELF_RIGHT_LEFT_THRESHOLD) {
      bitfield |= MANUAL_SELF_RIGHT_LEFT_BIT;
  }
  if (sbusChannels[MANUAL_SELF_RIGHT] > MANUAL_SELF_RIGHT_RIGHT_THRESHOLD) {
      bitfield |= MANUAL_SELF_RIGHT_RIGHT_BIT;
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

// 300-908, 600 mm neutral
uint16_t getRange() {
  uint16_t channel_val = sbusChannels[RANGE];
  if (channel_val < 172) { channel_val = 172; } else if (channel_val > 1811) { channel_val = 1811; }
  uint16_t range = (channel_val - 172) * 15 / 28 + 30;
  return range;
}
