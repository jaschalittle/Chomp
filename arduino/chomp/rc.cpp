// Hook up all the RC interrupts. 
#include "Arduino.h"
#include "rc.h"

HardwareSerial& SbusSerial = Serial3;

// initialize PWM vals to neutral values
static volatile int AILERON_pwm_val = 1520;
static volatile int AILERON_prev_time = 0;
static volatile int ELEVATOR_pwm_val = 1520;
static volatile int ELEVATOR_prev_time = 0;
static volatile int THROTTLE_pwm_val = 1520;
static volatile int THROTTLE_prev_time = 0;

void AILERON_rising();
void AILERON_falling();
void ELEVATOR_rising();
void ELEVATOR_falling();
void THROTTLE_rising();
void THROTTLE_falling();

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

CREATE_FALLING_ISR(AILERON);
CREATE_RISING_ISR(AILERON);
CREATE_FALLING_ISR(ELEVATOR);
CREATE_RISING_ISR(ELEVATOR);
CREATE_FALLING_ISR(THROTTLE);
CREATE_RISING_ISR(THROTTLE);

// Set up all RC interrupts
void attachRCInterrupts(){
  attachInterrupt(AILERON, AILERON_rising, RISING);
  attachInterrupt(ELEVATOR, ELEVATOR_rising, RISING);
  attachInterrupt(THROTTLE, THROTTLE_rising, RISING);
}

unsigned char sbusData[25] = {0};
bool bufferSbusData() {
  // returns number of bytes available for reading from serial receive buffer, which is 64 bytes
  unsigned int count = SbusSerial.available();
  if (count == 25){
    SbusSerial.readBytes(sbusData, count);
    return true;
  } else if (count > 25) {
    unsigned char trash[64];
    SbusSerial.readBytes(trash, count);
    return false;
  }
  return false;
}

uint16_t sbusChannels [17];
void parse_sbus(){
  if (sbusData[0] == 0x0F && sbusData[24] == 0x00) {
    // perverse little endian-ish packet structure-- low bits come in first byte, remaining high bits
    // in next byte
    // NB: chars are promoted to shorts implicitly before bit shifts
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
  }
}

float get_aileron() {
  // return (sbusChannels[0] - 236) / 1639;
  return AILERON_pwm_val / 20000.0;
}

float get_elevator() {
  // return (sbusChannels[1] - 236) / 1639;
  return ELEVATOR_pwm_val / 20000.0;
}

float get_throttle() {
  // return (sbusChannels[2] - 236) / 1639;
  return THROTTLE_pwm_val / 20000.0;
}

// TODO get real thresholds
#define WEAPONS_ENABLE_THRESHOLD 500
#define HAMMER_CTRL_THRESHOLD 500
#define FLAME_CTRL_THRESHOLD 500
#define AUTO_HAMMER_THRESHOLD 500
char get_rc_bitfield() {
  char bitfield = 0;
  
  if ( sbusChannels[WEAPONS_ENABLE] > WEAPONS_ENABLE_THRESHOLD ){
    bitfield |= WEAPONS_ENABLE_BIT;
  }
  if ( sbusChannels[AUTO_HAMMER_ENABLE] > AUTO_HAMMER_THRESHOLD){
    bitfield |= AUTO_HAMMER_ENABLE_BIT;
  }
  if ( sbusChannels[HAMMER_CTRL] > HAMMER_CTRL_THRESHOLD){
    bitfield |= HAMMER_CTRL_BIT;
  }
  if ( sbusChannels[FLAME_CTRL] > FLAME_CTRL_THRESHOLD){
    bitfield |= FLAME_CTRL_BIT;
  }
  return bitfield;
}

