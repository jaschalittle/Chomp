#include <Arduino.h>
#include <util/atomic.h>
#include "pins.h"
#include "weapons.h"
#include "wiring_private.h"
#include "telem.h"

static void setWeaponsEnabled(bool state);
static uint16_t computeRCBitfield();
static bool parseSbus();

static uint32_t last_sbus_time = 0;
static uint32_t radio_lost_timeout = 100000;
static uint8_t sbus_idx;
static uint8_t sbusData[25];
static bool new_packet = false;
uint16_t sbus_overrun;
static uint16_t sbusChannels [17];  // could initialize this with failsafe values for extra safety
static bool failsafe = true;
static uint32_t last_parse_time = 0;
static uint16_t bitfield;
static uint16_t last_bitfield;

ISR(USART3_RX_vect)
{
    uint8_t c = UDR3;
    uint32_t now = micros();
    int32_t dt = now-last_sbus_time;
    last_sbus_time = now;
    if(dt>1000) {
        sbus_idx = 0;
    }
    if(!new_packet && (sbus_idx>0 || c=='\x0f')) {
        sbusData[sbus_idx++] = c;
    }
    if(sbus_idx == 25) {
        if(sbusData[0] == '\x0f' && sbusData[24] == 0) {
            new_packet = true;
        } else {
            sbus_overrun++;

        }
        sbus_idx = 0;
    }
}


ISR(USART3_UDRE_vect)
{
}

static void processSbus(void) {
    if(new_packet) {
        bool fail = parseSbus();
        if(!fail) {
            computeRCBitfield();
        } else {
            bitfield = 0;
            setWeaponsEnabled(false);
        }
        last_parse_time = micros();
        new_packet = false;
    }
}

void SBusInit() {
    // Try u2x mode first
    uint32_t baud = 100000;
    uint16_t baud_setting = (F_CPU / 4 / baud - 1) / 2;
    UCSR3A = 1 << U2X0;

    // assign the baud_setting, a.k.a. ubrr (USART Baud Rate Register)
    UBRR3H = baud_setting >> 8;
    UBRR3L = baud_setting;

    UCSR3C = SERIAL_8N1;

    sbi(UCSR3B, RXEN0);
    sbi(UCSR3B, RXCIE0);
    last_parse_time = micros();
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

bool sbusGood(void) {
    processSbus();
    bool timeout = ((micros() - last_parse_time) > radio_lost_timeout);
    bool working = !(failsafe || timeout);
    if(!working) {
        setWeaponsEnabled(false);
    }
    return working;
}

static bool parseSbus(){
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

#define WEAPONS_ENABLE_THRESHOLD 1450
#define AUTO_HAMMER_THRESHOLD 1000 // (190 down - 1800 up)
#define HAMMER_FIRE_THRESHOLD 1500 // 900 neutral, 170 to 1800
#define HAMMER_RETRACT_THRESHOLD 500

#define FLAME_PULSE_THRESHOLD 500
#define FLAME_CTRL_THRESHOLD 1500

#define AUTO_HOLD_DOWN_THRESHOLD 500
#define MANUAL_HOLD_DOWN_THRESHOLD 1500

#define GENTLE_HAM_F_THRESHOLD 500
#define GENTLE_HAM_R_THRESHOLD 1500

// #define MAG_PULSE_THRESHOLD 500
#define AUTO_SELF_RIGHT_THRESHOLD 1500
#define MANUAL_SELF_RIGHT_LEFT_THRESHOLD 500
#define MANUAL_SELF_RIGHT_RIGHT_THRESHOLD 1500

#define DANGER_MODE_THRESHOLD 1500

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

  if( sbusChannels[HOLD_DOWN] > MANUAL_HOLD_DOWN_THRESHOLD ){
      bitfield |= MANUAL_HOLD_DOWN;
  }
  else if( AUTO_HOLD_DOWN_THRESHOLD < sbusChannels[HOLD_DOWN] &&
      sbusChannels[HOLD_DOWN] < MANUAL_HOLD_DOWN_THRESHOLD ){
      bitfield |= AUTO_HOLD_DOWN;
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

uint16_t getRcBitfield() {
    uint16_t bits;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        bits = bitfield;
    }
    return bits;
}

uint16_t getRcBitfieldChanges() {
    uint16_t changes;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        changes = bitfield ^ last_bitfield;
        last_bitfield = bitfield;
    }
    return changes;
}

// WARNING - this function assumes that you have successfully received an SBUS packet!
uint16_t getHammerIntensity(){
  uint16_t channel_val;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      channel_val = sbusChannels[INTENSITY];
  }
  if (channel_val < 172) { channel_val = 172; } else if (channel_val > 1811) { channel_val = 1811; }
  // Taranis throttle has been tuned for linearity, 9 steps on throttle lines. intensity is 0-based, 0-8.
  uint16_t intensity = (channel_val - 172 + 102) / 205;
  return intensity;
}

// 300-908, 600 mm neutral
uint16_t getRange() {
  uint16_t channel_val;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      channel_val = sbusChannels[RANGE];
  }
  if (channel_val < 172) { channel_val = 172; } else if (channel_val > 1811) { channel_val = 1811; }
  uint16_t range = (channel_val - 172) * 15 / 28 + 300;
  return range;
}
