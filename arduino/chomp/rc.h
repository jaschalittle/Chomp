#ifndef RC_H
#define RC_H

#include "pins.h"

enum RCinterrupts {
    WEAPONS_ENABLE = digitalPinToInterrupt(WEAPONS_ENABLE_PIN),
    TARGETING_ENABLE = digitalPinToInterrupt(TARGETING_ENABLE_PIN),
};

enum SBUSChannels {
    NUL = 0, // Global enable disable, read by ISR
    AUTO_HAMMER_ENABLE = 1,
    HAMMER_CTRL = 2,
    FLAME_CTRL = 3,
    AUTO_SELF_RIGHT = 4,
    GENTLE_HAM_CTRL = 5,
    INTENSITY = 6,
    DANGER_MODE = 7,
    RANGE = 8,
    MANUAL_SELF_RIGHT = 9,
};

// Boolean values coming in over RC are stored in a bitfield for ease of comparison
// to detect state changes.
enum RCBitfield {
    AUTO_HAMMER_ENABLE_BIT = 1,
    HAMMER_FIRE_BIT = 2,
    HAMMER_RETRACT_BIT = 4,
    FLAME_CTRL_BIT = 8,
    FLAME_PULSE_BIT = 16,
    GENTLE_HAM_F_BIT = 32,
    GENTLE_HAM_R_BIT = 64,
    AUTO_SELF_RIGHT_BIT = 128,
    DANGER_CTRL_BIT = 512,
    MANUAL_SELF_RIGHT_LEFT_BIT = 1024,
    MANUAL_SELF_RIGHT_RIGHT_BIT = 2048,
};

bool bufferSbusData();

bool parseSbus();

void attachRCInterrupts();

bool newRc();

int16_t getLeftRc();

int16_t getRightRc();

bool getTargetingEnable();

uint16_t getHammerIntensity();

uint16_t getRange();

uint16_t getRcBitfield();

#endif // RC_H
