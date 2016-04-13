#ifndef RC_H
#define RC_H

#include "pins.h"

enum RCinterrupts {
    WEAPONS_ENABLE = digitalPinToInterrupt(WEAPONS_ENABLE_PIN),
    LEFT_RC = digitalPinToInterrupt(FUTABA_CH1_PIN),
    RIGHT_RC = digitalPinToInterrupt(FUTABA_CH2_PIN),
    TARGETING_ENABLE = digitalPinToInterrupt(FUTABA_CH5_PIN),
};

enum SBUSChannels {
    NUL = 0, // Global enable disable, read by ISR
    AUTO_HAMMER_ENABLE = 1,
    HAMMER_CTRL = 2,
    FLAME_CTRL = 3,
    MAG_CTRL = 4,
    GENTLE_HAM_CTRL = 5,
    INTENSITY = 6,
    DANGER_MODE = 7,
};

// Boolean values coming in over RC are stored in a bitfield for ease of comparison
// to detect state changes.
enum RCBitfield {
    AUTO_HAMMER_ENABLE_BIT = 1,
    HAMMER_FIRE_BIT = 2,
    HAMMER_RETRACT_BIT = 4,
    FLAME_CTRL_BIT = 8,
    GENTLE_HAM_F_BIT = 16,
    GENTLE_HAM_R_BIT = 32,
    MAG_CTRL_BIT = 64,
    DANGER_CTRL_BIT = 128,
};

bool bufferSbusData();

bool parseSbus();

void attachRCInterrupts();

bool newRc();

int16_t getLeftRc();

int16_t getRightRc();

bool getTargetingEnable();

uint16_t getHammerIntensity();

uint8_t getRcBitfield();

#endif // RC_H
