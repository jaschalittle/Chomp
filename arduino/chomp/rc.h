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
    NUL = 0,
    AUTO_HAMMER_ENABLE = 1,
    HAMMER_CTRL = 2,
    FLAME_CTRL = 3,
    MAG_CTRL = 4
};

// Boolean values coming in over RC are stored in a bitfield for ease of comparison
// to detect state changes.
enum RCBitfield {
    AUTO_HAMMER_ENABLE_BIT = 2,
    HAMMER_FIRE_BIT = 4,
    HAMMER_RETRACT_BIT = 8,
    FLAME_CTRL_BIT = 16,
    MAG_CTRL_BIT = 32,
};

bool bufferSbusData();

bool parseSbus();

void attachRCInterrupts();

int16_t getLeftRc();

int16_t getRightRc();

bool getTargetingEnable();

uint8_t getRcBitfield();

#endif // RC_H
