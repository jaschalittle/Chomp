#ifndef RC_H
#define RC_H

#include "pins.h"

enum RCinterrupts {
    LEFT_RC = digitalPinToInterrupt(FUTABA_CH1),
    RIGHT_RC = digitalPinToInterrupt(FUTABA_CH2),
    TARGETING_ENABLE = digitalPinToInterrupt(FUTABA_CH5),
};

enum SBUSChannels {
    WEAPONS_ENABLE = 0,
    AUTO_HAMMER_ENABLE = 1,
    HAMMER_CTRL = 2,
    FLAME_CTRL = 3
};

// Boolean values coming in over RC are stored in a bitfield for ease of comparison
// to detect state changes.
enum RCBitfield {
    WEAPONS_ENABLE_BIT = 1,
    AUTO_HAMMER_ENABLE_BIT = 2,
    HAMMER_FIRE_BIT = 4,
    HAMMER_RETRACT_BIT = 8,
    FLAME_CTRL_BIT = 16,
};

bool bufferSbusData();

void parseSbus();

void attachRCInterrupts();

int16_t getLeftRc();

int16_t getRightRc();

float getTargetingEnable();

char getRcBitfield();

#endif // RC_H
