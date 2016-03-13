#ifndef RC_H
#define RC_H

// Mega2560 digital interrupt pins:
// 2 (int.0), 3 (int.1), 18 (int.5), 19 (int.4), 20 (int.3), 21 (int.2)
// Futaba Ch1 is ailerons, Ch2 is elevators. Same on Taranis 'chump drive' model.
enum RCinterrupts {
  LEFT_RC = digitalPinToInterrupt(2),
  RIGHT_RC = digitalPinToInterrupt(3),
  TARGETING_ENABLE = digitalPinToInterrupt(20),
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

float getLeftRc();

float getRightRc();

float getTargetingEnable();

char getRcBitfield();

#endif // RC_H
