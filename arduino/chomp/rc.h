#ifndef RC_H
#define RC_H

// Mega2560 digital interrupt pins:
// 2 (int.0), 3 (int.1), 18 (int.5), 19 (int.4), 20 (int.3), 21 (int.2)
// Futaba Ch1 is ailerons, Ch2 is elevators. Same on Taranis 'chump' model.
enum RCinterrupts {
  AILERON = digitalPinToInterrupt(2),
  ELEVATOR = digitalPinToInterrupt(3),
  THROTTLE = digitalPinToInterrupt(18),
  // for timed input
//  AILERON = digitalPinToInterrupt(2),
//  ELEVATOR = digitalPinToInterrupt(3),
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
  HAMMER_CTRL_BIT = 4,
  FLAME_CTRL_BIT = 8,
};

bool buffer_rc_data();

void parse_sbus();

// set up timers 3 and 4 for control inputs.

void attachRCInterrupts();

float get_aileron();

float get_elevator();

float get_throttle();

char get_rc_bitfield();

#endif // RC_H
