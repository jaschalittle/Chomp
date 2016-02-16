#ifndef RC_H
#define RC_H

enum RCinterrupts {
  WEAPONS_ENABLE = 0,
  AUTO_HAMMER_ENABLE = 1,
  HAMMER_CTRL = 2,
  HAMMER_INTENSITY = 3,
  FLAME_CTRL = 4,
};

enum RCBitfield {
  WEAPONS_ENABLE_BIT = 1,
  AUTO_HAMMER_ENABLE_BIT = 2,
  HAMMER_CTRL_BIT = 4,
  FLAME_CTRL_BIT = 8,
};

extern volatile int WEAPONS_ENABLE_pwm_value;
const static int WEAPONS_ENABLE_threshold = 500;

extern volatile int FLAME_CTRL_pwm_value;
const static int FLAME_CTRL_threshold = 500;

void attachRCInterrupts();

#endif // RC_H
