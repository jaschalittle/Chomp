#ifndef RC_H
#define RC_H

enum RCinterrupts {
  WEAPONS_ENABLE = 0,
  AUTO_HAMMER_ENABLE = 1,
  HAMMER_CTRL = 2,
  HAMMER_INTENSITY = 3,
  FIRE_CTRL = 4,
};

extern volatile int WEAPONS_ENABLE_pwm_value;

void attachRCInterrupts();

#endif // RC_H
