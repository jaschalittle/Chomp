#ifndef RC_H
#define RC_H

enum RCinterrupts {
  L_TREAD = digitalPinToInterrupt(2),
  R_TREAD = digitalPinToInterrupt(3),
};

extern volatile int L_TREAD_pwm_val;

extern volatile int R_TREAD_pwm_val;

void attachRCInterrupts();

#endif // RC_H
