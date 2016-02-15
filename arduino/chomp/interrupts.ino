// Hook up all the RC interrupts. 

enum RCinterrupts {
  WEAPONS_ENABLE = 0,
  AUTO_HAMMER_ENABLE = 1,
  HAMMER_CTRL = 2,
  HAMMER_INTENSITY = 3,
  FIRE_CTRL = 4,
};
 
volatile int WEAPONS_ENABLE_pwm_value = 0;
volatile int WEAPONS_ENABLE_prev_time = 0;

// Forgive me, I know not what I do.
#define CREATE_RISING_ISR( rc_interrupt )\
void rc_interrupt ## _rising() {\
  attachInterrupt(rc_interrupt, rc_interrupt ## _falling, FALLING);\
  rc_interrupt ## _prev_time = micros();\
}

#define CREATE_FALLING_ISR( rc_interrupt )\
void rc_interrupt ## _falling() {\
  attachInterrupt(rc_interrupt, rc_interrupt ## _rising, RISING);\
  rc_interrupt ## _pwm_value = micros() - rc_interrupt ## _prev_time;\
}

CREATE_FALLING_ISR(WEAPONS_ENABLE);
CREATE_RISING_ISR(WEAPONS_ENABLE);

// Set up all RC interrupts
void attachRCInterrupts(){
  attachInterrupt(WEAPONS_ENABLE, WEAPONS_ENABLE_rising, RISING);
}
