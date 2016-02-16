// Hook up all the RC interrupts. 
#include "Arduino.h"
#include "interrupts.h"

// Externed in chomp.h
volatile int WEAPONS_ENABLE_pwm_value = 0;
volatile int WEAPONS_ENABLE_prev_time = 0;

void WEAPONS_ENABLE_rising();
void WEAPONS_ENABLE_falling();

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
