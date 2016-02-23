// Hook up all the RC interrupts. 
#include "Arduino.h"
#include "rc.h"

// initialize PWM vals to neutral values
volatile int L_TREAD_pwm_val = 1520;
volatile int L_TREAD_prev_time = 0;
volatile int R_TREAD_pwm_val = 1520;
volatile int R_TREAD_prev_time = 0;

void L_TREAD_rising();
void L_TREAD_falling();
void R_TREAD_rising();
void R_TREAD_falling();

// set up PWM input pins. these ought to reset timer if it ever gets to TOP, as this implies falling edge was not seen.
// noise canceler takes up four system clock cycles, not affected by prescaler
// need to set ICIEn bit in TIMSKn register to enable input interrupts?

// ASAP change edge detection to falling ICESn bit in TCCRnx register, 0 falling 1 rising
// ASAP clear ICFn. ICFn triggers interrupt. ICFn bit is in TIFRn register.
// set TCTn to 0? seems like starting timer at bottom would be nice
// read ICRn register to get timestamp and store it


// Forgive me, I know not what I do.
#define CREATE_RISING_ISR( rc_interrupt )\
void rc_interrupt ## _rising() {\
  attachInterrupt(rc_interrupt, rc_interrupt ## _falling, FALLING);\
  rc_interrupt ## _prev_time = micros();\
}

// read ICRn register to get timestamp, compare to rising timestamp. might need some cuteness if overlaps a clock cycle

#define CREATE_FALLING_ISR( rc_interrupt )\
void rc_interrupt ## _falling() {\
  attachInterrupt(rc_interrupt, rc_interrupt ## _rising, RISING);\
  rc_interrupt ## _pwm_val = micros() - rc_interrupt ## _prev_time;\
}

CREATE_FALLING_ISR(L_TREAD);
CREATE_RISING_ISR(L_TREAD);
CREATE_FALLING_ISR(R_TREAD);
CREATE_RISING_ISR(R_TREAD);

// Set up all RC interrupts
void attachRCInterrupts(){
  attachInterrupt(L_TREAD, L_TREAD_rising, RISING);
  attachInterrupt(R_TREAD, R_TREAD_rising, RISING);
}
