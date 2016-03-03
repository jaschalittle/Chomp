// Hook up all the RC interrupts. 
#include "Arduino.h"
#include "rc.h"

// set up pins to use for aileron and elevator input from drive
// input capture unit p140. have to use ICP4 (pin 49) and 5 (pin 48), ICP1 and 3 not pinned out.
// need to think about this for chomp
static const int aileron_pin = 48;
static const int elevator_pin = 49;
static char* const tccr_reg_A = (char*) &TCCR4A;
static char* const tccr_reg_B = (char*) &TCCR4B;
static short* const icr_reg = (short*) &ICR4;
static short* const l_tread_ocr_reg = (short*) &OCR4B;
static short* const r_tread_ocr_reg = (short*) &OCR4A;

// initialize PWM vals to neutral values
static volatile int AILERON_pwm_val = 1520;
static volatile int AILERON_prev_time = 0;
static volatile int ELEVATOR_pwm_val = 1520;
static volatile int ELEVATOR_prev_time = 0;
static volatile int THROTTLE_pwm_val = 1520;
static volatile int THROTTLE_prev_time = 0;

void AILERON_rising(), AILERON_falling(), ELEVATOR_rising(), ELEVATOR_falling(), THROTTLE_rising(), THROTTLE_falling();

//void AILERON_rising();
//void AILERON_falling();
//void ELEVATOR_rising();
//void ELEVATOR_falling();
//void THROTTLE_rising();
//void THROTTLE_falling();

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

#define CREATE_RISING_TIMER_ISR( timer_capt_interrupt )\
void rc_interrupt ## _rising() {\
  attachInterrupt(timer_capt_interrupt, timer_capt_interrupt ## _falling, FALLING);\
  rc_interrupt ## _prev_time = micros();\
}

#define CREATE_FALLING_TIMER_ISR( timer_capt_interrupt )\
void timer_capt_interrupt ## _falling() {\
  attachInterrupt(timer_capt_interrupt, timer_capt_interrupt ## _rising, RISING);\
  timer_capt_interrupt ## _pwm_val = micros() - timer_capt_interrupt ## _prev_time;\
}

CREATE_FALLING_ISR(AILERON);
CREATE_RISING_ISR(AILERON);
CREATE_FALLING_ISR(ELEVATOR);
CREATE_RISING_ISR(ELEVATOR);
CREATE_FALLING_ISR(THROTTLE);
CREATE_RISING_ISR(THROTTLE);

// Set up all RC interrupts
void attachRCInterrupts(){
  attachInterrupt(AILERON, AILERON_rising, RISING);
  attachInterrupt(ELEVATOR, ELEVATOR_rising, RISING);
  attachInterrupt(THROTTLE, THROTTLE_rising, RISING);
}

float get_aileron() {
  return AILERON_pwm_val / 20000.0;
}

float get_elevator() {
  return ELEVATOR_pwm_val / 20000.0;
}

float get_throttle() {
  return THROTTLE_pwm_val / 20000.0;
}
