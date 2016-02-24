// Setup and output functions for PWM output pins. 
#include "Arduino.h"
#include "pwm.h"

// PWM output frequency in Hz
static const int PWM_FREQ = 50;

// OC1A pin 11, OC5A pin 46
void pwm_duty_L( float duty ) {
  // turn off interrupts for atomic register operation
  noInterrupts();
//  OCR1A = 40000 * duty;
  OCR5A = 40000 * duty;
  interrupts();
}

// OC1B pin 12, OC5B pin 45
void pwm_duty_R( float duty ) {
  // turn off interrupts for atomic register operation
  noInterrupts();
//  OCR1B = 40000 * duty;
  OCR5B = 40000 * duty;
  interrupts();
}

void pwm_setup() {  
  // set timer1 to fast PWM mode 14 (WGM bits 1110), non-inverted (only COM1 bit 1). 
//  TCCR1A = (1 << COM1A1) | (1 << COM1B1) | (1 << WGM11);
  TCCR5A = (1 << COM5A1) | (1 << COM5B1) | (1 << WGM51);

  // set output PWM duty to neutral, 1520 / 20000. OCR1A = 40000 * 0.075
  pwm_duty_L(pwm_neutral);
  pwm_duty_R(pwm_neutral);

  // set timer1 prescaler to 8, set WGM bits for fast PWM mode 15
//  TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11);
  TCCR5B = (1 << WGM53) | (1 << WGM52) | (1 << CS51);

  // set timer1 TOP (ICR5) to counter needed for PWM freq, prescaled 2 GHz / PWM freq in Hz
  int top = 2000000 / PWM_FREQ;
  // turn off interrupts for atomic register operation
  noInterrupts();
//  ICR1 = top
  ICR5 = top;
  interrupts();

  // need to set OC1A OC1B to low before enabling output?
  // set pin (pins 45 and 46) data direction to output
  DDRL = (1 << DDB4) | (1 << DDB3);
}

void targeting_disable() { 
  // set pin (pins 45 and 46) data direction to input so that Futabas on Roboteqs will take control
  // NEED TO SET LOW FIRST???
  DDRL = (0 << DDB4) | (0 << DDB3);
}

void targeting_enable() {
  // set pin (pins 45 and 46) data direction to output
  DDRL = (1 << DDB4) | (1 << DDB3);
}
