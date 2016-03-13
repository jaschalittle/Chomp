// Setup and output functions for PWM output pins. 
#include "Arduino.h"
#include "pwm_drive.h"

// high PWM duty to left tread of chump is forward
// low PWM duty to right tread of chump is backward

// PWM output frequency in Hz
static const int pwm_freq = 50;

// pins to use for l and r treads on chump
// need to think about this for chomp
//static const int l_tread_pin = 45;
//static const int r_tread_pin = 46;
//static char* const tccr_reg_A = (char*) &TCCR5A;
//static char* const tccr_reg_B = (char*) &TCCR5B;
//static short* const icr_reg = (short*) &ICR5;
//static short* const l_tread_ocr_reg = (short*) &OCR5B;
//static short* const r_tread_ocr_reg = (short*) &OCR5A;
//static char* const ddr_reg = (char*) &DDRL;
//static const int l_tread_ddr_reg_bit = 4;
//static const int r_tread_ddr_reg_bit = 3;

//static const int l_tread_pin = 11;
//static const int r_tread_pin = 12;
//static char* const tccr_reg_A = (char*) &TCCR1A;
//static char* const tccr_reg_B = (char*) &TCCR1B;
//static short* const icr_reg = (short*) &ICR1;
//static short* const l_tread_ocr_reg = (short*) &OCR1A;
//static short* const r_tread_ocr_reg = (short*) &OCR1B;
//static char* const ddr_reg = (char*) &DDRB;
//static const int l_tread_ddr_reg_bit = 5;
//static const int r_tread_ddr_reg_bit = 6;

//static const int l_tread_pin = 2;
//static const int r_tread_pin = 3;
//static char* const tccr_reg_A = (char*) &TCCR3A;
//static char* const tccr_reg_B = (char*) &TCCR3B;
//static short* const icr_reg = (short*) &ICR3;
//static short* const l_tread_ocr_reg = (short*) &OCR3B;
//static short* const r_tread_ocr_reg = (short*) &OCR3C;
//static char* const ddr_reg = (char*) &DDRE;
//static const int l_tread_ddr_reg_bit = 4;
//static const int r_tread_ddr_reg_bit = 5;

static const int l_tread_pin = 6;
static const int r_tread_pin = 7;
static char* const tccr_reg_A = (char*) &TCCR4A;
static char* const tccr_reg_B = (char*) &TCCR4B;
static short* const icr_reg = (short*) &ICR4;
static short* const l_tread_ocr_reg = (short*) &OCR4A;
static short* const r_tread_ocr_reg = (short*) &OCR4B;
static char* const ddr_reg = (char*) &DDRH;
static const int l_tread_ddr_reg_bit = 3;
static const int r_tread_ddr_reg_bit = 4;


void pwmDutyL( float duty ) {
  // turn off interrupts for atomic register operation
  noInterrupts();
  *l_tread_ocr_reg = 40000 * duty;
  interrupts();
}

// OC1B pin 12, OC5B pin 45
void pwmDutyR( float duty ) {
  // turn off interrupts for atomic register operation
  noInterrupts();
  *r_tread_ocr_reg = 40000 * duty;
  interrupts();
}

void pwmOutputSetup() {
  
  // set output PWM duty to neutral before enabling output
  pwmDutyL(PWM_NEUTRAL);
  pwmDutyR(PWM_NEUTRAL);
  
  // set timer TOP (ICRn) to counter needed for PWM freq, prescaled 2 MHz / PWM freq in Hz
  int top = 2000000 / pwm_freq;

  // TCCRnA set timer to fast PWM mode 14 (WGM bits 1110), non-inverted (only COM1 bit 1)
  // TCCRnB set timer prescaler to 8, set WGM bits for fast PWM mode 15

  *tccr_reg_A = (1 << 7) | (1 << 5) | (1 << 1);
  *tccr_reg_B = (1 << 4) | (1 << 3) | (1 << 1);
  noInterrupts();
  *icr_reg = top;
  interrupts();
  *ddr_reg = (1 << l_tread_ddr_reg_bit) | (1 << r_tread_ddr_reg_bit);
}

void targetingDisable() { 
  // set pin data direction to input so that Futabas on Roboteqs will take control
  // NEED TO SET LOW FIRST???
  *ddr_reg = (0 << l_tread_ddr_reg_bit) | (0 << r_tread_ddr_reg_bit);
}

void targetingEnable() {
  // set pin data direction to output to take control from Futaba receivers on Roboteqs
  *ddr_reg = (1 << l_tread_ddr_reg_bit) | (1 << r_tread_ddr_reg_bit);
}
