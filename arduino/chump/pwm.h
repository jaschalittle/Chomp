#ifndef PWM_H
#define PWM_H

// PWM output frequency in Hz
static int PWM_FREQ = 50;

// this may have wreaked havoc with pwm vals on Sunday, Feb 21?
//extern volatile int L_TREAD_pwm_val;
//
//extern volatile int R_TREAD_pwm_val;

void pwm_duty_L(float duty);

void pwm_duty_R(float duty);

void pwm_setup();

void targeting_disable();

void targeting_enable();

#endif // PWM_H
