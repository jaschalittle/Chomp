#ifndef PWM_H
#define PWM_H

const float pwm_neutral = 1520/20000.0;

void pwm_duty_L(float duty);

void pwm_duty_R(float duty);

void pwm_setup();

void targeting_disable();

void targeting_enable();

#endif // PWM_H
