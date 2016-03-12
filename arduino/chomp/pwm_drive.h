#ifndef PWM_DRIVE_H
#define PWM_DRIVE_H

const float PWM_NEUTRAL = 1500/20000.0;

void pwm_duty_L(float duty);

void pwm_duty_R(float duty);

void pwm_output_setup();

void targeting_disable();

void targeting_enable();

#endif // PWM_DRIVE_H
