#ifndef PWM_DRIVE_H
#define PWM_DRIVE_H

extern const float PWM_NEUTRAL;

void pwmDutyL(float duty);

void pwmDutyR(float duty);

void pwmOutputSetup();

void targetingDisable();

void targetingEnable();

#endif // PWM_DRIVE_H
