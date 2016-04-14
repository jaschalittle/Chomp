#ifndef WEAPONS_H
#define WEAPONS_H

bool weaponsEnabled();

bool autofireEnabled(char bitfield);

void retract();

void fire( uint16_t hammer_intensity );

void noAngleRetract();

void noAngleFire( uint16_t hammer_intensity );

void gentleFire();

void flameStart();

void flameEnd();

void magOn();

void magOff();

void valveSafe();

void valveEnable();

void flameSafe();

void flameEnable();

void magnetSafe();


#endif // WEAPONS_H
