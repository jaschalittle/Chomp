#ifndef WEAPONS_H
#define WEAPONS_H
#include "rc.h"

bool weaponsEnabled();

bool autofireEnabled(char bitfield);

void retract( bool check_velocity = true );

void fire( uint16_t hammer_intensity, bool flame_pulse, bool mag_pulse );

void noAngleFire( uint16_t hammer_intensity, bool flame_pulse, bool mag_pulse );

void gentleFire( RCBitfield control );

void gentleRetract( RCBitfield control );

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
