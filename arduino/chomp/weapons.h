#ifndef WEAPONS_H
#define WEAPONS_H

bool weaponsEnabled(char bitfield);

bool autofireEnabled(char bitfield);

void retract(char bitfield);

void fire(char bitfield);

void valveSafe();

void valveSetup();

void flameSafe();

void flameSetup();

void flameStart();

void flameEnd();

void magnetSafe();


#endif // WEAPONS_H
