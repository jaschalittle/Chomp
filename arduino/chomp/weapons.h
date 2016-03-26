#ifndef WEAPONS_H
#define WEAPONS_H

bool weaponsEnabled(char bitfield);

bool autofireEnabled(char bitfield);

void retract(char bitfield);

void fire(char bitfield);

void valveReset();

void valveSetup();

void flameStart(char bitfield);

void flameEnd();

#endif // WEAPONS_H
