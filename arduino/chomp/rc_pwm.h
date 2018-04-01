#ifndef RC_H
#define RC_H

#include "pins.h"

void rcInit();

bool newRc();

int16_t getLeftRc();

int16_t getRightRc();

bool getTargetingEnable();

int16_t getDriveDistance();

#endif // RC_H
