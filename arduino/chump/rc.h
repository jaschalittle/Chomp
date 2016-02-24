#ifndef RC_H
#define RC_H

// Futaba Ch1 is ailerons, Ch2 is elevators
enum RCinterrupts {
  AILERON = digitalPinToInterrupt(2),
  ELEVATOR = digitalPinToInterrupt(3),
  THROTTLE = digitalPinToInterrupt(18),
  // for timed input
//  AILERON = digitalPinToInterrupt(2),
//  ELEVATOR = digitalPinToInterrupt(3),
};

void attachRCInterrupts();

float get_aileron();

float get_elevator();

float get_throttle();

#endif // RC_H
