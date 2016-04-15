#ifndef RC_H
#define RC_H

// Mega2560 digital interrupt pins:
// 2 (int.0), 3 (int.1), 18 (int.5), 19 (int.4), 20 (int.3), 21 (int.2)
// Futaba Ch1 is ailerons, Ch2 is elevators. Same on Taranis 'chump' model.
enum RCinterrupts {
  AILERON = digitalPinToInterrupt(2),
  ELEVATOR = digitalPinToInterrupt(3),
  THROTTLE = digitalPinToInterrupt(18),
  // for timed input
//  AILERON = digitalPinToInterrupt(2),
//  ELEVATOR = digitalPinToInterrupt(3),
};

bool buffer_rc_data();

void parse_sbus();

struct RCPacket {
  char bitfield;
  
};

// set up timers 3 and 4 for control inputs.

void attachRCInterrupts();

float get_aileron();

float get_elevator();

float get_throttle();

#endif // RC_H
