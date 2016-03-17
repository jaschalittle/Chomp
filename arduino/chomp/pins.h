#ifndef PINS_H
#define PINS_H

#define GREEN 5
#define RED 4

// RC fast interrupt channels (note: these are not pin numbers, they are interrupt numbers!)
#define WEAPON_ENABLE 5

// RC pin bank channels


// Sensors
#define PRESSURE_AI 7
#define ANGLE_AI 15

// Valve DOs
#define ENABLE_VALVE_DO 12
#define THROW_VALVE_DO 11
#define VENT_VALVE_DO 10
#define RETRACT_VALVE_DO 9

// Forward declaration of SoftwareSerial class
class SoftwareSerial;

extern HardwareSerial& Debug;               // Serial, defined in chomp.ino
extern HardwareSerial& Xbee;                // Serial1
extern HardwareSerial& LeddarSerial;        // Serial2
extern HardwareSerial& Sbus;                // Serial3
extern SoftwareSerial& LeftWheelSerial;     // RX pin 22, TX pin 6
extern SoftwareSerial& RightWheelSerial;    // RX pin 23, TX pin 7


#endif
