#ifndef PINS_H
#define PINS_H
#include "DMASerial.h"

//------------------ DEFINES----------------
// #define HARD_WIRED

// ----------------- ANALOG ----------------- 
// Sensors
#define PRESSURE_AI A15
#define ANGLE_AI A1
#define XBEE_CTS A12
// ----------------- DIGITAL  ---------------

// Mega2560 digital interrupt pins:
// 2 (int.0), 3 (int.1), 18 (int.5), 19 (int.4), 20 (int.3), 21 (int.2)
// We currently use 2 and 3 for these high-pri PWM inputs and leave the
// rest for other purposes
#define WEAPONS_ENABLE_PIN 2       // Weapons radio
#define TARGETING_ENABLE_PIN 3     // Drive radio ch5

#define ENABLE_VALVE_DO 4
#define VENT_VALVE_DO 5

#define IGNITER_DO 6      
#define PROPANE_DO 7

#define RETRACT_VALVE_DO 8
#define THROW_VALVE_DO 9

#define MAG2_DO 10
#define MAG1_DO 11

// These are handled as pin change interrupts on the PINB bank
#define LEFT_RC_PIN 12             // Drive radio ch1
#define RIGHT_RC_PIN 13            // Drive radio ch2

// Drive control output (for CAN testing)
// #define CHIP_SELECT_PIN 19

// ----------------- SERIAL ------------------
// These are defined in chomp.ino
extern DMASerial& Xbee;
extern HardwareSerial& LeddarSerial;
extern HardwareSerial& Sbus;
extern HardwareSerial& DriveSerial;

// ----------------- GLOBALS ----------------
extern volatile bool g_enabled;

#endif
