#ifndef PINS_H
#define PINS_H

//------------------ DEFINES----------------
// #define HARD_WIRED

// ----------------- ANALOG ----------------- 
// Sensors
#define PRESSURE_AI A15
#define ANGLE_AI A1
#define XBEE_CTS A12
// ----------------- DIGITAL  ---------------

// Weapons RC input
#define WEAPONS_ENABLE_PIN 2

#define ENABLE_VALVE_DO 4
#define VENT_VALVE_DO 5

#define IGNITER_DO 6
#define PROPANE_DO 7

#define RETRACT_VALVE_DO 8
#define THROW_VALVE_DO 9

#define MAG2_DO 10
#define MAG1_DO 11

// Drive RC input
// Mega2560 digital interrupt pins:
// 2 (int.0), 3 (int.1), 18 (int.5), 19 (int.4), 20 (int.3), 21 (int.2)
#define FUTABA_CH1_PIN 3
#define FUTABA_CH2_PIN 20
#define FUTABA_CH5_PIN 21

// Drive control output (for CAN testing)
// #define CHIP_SELECT_PIN 19

// ----------------- SERIAL ------------------
// These are defined in chomp.ino
extern HardwareSerial& Debug;
extern HardwareSerial& Xbee;
extern HardwareSerial& LeddarSerial;
extern HardwareSerial& Sbus;
extern HardwareSerial& DriveSerial;

// ----------------- GLOBALS ----------------
extern volatile bool g_enabled;

#endif
