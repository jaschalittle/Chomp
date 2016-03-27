#ifndef PINS_H
#define PINS_H

#define GREEN 5
#define RED 4

// Drive RC input
// Mega2560 digital interrupt pins:
// 2 (int.0), 3 (int.1), 18 (int.5), 19 (int.4), 20 (int.3), 21 (int.2)
#define FUTABA_CH1_PIN 3
#define FUTABA_CH2_PIN 20
#define FUTABA_CH5_PIN 21

// Weapons RC input
#define WEAPONS_ENABLE_PIN 2

#define XBEE_CTS 8

// Sensors
#define PRESSURE_AI A7
#define ANGLE_AI A3

// Valve DOs
#define ENABLE_VALVE_DO 12
#define THROW_VALVE_DO 11
#define VENT_VALVE_DO 10
#define RETRACT_VALVE_DO 9

// Drive control output (for CAN testing)
// #define CHIP_SELECT_PIN 19

// These are defined in chomp.ino
extern HardwareSerial& Debug;
extern HardwareSerial& Xbee;
extern HardwareSerial& LeddarSerial;
extern HardwareSerial& Sbus;
extern HardwareSerial& DriveSerial;

extern bool g_enabled;

#endif
