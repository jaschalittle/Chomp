// functions to send serial drive commands to Roboteq motor controllers 
#include "Arduino.h"
#include "drive.h"
#include "pins.h"
#include "SoftwareSerial.h"

// Serial out pins defined in chomp.ino-- check there to verify proper connectivity to motor controllers

void driveL( int16_t drive_value ) {
  // send "!G [nn] mm" over software serial. channel [nn] just absent for single channel controller? -1000 to 1000
    if (drive_value > 1000) {
        drive_value = 1000;
    } else if (drive_value < -1000) {
        drive_value = -1000;
    }
    LeftWheelSerial.print("!G ");
    LeftWheelSerial.print(drive_value);
}

// OC1B pin 12, OC5B pin 45
void driveR( int16_t drive_value ) {
  // send "!G [nn] mm" over software serial. channel [nn] just absent for single channel controller? -1000 to 1000
    if (drive_value > 1000) {
        drive_value = 1000;
    } else if (drive_value < -1000) {
        drive_value = -1000;
    }
    RightWheelSerial.print("!G ");
    RightWheelSerial.print(drive_value);
}
