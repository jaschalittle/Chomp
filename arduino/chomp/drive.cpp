// functions to send serial drive commands to Roboteq motor controllers 
#include "Arduino.h"
#include "drive.h"
#include "pins.h"


// Serial out pins defined in chomp.ino-- check there to verify proper connectivity to motor controllers


void driveSetup() {
    DriveSerial.begin(115200);
    DriveSerial.println("@00^CPRI 1 0");  // set serial priority first
    delay(5);
    DriveSerial.println("@00^CPRI 2 1");  // set RC priority second
    delay(5);
    DriveSerial.println("@00^ECHOF 1");  // turn off command echo
    delay(5);
    DriveSerial.println("@00^RWD 100");  // set RS232 watchdog to 100 ms
}


void drive( int16_t l_drive_value, int16_t r_drive_value ) {
  // send "@nn!G mm" over software serial. mm is a command value, -1000 to 1000. nn is node number in RoboCAN network.
    if (l_drive_value > 1000) {
        l_drive_value = 1000;
    } else if (l_drive_value < -1000) {
        l_drive_value = -1000;
    }
    if (r_drive_value > 1000) {
        r_drive_value = 1000;
    } else if (r_drive_value < -1000) {
        r_drive_value = -1000;
    }
    DriveSerial.print("@01!G ");
    DriveSerial.println(l_drive_value);
    DriveSerial.print("@02!G ");
    DriveSerial.println(r_drive_value);
    DriveSerial.print("@03!G ");
    DriveSerial.println(r_drive_value);
    DriveSerial.print("@04!G ");
    DriveSerial.println(l_drive_value);
}
