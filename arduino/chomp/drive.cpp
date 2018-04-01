// functions to send serial drive commands to Roboteq motor controllers 
#include "Arduino.h"
#include "drive.h"
#include "pins.h"
#include "telem.h"

// Serial out pins defined in chomp.ino-- check there to verify proper connectivity to motor controllers
extern HardwareSerial& DriveSerial;

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

static void clampDriveCommands(int16_t &l_drive_value, int16_t &r_drive_value) {
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
}

void drive( int16_t &l_drive_value, int16_t &r_drive_value) {
    // send "@nn!G mm" over software serial. mm is a command value, -1000 to 1000. nn is node number in RoboCAN network.
    clampDriveCommands(l_drive_value, r_drive_value);
    DriveSerial.print("@01!G ");
    DriveSerial.println(l_drive_value);
    DriveSerial.print("@02!G ");
    DriveSerial.println(r_drive_value);
    DriveSerial.print("@03!G ");
    DriveSerial.println(r_drive_value);
    DriveSerial.print("@04!G ");
    DriveSerial.println(l_drive_value);
}

#define VOLTAGE_RESPONSE_LENGTH 12
#define VOLTAGE_RESPONSE_TIMEOUT 5000
#define NUM_DRIVES 5
void driveTelem(void) {
    static int idx = 0;
    static int16_t volts[NUM_DRIVES];
    char volt_buffer[VOLTAGE_RESPONSE_LENGTH];
    if(isTLMEnabled(TLM_ID_DRV)) {
        while(DriveSerial.available()) DriveSerial.read();
        String request("@0");
        request += (idx+1);
        request += "?V 2_";
        DriveSerial.write(request.c_str());
        uint32_t now = micros();
        while(DriveSerial.available()<VOLTAGE_RESPONSE_LENGTH &&
              micros() - now < VOLTAGE_RESPONSE_TIMEOUT);
        if(DriveSerial.available() >= VOLTAGE_RESPONSE_LENGTH) {
            DriveSerial.readBytes(volt_buffer, VOLTAGE_RESPONSE_LENGTH);
            // response is "@0i V=nnn\x0d+\x0d"
            // i=[1-NUM_DRIVES], nnn=volts*10
            volt_buffer[9] = '\x00';
            volts[idx++] = atoi(volt_buffer+6);
        } else {
            volts[idx++] = -1;
        }
        if(idx == NUM_DRIVES) {
            // id 1-4 are wheels, id 5 is weapons
            sendDriveTelem(reinterpret_cast<int16_t(&)[4]>(volts), volts[4]);
            idx = 0;
        }
    }
}
