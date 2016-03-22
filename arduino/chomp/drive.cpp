// functions to send serial drive commands to Roboteq motor controllers 
#include "Arduino.h"
#include "drive.h"
#include "pins.h"

// for testing CAN interface. RoboCAN does not require this.
static const uint8_t READ  = 0x03;   // SCP1000's read command
static const uint8_t WRITE = 0x02;   // SCP1000's write command
static const uint8_t RESET = 0xC0;   // SCP1000's write command

// Serial out pins defined in chomp.ino-- check there to verify proper connectivity to motor controllers


// for testing CAN interface. RoboCAN does not require this.
// void writeRegister( char address, char value) {
//     digitalWrite(CHIP_SELECT_PIN, LOW);
//     SPI.transfer(WRITE);
//     SPI.transfer(address);
//     SPI.transfer(value);
//     digitalWrite(CHIP_SELECT_PIN, HIGH);
//     delay(1);
// }


// for testing CAN interface. RoboCAN does not require this.
// void readRegister( char address ) {
//     digitalWrite(CHIP_SELECT_PIN, LOW);
//     SPI.transfer(READ); // command
//     SPI.transfer(address); // CNF1 address
//     uint8_t result = 0;
//     result = SPI.transfer(0x00);
//     digitalWrite(CHIP_SELECT_PIN, HIGH);
//     Debug.println(result, DEC);
// }


// // for testing CAN interface. RoboCAN does not require this.
// void canSetup() {
//     // SPI setup?
//     pinMode(CHIP_SELECT_PIN, OUTPUT);
//     SPI.begin();
//     SPI.beginTransaction(SPISettings(16000000, MSBFIRST, SPI_MODE0));
//     // SPI.setDataMode(SPI_MODE0);
//     // SPI.setBitOrder(MSBFIRST);
    
//     // reset, as recommended in MCP2515 datasheet
//     digitalWrite(CHIP_SELECT_PIN, LOW);
//     SPI.transfer(RESET);
//     digitalWrite(CHIP_SELECT_PIN, HIGH);
//     delay(10);
    
//     // set BRP to 4
//     writeRegister(0b00101010, 0b00000010);  // set CNF1 to 2 for 125k baud?
//     writeRegister(0b00101001, 0b00111010); // set CNF2  bits 3-5 are PS1 length in TQ units, bits 0-2 are PRSEG length in TQ units
//     writeRegister(0b00101000, 0b00000110); // set CNF3 bits 0-2 are PS2 length in TQ units
// }


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
}
