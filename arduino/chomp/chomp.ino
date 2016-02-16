#include <Leddar.h>
#include "interrupts.h"
#include "leddar_wrapper.h"

// Xbee configuration notes:
// Followed tutorial here: https://eewiki.net/display/Wireless/XBee+Wireless+Communication+Setup
// Xbee SN 13A200 40BEFC5C is set to Coordinator AT, and DH/DL programmed to the SN of the Router AT
// Xbee SN 13A200 40B9D1B1 is set to Router AT, and DH/DL programmed to the SN of the Coordinator AT
// They're talking on PAN ID 2001 (A Space Odyssey)
HardwareSerial & Xbee = Serial2;

bool VERBOSE = false;

void setup() {
  Xbee.begin(9600);
  leddar_wrapper_init();
  attachRCInterrupts();
}

void readMLHPressure(){
  int counts = analogRead(7);
  float voltage = counts * (5.0 / 1023);
  float pressure = (voltage - 0.5) * (500.0/4.0);
  Xbee.write("Pressure: ");
  Xbee.print(pressure);
  Xbee.write(" (");
  Xbee.print(voltage);
  Xbee.write("V)\r\n");
}

int previous_leddar_state = FAR_ZONE;
void loop() {
  int current_leddar_state = poll_leddar();
  if (WEAPONS_ENABLE_pwm_value > 5){
    Xbee.write("yo");
  }
}
