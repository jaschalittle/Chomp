#include <Leddar.h>

// Xbee configuration notes:
// Followed tutorial here: https://eewiki.net/display/Wireless/XBee+Wireless+Communication+Setup
// Xbee SN 13A200 40BEFC5C is set to Coordinator AT, and DH/DL programmed to the SN of the Router AT
// Xbee SN 13A200 40B9D1B1 is set to Router AT, and DH/DL programmed to the SN of the Coordinator AT
// They're talking on PAN ID 2001 (A Space Odyssey)
//HardwareSerial & Xbee = Serial;
HardwareSerial & TTL = Serial3;
HardwareSerial & LeddarSerial = Serial;

Leddar16 leddar(115200,1);

void setup() {
  // Xbee.begin(9600);
  TTL.begin(9600);
  LeddarSerial.begin(115200);
  leddar.init();
}

void loop() {
  // Xbee.write("A");
  char detections = leddar.getDetections();
  TTL.write("Detections: ");
  TTL.print(detections, DEC);
  TTL.write("\r\n");
  if (detections >= 0){
    for (int i = 0; i < leddar.NbDet; i++){
      TTL.print(leddar.Detections[i].Segment, DEC);
      TTL.write("/");
      TTL.print(leddar.Detections[i].Distance, DEC);
      TTL.write(" ");
      //leddar.Detections[i].Amplitude;
    }
    TTL.write("\r\n");   
  }
  delay(500);
}
