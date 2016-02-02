#include <Leddar.h>

// Xbee configuration notes:
// Followed tutorial here: https://eewiki.net/display/Wireless/XBee+Wireless+Communication+Setup
// Xbee SN 13A200 40BEFC5C is set to Coordinator AT, and DH/DL programmed to the SN of the Router AT
// Xbee SN 13A200 40B9D1B1 is set to Router AT, and DH/DL programmed to the SN of the Coordinator AT
// They're talking on PAN ID 2001 (A Space Odyssey)
HardwareSerial & Xbee = Serial2;
HardwareSerial & LeddarSerial = Serial;

Leddar16 leddar(115200,1);
bool VERBOSE = false;

void setup() {
  Xbee.begin(9600);
  LeddarSerial.begin(115200);
  leddar.init();
}

void fire(){
  pinMode(27, OUTPUT);
  digitalWrite(27, HIGH);
//  Xbee.write("FIRE!\t");
//  Xbee.write(0x07);
//  Xbee.flush();
  delay(50);
  digitalWrite(27, LOW);
}

void loop() {
  int fire_threshold = 60;
  char detections = leddar.getDetections();
  if ( VERBOSE ){
    Xbee.write("\r\n");
    Xbee.write("Detections: ");
    Xbee.print(detections, DEC);
    Xbee.write("\r\n");
  }
  if (detections >= 0){
    for (int i = 0; i < leddar.NbDet; i++){
      if ( VERBOSE ) {
        Xbee.print(leddar.Detections[i].Segment, DEC);
        Xbee.write("/");
        Xbee.print(leddar.Detections[i].Distance, DEC);
        Xbee.write(" ");
      }
      if (leddar.Detections[i].Distance < fire_threshold){
        fire();
        break;
      }
      //leddar.Detections[i].Amplitude;
    }
       
  }
  //delay(500);
}
