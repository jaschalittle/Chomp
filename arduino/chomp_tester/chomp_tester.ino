#include <Servo.h>
Servo myservo;
HardwareSerial &Sbus = Serial;
HardwareSerial &Debug = Serial;

// Copied from pins.h in chomp
// ----------------- ANALOG ----------------- 
// Sensors
#define PRESSURE_AI A15
#define ANGLE_AI A1

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

#define XBEE_CTS 12

//-------------- Utilities ---------------------

void enable(){
  myservo.writeMicroseconds(2000);
  delay(100);
}

void disable(){
  myservo.writeMicroseconds(500);
  delay(100);
}

void expect( bool e){
  if( e ){
    Serial.print("\tPASS\r\n");
  } else {
    Serial.print("\tFAIL\r\n");
  }
}

void send_sbus(uint16_t* sbus_channels){
  char sbus_data[25];
  sbus_data[0] = 0x0F;
  sbus_data[24] = 0x00;
  sbus_data[1] = sbus_channels[0];
  sbus_data[2] = sbus_channels[0] >> 8 | sbus_channels[1] << 3;
  sbus_data[3] = sbus_channels[1] >> 5;
  
}
// -------------- Tests ------------------------


bool test_enable_disable(){
  pinMode(ENABLE_VALVE_DO, INPUT);
  pinMode(IGNITER_DO, INPUT);
  enable();
  int enable_valve = digitalRead(ENABLE_VALVE_DO);
  int igniter = digitalRead(IGNITER_DO);
  bool test1 = enable_valve == HIGH && igniter == HIGH;
  disable();
  enable_valve = digitalRead(ENABLE_VALVE_DO);
  igniter = digitalRead(IGNITER_DO);
  bool test2 = enable_valve == LOW && igniter == LOW;
  enable();
  enable_valve = digitalRead(ENABLE_VALVE_DO);
  igniter = digitalRead(IGNITER_DO);
  bool test3 = enable_valve == HIGH && igniter == HIGH;
  return test1 && test2 && test3;
}

void test_flamethrower(){
  
}

// Program without sbus line in
// Plug in sbus line
// Open Putty to Debug line
// Restart board
// Wait for light to turn on
// Receive report
void setup() {
  // Start sbus
  Sbus.begin(100000);
  // Simulates enable/disable RC pulse
  myservo.attach(3);

  bool enable_disable_result = test_enable_disable();
  
  // Signal readiness to print report, give user time to unplug sbus
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  delay(15*1000);
  digitalWrite(13, LOW);
  Debug.print("=========== TEST REPORT ===========\r\n");
  Debug.print("Testing enable disable\r\n");
  expect(enable_disable_result);
}

void loop() {
}
