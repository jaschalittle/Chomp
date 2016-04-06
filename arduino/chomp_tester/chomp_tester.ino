#include <Servo.h>
#include <SoftwareSerial.h>
Servo myservo;
SoftwareSerial Sbus = SoftwareSerial(9,8);
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
  sbus_data[3] = sbus_channels[1] >> 5 | sbus_channels[2] << 6;
  sbus_data[4] = sbus_channels[2] >> 2;
  sbus_data[5] = sbus_channels[2] >> 10 | sbus_channels[3] << 1;
  sbus_data[6] = sbus_channels[3] >> 7 | sbus_channels[4] << 4;
  sbus_data[7] = sbus_channels[4] >> 4 | sbus_channels[5] << 7;
  Sbus.write(sbus_data, 25);  
}

void zero_sbus(){
  uint16_t sbus_channels[6] = {0, 0, 0, 0, 0};
  send_sbus(sbus_channels);
}

// -------------- Tests ------------------------


void test_enable_disable(){
  Debug.println("Testing enable disable");
  pinMode(ENABLE_VALVE_DO, INPUT);
  pinMode(IGNITER_DO, INPUT);
  enable();
  int enable_valve = digitalRead(ENABLE_VALVE_DO);
  int igniter = digitalRead(IGNITER_DO);
  expect(enable_valve == HIGH && igniter == HIGH);
  disable();
  enable_valve = digitalRead(ENABLE_VALVE_DO);
  igniter = digitalRead(IGNITER_DO);
  expect(enable_valve == LOW && igniter == LOW);
  enable();
  enable_valve = digitalRead(ENABLE_VALVE_DO);
  igniter = digitalRead(IGNITER_DO);
  expect(enable_valve == HIGH && igniter == HIGH);
  disable();
}


void test_flamethrower(){
  Debug.println("Testing flamethrower");
  pinMode(PROPANE_DO, INPUT);
  enable();
  zero_sbus();
  uint16_t sbus_channels_flame[6] = {0, 0, 0, 1000, 0};
  for (int i = 0; i < 10; i++){
    send_sbus(sbus_channels_flame);
  }
  delay(100);
  int propane  = digitalRead(PROPANE_DO);
  expect(propane == HIGH);
  disable();
}

void setup() {
  // Start sbus
  Sbus.begin(100000);
  Debug.begin(115200);
  // Simulates enable/disable RC pulse
  myservo.attach(WEAPONS_ENABLE_PIN);

  test_enable_disable();
  test_flamethrower();
}

void loop() {
}
