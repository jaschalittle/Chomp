/*
  Analog input, analog output, serial output

 Reads an analog input pin, maps the result to a range from 0 to 255
 and uses the result to set the pulsewidth modulation (PWM) of an output pin.
 Also prints the results to the serial monitor.

 The circuit:
 * potentiometer connected to analog pin 0.
   Center pin of the potentiometer goes to the analog pin.
   side pins of the potentiometer go to +5V and ground
 * LED connected from digital pin 9 to ground

 created 29 Dec. 2008
 modified 9 Apr 2012
 by Tom Igoe

 This example code is in the public domain.

 */
const int analogInPin = A15;  // Analog input pin that the potentiometer is attached to
const int analogOutPin = 5; // Analog output pin that the LED is attached to

const int datapoints = 2000;
short data[datapoints];
int collectData(int microseconds_to_collect) {
  long stop_time = micros() + microseconds_to_collect;
  int read_time;
  int datapoints_collected = 0;
  for (int i = 0; i < datapoints; i++) {
    if (micros() < stop_time) {
      read_time = micros();
      data[i] = analogRead(analogInPin);
      read_time = micros() - read_time;
      delayMicroseconds(1000 - read_time);
      datapoints_collected++;
    }
  }
  return datapoints_collected;
}

// These constants won't change.  They're used to give names
// to the pins used:

int sensorValue = 0;        // value read from the pot
int counts = 0;        // value read from the pot
int outputValue = 0;        // value output to the PWM (analog out)
int sensor_min = 0.5 / 5 * 1023;
int sensor_max = 4.5 / 5 * 1023;

#define FUTABA_CH1_PIN 3
#define FUTABA_CH2_PIN 20
#define FUTABA_CH5_PIN 21

enum RCinterrupts {
    LEFT_RC = digitalPinToInterrupt(FUTABA_CH1_PIN),
    RIGHT_RC = digitalPinToInterrupt(FUTABA_CH2_PIN),
    TARGETING_ENABLE = digitalPinToInterrupt(FUTABA_CH5_PIN)
};

// initialize PWM vals to neutral values
static volatile uint32_t LEFT_RC_pwm_val = 1520;
static volatile uint32_t LEFT_RC_prev_time = 0;
static volatile uint32_t RIGHT_RC_pwm_val = 1520;
static volatile uint32_t RIGHT_RC_prev_time = 0;
static volatile uint32_t TARGETING_ENABLE_pwm_val = 1520;
static volatile uint32_t TARGETING_ENABLE_prev_time = 0;

void LEFT_RC_rising();
void LEFT_RC_falling();
void RIGHT_RC_rising();
void RIGHT_RC_falling();
void TARGETING_ENABLE_rising();
void TARGETING_ENABLE_falling();

// Forgive me, I know not what I do.
#define CREATE_RISING_ISR( rc_interrupt )\
void rc_interrupt ## _rising() {\
  attachInterrupt(rc_interrupt, rc_interrupt ## _falling, FALLING);\
  rc_interrupt ## _prev_time = micros();\
}

#define CREATE_FALLING_ISR( rc_interrupt )\
void rc_interrupt ## _falling() {\
  attachInterrupt(rc_interrupt, rc_interrupt ## _rising, RISING);\
  rc_interrupt ## _pwm_val = micros() - rc_interrupt ## _prev_time;\
}

CREATE_FALLING_ISR(LEFT_RC);
CREATE_RISING_ISR(LEFT_RC);
CREATE_FALLING_ISR(RIGHT_RC);
CREATE_RISING_ISR(RIGHT_RC);
CREATE_FALLING_ISR(TARGETING_ENABLE);
CREATE_RISING_ISR(TARGETING_ENABLE);

void attachRCInterrupts(){
  attachInterrupt(LEFT_RC, LEFT_RC_rising, RISING);
  attachInterrupt(RIGHT_RC, RIGHT_RC_rising, RISING);
  attachInterrupt(TARGETING_ENABLE, TARGETING_ENABLE_rising, RISING);
}

void setup() {
  // initialize serial communications at 9600 bps:
  Serial.begin(115200);
  attachRCInterrupts();
  pinMode(3, INPUT_PULLUP);
  pinMode(20, INPUT_PULLUP);
  pinMode(21, INPUT_PULLUP);
}

int min_sensor = 150;
void loop() {
  // read the analog in value:
//  int datapoints_collected = collectData(1000000);
//  long pre_read = micros();
//  counts = analogRead(analogInPin);
//  int pressure = (int16_t) (counts - 81) * 11 / 18;
//  int angle = (int16_t) (counts - 102) * 11 / 25;
//  if (counts > min_sensor) {min_sensor = counts;}
//  sensorValue = map(sensorValue, 102, 920, 0, 359);
//  sensorValue = map(sensorValue, 114, 932, 0, 359);
//  pre_read = micros() - pre_read;
  // map it to the range of the analog out:
//  outputValue = map(sensorValue, sensor_min, sensor_max, 0, 180);
  // change the analog out value:
//  analogWrite(analogOutPin, outputValue);

  // print the results to the serial monitor:
//  for (int i = 0; i < datapoints_collected; i++) {
//    Serial.print("sensor = ");
//    Serial.println(data[i]);
//  }
  
//  Serial.print("sensor = ");
//  Serial.print(counts);
//  Serial.print("\t time = ");
//  Serial.println(pre_read);

  Serial.print(LEFT_RC_pwm_val);
  Serial.print("\t");
  Serial.println(RIGHT_RC_pwm_val);

  // wait 2 milliseconds before the next loop
  // for the analog-to-digital converter to settle
  // after the last reading:
  delay(10);
}
