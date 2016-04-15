/**
 * Sample program for the MPU9250 using SPI
 *
 * Sample rate of the AK8963 magnetometer is set at 100Hz. 
 * There are only two options: 8Hz or 100Hz so I've set it at 100Hz
 * in the library. This is set by writing to the CNTL1 register
 * during initialisation.
 *
 * Copyright (C) 2015 Brian Chen
 *
 * Open source under the MIT license. See LICENSE.txt.
 */

#include <SPI.h>
#include "MPU9250.h"

#define SPI_CLOCK 8000000  // 8MHz clock works.

#define SCK_PIN  52
#define SS_PIN   A11 
#define INT_PIN  3
#define LED      13

//#define WAITFORINPUT(){          \
//  while(!Serial.available()){};  \
//  while(Serial.available()){     \
//    Serial.read();               \
//  };                             \
//}                                \

MPU9250 mpu(SPI_CLOCK, SS_PIN);

uint32_t last_imu_read;

void setup() {
  Serial.begin(115200);

  pinMode(INT_PIN, INPUT);
//  pinMode(LED, OUTPUT);
//  digitalWrite(LED, HIGH);

//  SPI.setSCK(SCK_PIN);
  SPI.begin();

//  Serial.println("Press any key to continue");
//  WAITFORINPUT();

  mpu.init(true, true);

  uint8_t wai = mpu.whoami();
  if (wai == 0x71){
    Serial.println("Successful connection");
  }
  else{
    Serial.print("Failed connection: ");
    Serial.println(wai, HEX);
  }

//  uint8_t wai_AK8963 = mpu.AK8963_whoami();
//  if (wai_AK8963 == 0x48){
//    Serial.println("Successful connection to mag");
//  }
//  else{
//    Serial.print("Failed connection to mag: ");
//    Serial.println(wai_AK8963, HEX);
//  }
  delay(2000);
  mpu.calib_acc();
//  mpu.calib_mag();

//  Serial.println("Send any char to begin main loop.");
//  WAITFORINPUT();
  last_imu_read = micros();
}

float filtered_x_gyro = 0.0;
float filtered_y_gyro = 0.0;
float filtered_z_gyro = 0.0;
float filtered_y_accel = 0.0;
float filtered_z_accel = 0.0;
float velocity = 0.0;

void loop() {
  // various functions for reading
  // mpu.read_mag();
  // mpu.read_acc();
  // mpu.read_gyro();

  mpu.read_all();

  float cm_per_sec_sec = mpu.accel_data[1] * 9.8;
  uint32_t delta_t = micros() - last_imu_read;
  float timestep = (float) delta_t / 1000000;

  velocity += cm_per_sec_sec * timestep;
  
  filtered_x_gyro = 0.9 * filtered_x_gyro + 0.1 * mpu.gyro_data[0];
  filtered_y_gyro = 0.9 * filtered_y_gyro + 0.1 * mpu.gyro_data[1];
  filtered_z_gyro = 0.9 * filtered_z_gyro + 0.1 * mpu.gyro_data[2];
  filtered_y_accel = 0.99 * filtered_y_accel + 0.01 * mpu.accel_data[1];
  filtered_z_accel = 0.9 * filtered_z_accel + 0.1 * mpu.accel_data[2];
//  Serial.print(filtered_x_gyro, 4);   Serial.print('\t');
//  Serial.print(filtered_y_gyro, 4);   Serial.print('\t');
  Serial.print(filtered_z_gyro, 3);   Serial.print('\t'); Serial.print('\t');
//  Serial.print(mpu.gyro_data[0]);   Serial.print('\t');
//  Serial.print(mpu.gyro_data[1]);   Serial.print('\t');
//  Serial.print(mpu.gyro_data[2]);   Serial.print('\t');
//  Serial.print(mpu.accel_data[0], 4);  Serial.print('\t');
  Serial.print(filtered_y_accel, 6);  Serial.print('\t'); Serial.print('\t');
  Serial.print(mpu.accel_data[2], 6);  Serial.print('\t');
//  Serial.print(mpu.mag_data[0]);    Serial.print('\t');
//  Serial.print(mpu.mag_data[1]);    Serial.print('\t');
//  Serial.print(mpu.mag_data[2]);    Serial.print('\t');
//  Serial.print(mpu.temperature);
  Serial.println(velocity / millis() * 1000);

  delay(20);
}
