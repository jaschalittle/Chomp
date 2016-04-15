int gyroResult[3], accelResult[3];
float timeStep = 0.02;
float biasGyroX, biasGyroY, biasGyroZ, biasAccelX, biasAccelY, biasAccelZ;
float pitchGyro = 0;
float pitchAccel = 0;
float pitchPrediction = 0; //Output of Kalman filter
float rollGyro = 0;
float rollAccel = 0;
float rollPrediction = 0;  //Output of Kalman filter
float giroVar = 0.1;
float deltaGiroVar = 0.1;
float accelVar = 5;
float Pxx = 0.1; // angle variance
float Pvv = 0.1; // angle change rate variance
float Pxv = 0.1; // angle and angle change rate covariance
float kx, kv;
unsigned long timer;

void loop() {
  timer = millis();
  getGyroscopeReadings(gyroResult);
  getAccelerometerReadings(accelResult);
  
  pitchAccel = atan2((accelResult[1] - biasAccelY) / 256, (accelResult[2] - biasAccelZ) / 256) * 360.0 / (2*PI);
  pitchGyro = pitchGyro + ((gyroResult[0] - biasGyroX) / 14.375) * timeStep;
  pitchPrediction = pitchPrediction + ((gyroResult[0] - biasGyroX) / 14.375) * timeStep;
  
  rollAccel = atan2((accelResult[0] - biasAccelX) / 256, (accelResult[2] - biasAccelZ) / 256) * 360.0 / (2*PI);
  rollGyro = rollGyro - ((gyroResult[1] - biasGyroY) / 14.375) * timeStep; 
  rollPrediction = rollPrediction - ((gyroResult[1] - biasGyroY) / 14.375) * timeStep;
  
  Pxx += timeStep * (2 * Pxv + timeStep * Pvv);
  Pxv += timeStep * Pvv;
  Pxx += timeStep * giroVar;
  Pvv += timeStep * deltaGiroVar;
  kx = Pxx * (1 / (Pxx + accelVar));
  kv = Pxv * (1 / (Pxx + accelVar));
  
  pitchPrediction += (pitchAccel - pitchPrediction) * kx;
  rollPrediction += (rollAccel - rollPrediction) * kx;
  
  Pxx *= (1 - kx);
  Pxv *= (1 - kx);
  Pvv -= kv * Pxv;
  
  Serial.print(pitchGyro);
  Serial.print("\t");
  Serial.print(pitchAccel);
  Serial.print("\t");
  Serial.print(pitchPrediction);
  Serial.print("\t"); 
  Serial.print(rollGyro);
  Serial.print("\t");
  Serial.print(rollAccel);
  Serial.print("\t");
  Serial.print(rollPrediction);
  Serial.print("\n");
  
  timer = millis() - timer;
  timer = (timeStep * 1000) - timer; 
  delay(timer);
}