#include <Arduino.h>
#include "NocPID.h"


NocPID::NocPID(float kp, float ki, float kd, float maxOutputAbs) 
{
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
    this->maxOutputAbs = maxOutputAbs;
}


void NocPID::calculate() 
{

    unsigned int currentTime = millis();
    float deltaTime = (currentTime - lastTime);      // Time difference in milliseconds
    lastTime = currentTime;
    
    float error = setPoint - input;

    P = kp * error;
    
    if (!(abs(output) >= maxOutputAbs)) {
        cumError += error * deltaTime / 1000;
        I = kp * (1 / ki) * cumError;
    }

    D = kp * kd * ((input - lastInput) / deltaTime);
    
    float tempOutput = P + I + D;
    /*
    Serial.print(setPoint);
    Serial.print(" P: ");
    Serial.print(P);
    Serial.print(" I: ");
    Serial.print(I);
    Serial.print(" D: ");
    Serial.print(D);
    Serial.print(" ");
    */
    lastInput = input;
    
    if (tempOutput > maxOutputAbs)
        output =  maxOutputAbs;
    else if (tempOutput < -maxOutputAbs)
        output = -maxOutputAbs;
    else
        output = tempOutput;
}

/*
float SteeringController::calculateZGyroBias() {
  Serial.print("Starting calibration of gyro!");
  int startTime = millis();
  int elapsedTime = startTime;
  int targetTime = 1000;

  float lastValue = 0;
  float measuredValue = 0;

  while (true) {
    //sensors_event_t a, g, temp;
    //SteeringController::mpu.getEvent(&a, &g, &temp);
    readMPUValues();

    measuredValue = float(int(g.gyro.z * 100)) / 100;   // Rounding the value to two decimals


    Serial.print("Measured value: ");
    Serial.print(measuredValue);
    Serial.print("  Elapsed time: ");
    Serial.print(elapsedTime);
    Serial.print("  Time Difference: ");
    Serial.println(elapsedTime - startTime);

    
    if (measuredValue != lastValue)               // Restarting test if values change
      startTime = elapsedTime;
    
    if ((elapsedTime - startTime) > targetTime)   // Return if target time is reached without interruption
      return measuredValue;

    lastValue = measuredValue;
    elapsedTime = millis();
  }

  Serial.println("Calibrated!");
}
*/