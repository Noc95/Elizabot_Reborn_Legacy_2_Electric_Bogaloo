#include <Arduino.h>
#include "NocPID.h"


NocPID::NocPID(float kp, float ki, float kd, float maxOutputAbs) 
{
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
    this->maxOutputAbs = maxOutputAbs;
    this->enabled = false;
}


void NocPID::calculate() 
{
    // TODO: Test PID controller

    if (enabled == false){    // Returns if PID is disabled
      outputValid = false;
      cumError = 0;
      P = 0;
      I = 0;
      D = 0;
      lastTime = 0;
      output = 0;

      return;
    }

    if (lastTime == 0) {
      lastTime = micros();  // If PID is reset then make sure there is no big difference in delta time making a fuss
    }
    unsigned int currentTime = micros();            // possible with millis if fast frequency???
    float deltaTime = (currentTime - lastTime);      // Time difference in milliseconds
    lastTime = currentTime;
    
    float error = setPoint - input;

    P = kp * error;
    
    if (!(abs(output) >= maxOutputAbs)) {
        cumError += error * deltaTime / 1000000;
        if (isnan(cumError)) {
            cumError = 0;
        }
        I = ki * cumError;
    }

    // D = kp * kd * ((input - lastInput) / deltaTime);
    D = kd * ((error - lastError) / (deltaTime / 1000000));
    if (isnan(D)) {
        D = 0;
    }
    D = alphaD * D + (1 - alphaD) * lastD;    // Low-pass filter
    lastD = D;
    
    float tempOutput = P + I + D;
    
    // Serial.print(" Delta time: ");
    // Serial.print(deltaTime);
    // Serial.print(setPoint);
    // Serial.print(" P: ");
    // Serial.print(P);
    // Serial.print(" I: ");
    // Serial.print(I);
    // Serial.print(" D: ");
    // Serial.print(D);
    // Serial.println(" ");
    
    lastInput = input;
    
    if (tempOutput > maxOutputAbs)
        output =  maxOutputAbs;
    else if (tempOutput < -maxOutputAbs)
        output = -maxOutputAbs;
    else
        output = tempOutput;

    outputValid = true;

    return;
}

