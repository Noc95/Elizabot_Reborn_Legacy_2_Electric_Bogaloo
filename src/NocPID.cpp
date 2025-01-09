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
