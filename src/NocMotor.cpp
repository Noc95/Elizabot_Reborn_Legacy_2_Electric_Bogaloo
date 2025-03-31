#include <Arduino.h>
#include <Arduino_LSM6DS3.h>
#include "NocMotor.h"
#include "NocPID.h"


NocMotor::NocMotor(int maxRPS, int motorPinA, int motorPinB, int encoderPinA, int encoderPinB, int direction) 
{

    this->maxRPS = maxRPS;
    this->motorPinA = motorPinA;
    this->motorPinB = motorPinB;
    this->encoderPinA = encoderPinA;
    this->encoderPinB = encoderPinB;
    this->direction = direction;

    return;
}

void NocMotor::init() {
    pinMode(motorPinA, OUTPUT);
    pinMode(motorPinB, OUTPUT);

    pinMode(encoderPinA, INPUT);
    pinMode(encoderPinB, INPUT);

    for (int i = 0; i < rollingFilterSize; i++) {
        rotationSpeedBuffer[i] = 0; // Initialize all readings to 0
    }
}

void NocMotor::handleEncoderInterrupt() {
  if (digitalRead(encoderPinA) == digitalRead(encoderPinB)) {
    encoderCount++;
  } else {
    encoderCount--;
  }
}

void NocMotor::calculateRotationSpeed() {
    
    // If the encoder has a new value -> read time difference and calculate rotation speed
    if (encoderCount != lastEncoderCount) {
        rotationLastTime = rotationCurrentTime;
        rotationCurrentTime = micros();
        unsigned int deltaTime = rotationCurrentTime - rotationLastTime;

        float rotationSpeedTemp = direction * ((encoderCount-lastEncoderCount) / 180.0) * (1000000.0 / deltaTime); // Gives RPS, Rounds Per Second
        // Serial.println(rotationSpeedTemp);
        lastEncoderCount = encoderCount;
        
        rollingFilterSum -= rotationSpeedBuffer[rollingFilterIndex];  // Remove the oldest reading from sum
        rotationSpeedBuffer[rollingFilterIndex] = rotationSpeedTemp;  // Store the new reading
        rollingFilterSum += rotationSpeedTemp;  // Add the new reading to sum

        rollingFilterIndex = (rollingFilterIndex + 1) % rollingFilterSize;  // Move index circularly

        rotationSpeed = rollingFilterSum / rollingFilterSize;  // Compute the rolling average
    }
    else if (micros() - rotationLastTime > maxTimeBetweenEncoderUpdate) {
        rotationSpeed = 0;
    }
    return;
}

void NocMotor::calculateMotorPID() {
    if (enabled == false){    // Returns if PID is disabled
        // controlSignalValid = false;
        cumError = 0;
        P = 0;
        I = 0;
        D = 0;
        // PIDlastTime = 0;
        controlSignal = 0;

        return;
      }
  
      if (PIDlastTime == 0) {
        PIDlastTime = micros();  // If PID is reset then make sure there is no big difference in delta time making a fuss
      }
      PIDcurrentTime = micros();            // possible with millis if fast frequency???
      float deltaTime = (PIDcurrentTime - PIDlastTime);      // Time difference in milliseconds
      PIDlastTime = PIDcurrentTime;
      
      float error = PIDsetPoint - rotationSpeed;    // Setpoint - Input
  
      P = kp * error;
      
      if (!(abs(controlSignal) >= maxcontrolSignalAbs)) {
          cumError += error * deltaTime / 1000000;  // Divided by 1000000 because of micro seconds
          if (isnan(cumError)) {
            cumError = 0;
          }
          I = ki * cumError;
      }
    //   Serial.print(" cumError: ");
    //   Serial.print(cumError);
    //   Serial.print(" error: ");
    //   Serial.print(error);
    //   Serial.print(" deltaTime: ");
    //   Serial.print(deltaTime);
    //   Serial.println(" ");
  
      D = kd * ((error - lastError) / (deltaTime / 1000000));
      if (isnan(D)) {
        D = 0;
      }
      D = alphaD * D + (1 - alphaD) * lastD;    // Low-pass filter
      lastD = D;
      
      float tempControlSignal = P + I + D;
    
      // Serial.print(" Delta time: ");
    //   Serial.print((PIDcurrentTime));
    //   Serial.print("  ");
    //   Serial.println((PIDlastTime));
    //   Serial.print(PIDsetPoint);
    //   Serial.print(" P: ");
    //   Serial.print(P);
    //   Serial.print(" I: ");
    //   Serial.print(I);
    //   Serial.print(" D: ");
    //   Serial.print(D);
    //   Serial.println(" ");
    
      
      lastPIDinput = PIDinput;
      
      if (tempControlSignal > maxcontrolSignalAbs)
          controlSignal =  maxcontrolSignalAbs;
      else if (tempControlSignal < -maxcontrolSignalAbs)
          controlSignal = -maxcontrolSignalAbs;
      else
          controlSignal = tempControlSignal;
  
    //   Serial.println(controlSignal);

      return;
}

void NocMotor::RunMotorControl() {
    
    calculateMotorPID();

    if (enabled == true) {
        if (direction == 1) {
            if (controlSignal > 0) {
                analogWrite(motorPinA, abs(controlSignal));
                analogWrite(motorPinB, 0);
            }
            else if (controlSignal < 0) {
                analogWrite(motorPinA, 0);
                analogWrite(motorPinB, abs(controlSignal));
            }
            else {
                digitalWrite(motorPinA, LOW);
                digitalWrite(motorPinB, LOW);
            }
        }
        else if (direction == -1) {
            if (controlSignal > 0) {
                analogWrite(motorPinA, 0);
                analogWrite(motorPinB, abs(controlSignal));
            }
            else if (controlSignal < 0) {
                analogWrite(motorPinA, abs(controlSignal));
                analogWrite(motorPinB, 0);
            }
            else {
                digitalWrite(motorPinA, LOW);
                digitalWrite(motorPinB, LOW);
            }
        }
    }
    else {
        digitalWrite(motorPinA, LOW);
        digitalWrite(motorPinB, LOW);
    }
}