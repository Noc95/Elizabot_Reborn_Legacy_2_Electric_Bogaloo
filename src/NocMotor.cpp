#include <Arduino.h>
#include <Arduino_LSM6DS3.h>
#include "NocMotor.h"
#include "NocPID.h"


NocMotor::NocMotor(int maxRPM, int motorPinA, int motorPinB, int encoderPinA, int encoderPinB, int direction) 
{

    this->maxRPM = maxRPM;
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
          I = ki * cumError;
      }
  
      D = kp * kd * ((PIDinput - lastPIDinput) / deltaTime);
      
      float tempControlSignal = P + I + D;
      
      // Serial.print(" Delta time: ");
      // Serial.print(deltaTime);
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
  
    //   outputValid = true;
  
      return;
}

void NocMotor::RunMotorControl() {
    
    // if (enabled == false) {
    //     lastTime = micros();
    // }
    // else {
    // PIDlastTime = PIDcurrentTime;
    // // }
    // PIDcurrentTime = micros();
    // unsigned int deltaTime = PIDcurrentTime - PIDlastTime;

    calculateMotorPID();

    if (enabled == true) {
        if (direction != 0) {
            if (controlSignal < 0) {
                analogWrite(motorPinA, abs(controlSignal));
                analogWrite(motorPinB, 0);
            }
            else if (controlSignal > 0) {
                analogWrite(motorPinA, 0);
                analogWrite(motorPinB, abs(controlSignal));
            }
            else {
                digitalWrite(motorPinA, LOW);
                digitalWrite(motorPinB, LOW);
            }
        }
        else {
            if (controlSignal > 0) {
                analogWrite(motorPinA, 0);
                analogWrite(motorPinB, abs(controlSignal));
            }
            else if (controlSignal < 0) {
                analogWrite(motorPinB, abs(controlSignal));
                analogWrite(motorPinA, 0);
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