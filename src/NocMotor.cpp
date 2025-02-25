#include <Arduino.h>
#include <Arduino_LSM6DS3.h>
#include "NocMotor.h"
#include "NocPID.h"


NocMotor::NocMotor(int maxRPM, int motorPinA, int motorPinB, int encoderPinA, int encoderPinB) 
{

    this->maxRPM = maxRPM;
    this->motorPinA = motorPinA;
    this->motorPinB = motorPinB;
    this->encoderPinA = encoderPinA;
    this->encoderPinB = encoderPinB;

    // this->pid = pid;
    
    return;
}

void NocMotor::init() {
    pinMode(motorPinA, OUTPUT);
    pinMode(motorPinB, OUTPUT);



    // Attach Encoder Interrupt
    // attachInterrupt(digitalPinToInterrupt(encoderPinA), encoderCallback, CHANGE);
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
        lastEncoderCount = encoderCount;
        lastTime = currentTime;
        currentTime = micros();
        deltaTime = currentTime - lastTime;

        rotationSpeed = ((encoderCount-lastEncoderCount) / 180.0) * (1000000.0 / deltaTime); // Gives RPS, Rounds Per Second
    }
    else if (micros() - lastTime > maxTimeBetweenEncoderUpdate) {
        rotationSpeed = 0;
    }
    return;
}
