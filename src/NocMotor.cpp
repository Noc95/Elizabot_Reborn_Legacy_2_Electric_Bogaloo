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

    return;
}

void NocMotor::init() {
    pinMode(motorPinA, OUTPUT);
    pinMode(motorPinB, OUTPUT);

    pinMode(encoderPinA, INPUT);
    pinMode(encoderPinB, INPUT);

    // gpio_t gpio; // Declare a gpio_t object
    // gpio_init_in(&gpio, encoderPinA); // Initialize the GPIO for input

    // gpio_init(encoderPinA); // Replace 2 with your pin number
    // gpio_set_dir(2, GPIO_IN);
    // gpio_init(encoderPinB); // Replace 3 with your pin number
    // gpio_set_dir(3, GPIO_IN);
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
        lastTime = currentTime;
        currentTime = micros();
        deltaTime = currentTime - lastTime;

        rotationSpeed = ((encoderCount-lastEncoderCount) / 180.0) * (1000000.0 / deltaTime); // Gives RPS, Rounds Per Second
        
        lastEncoderCount = encoderCount;
    }
    else if (micros() - lastTime > maxTimeBetweenEncoderUpdate) {
        rotationSpeed = 0;
    }
    return;
}
