// #include <Arduino.h>
// #include <Arduino_LSM6DS3.h>
// #include "NocMotor.h"


// NocMotor::NocMotor(int maxRPM, int motorPinA, int motorPinB, int encoderPinA, int encoderPinB) 
// {
//     // Callback instance mumbo jumbo
//     // instance = this;

//     this->maxRPM = maxRPM;
//     this->motorPinA = motorPinA;
//     this->motorPinB = motorPinB;
//     this->encoderPinA = encoderPinA;
//     this->encoderPinB = encoderPinB;

//     // init();
    
//     return;
// }

// void NocMotor::init() {
//     pinMode(motorPinA, OUTPUT);
//     pinMode(motorPinB, OUTPUT);

//     // Attach Encoder Interrupt
//     attachInterrupt(digitalPinToInterrupt(encoderPinA), encoderCallback, CHANGE);
// }

// // void NocMotor::encoderCallback() {
// //     if (instance) {
// //         instance->handleEncoderInterrupt();
// //     }
// // }

// // void handleEncoderInterrupt() {
// //     Serial.print("Interrupted!");
// // }

// void NocMotor::test(int motorSignal) {
//     analogWrite(motorPinA, motorSignal);
//     analogWrite(motorPinB, 0);
// }