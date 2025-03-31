
#ifndef NOCMOTOR_H
#define NOCMOTOR_H


class NocMotor
{
    
public:

    int dummy = 0;

    bool enabled = false;

    int direction;  // -1 for left wheel, 1 for the right wheel

    int maxRPS;
    int motorPinA;
    int motorPinB;
    int encoderPinA;
    int encoderPinB;
    int encoderCount = 0;

    float rotationSpeed = 0;
    float PIDsetPoint = 0;
    
    void init();
    void handleEncoderInterrupt();
    void calculateRotationSpeed();
    void RunMotorControl();
    


    NocMotor(int maxRPS, int motorPinA, int motorPinB, int encoderPinA, int encoderPinB, int direction);


private:
    
    unsigned int rotationLastTime = 0;
    unsigned int rotationCurrentTime = 0;
    unsigned int maxTimeBetweenEncoderUpdate = 100000;

    int rollingFilterSize = 20;
    float rotationSpeedBuffer[20];  // Must be same as rollingFilterSize
    int rollingFilterIndex = 0;
    float rollingFilterSum = 0;

    unsigned int PIDlastTime = 0;
    unsigned int PIDcurrentTime = 0;

    int lastEncoderCount = 0;

    void calculateMotorPID();

    float kp = 90;  // 25!
    float ki = 400;  // 400
    float kd = 0.001;

    float alphaD = 0.1; // Filter for the D part of the PID controller
    float lastD = 0;

    float PIDinput = 0;
    float lastPIDinput = 0;
    
    float cumError = 0;
    float lastError = 0;

    float P = 0;
    float I = 0;
    float D = 0;
    
    float maxcontrolSignalAbs = 255;
    float controlSignal = 0;
    

};




#endif