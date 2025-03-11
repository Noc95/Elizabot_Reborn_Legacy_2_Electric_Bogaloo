
#ifndef NOCMOTOR_H
#define NOCMOTOR_H


class NocMotor
{
    
public:

    int dummy = 0;

    bool enabled = false;

    int direction;  // -1 for left wheel, 1 for the right wheel

    int maxRPM;
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
    


    NocMotor(int maxRPM, int motorPinA, int motorPinB, int encoderPinA, int encoderPinB, int direction);


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

    float kp = 45;  // 50 ? 45 ?
    float ki = 90;  // 70 ? 120 ?
    float kd = 4.5;

    float PIDinput = 0;
    float lastPIDinput = 0;
    

    float cumError = 0;

    float P = 0;
    float I = 0;
    float D = 0;
    
    float maxcontrolSignalAbs = 255;
    float controlSignal = 0;
    

};




#endif