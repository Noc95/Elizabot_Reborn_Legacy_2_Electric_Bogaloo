
#ifndef NOCMOTOR_H
#define NOCMOTOR_H


class NocMotor
{
    
public:

    int dummy = 0;

    bool enabled = false;

    int maxRPM;
    int motorPinA;
    int motorPinB;
    int encoderPinA;
    int encoderPinB;
    int encoderCount = 0;

    float rotationSpeed = 0;
    
    void init();
    void handleEncoderInterrupt();
    void calculateRotationSpeed();



    NocMotor(int maxRPM, int motorPinA, int motorPinB, int encoderPinA, int encoderPinB);


private:
    
    unsigned int lastTime;
    unsigned int currentTime;
    unsigned int deltaTime;
    unsigned int maxTimeBetweenEncoderUpdate = 100000;

    int lastEncoderCount = 0;

    // NocPID pid;
    

};




#endif