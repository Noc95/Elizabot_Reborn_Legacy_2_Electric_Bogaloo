
#ifndef NOCPID_H
#define NOCPID_H


class NocPID
{
    
public:

    bool enabled;       // Enables / disables the PID controller
    bool outputValid;   // True if the output value is to be trusted

    float kp = 1;
    float ki = 0;
    float kd = 0;

    float input = 0;
    float output = 0;
    float setPoint = 0;

    float maxOutputAbs;
    
    void calculate();

    NocPID(float kp, float ki, float kd, float maxOutputAbs);

private:

    int dummy;  // For some reason this line is necessary to make the next line work??

    unsigned int lastTime;
    float lastInput;

    float P;
    float I;
    float D;

    float cumError = 0;
    float lastError = 0;
    float alphaD = 0.1; // Filter thing
    float lastD = 0; // Filter thing
};




#endif