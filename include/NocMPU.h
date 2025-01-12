
#ifndef NOCMPU_H
#define NOCMPU_H

#include <Adafruit_MPU6050.h>


class NocMPU
{
    
public:
   
    NocMPU();
    float angle;    // Angle in degrees

    void initializeMPU();
    
    void calculateAngle();    



private:

    Adafruit_MPU6050 mpu;
    
};




#endif