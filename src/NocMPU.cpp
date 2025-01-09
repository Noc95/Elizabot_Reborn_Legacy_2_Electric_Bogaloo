#include <Arduino.h>
#include "NocMPU.h"
#include <Adafruit_MPU6050.h>


NocMPU::NocMPU() 
{
    Adafruit_MPU6050 mpu;
    this->mpu = mpu;
}

void NocMPU::initializeMPU() {

    if (!mpu.begin()) {
        Serial.println("Failed to find MPU6050 chip");
        while (1) {
            delay(10);
        }
    }
    Serial.println("MPU6050 Found!");

    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
}

void NocMPU::calculateAngle() 
{
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    float Ax = a.acceleration.x;
    float Ay = a.acceleration.y;
    float Az = a.acceleration.z;

    float Gx = g.gyro.x;
    float Gy = g.gyro.y;
    float Gz = g.gyro.z;

    float Glen = sqrt(pow(Ax, 2) + pow(Ay, 2) + pow(Az, 2));

    float rotatedX;
    float rotatedY;
    float rotatedZ;

    float angleRadian = asin(Ay / Glen);

    // Conversion from rad to angle
    angleRadian *= 180;
    angle = angleRadian / PI;
}
