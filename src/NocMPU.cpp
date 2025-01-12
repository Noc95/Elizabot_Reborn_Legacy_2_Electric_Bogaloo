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
    // TODO: Rotate vector with gyro values
    // TODO: Test angle calculation

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

/*
float SteeringController::calculateZGyroBias() {
  Serial.print("Starting calibration of gyro!");
  int startTime = millis();
  int elapsedTime = startTime;
  int targetTime = 1000;

  float lastValue = 0;
  float measuredValue = 0;

  while (true) {
    //sensors_event_t a, g, temp;
    //SteeringController::mpu.getEvent(&a, &g, &temp);
    readMPUValues();

    measuredValue = float(int(g.gyro.z * 100)) / 100;   // Rounding the value to two decimals


    Serial.print("Measured value: ");
    Serial.print(measuredValue);
    Serial.print("  Elapsed time: ");
    Serial.print(elapsedTime);
    Serial.print("  Time Difference: ");
    Serial.println(elapsedTime - startTime);

    
    if (measuredValue != lastValue)               // Restarting test if values change
      startTime = elapsedTime;
    
    if ((elapsedTime - startTime) > targetTime)   // Return if target time is reached without interruption
      return measuredValue;

    lastValue = measuredValue;
    elapsedTime = millis();
  }

  Serial.println("Calibrated!");
}
*/