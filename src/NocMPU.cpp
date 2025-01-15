#include <Arduino.h>
#include "NocMPU.h"
#include <Adafruit_MPU6050.h>
#include <vector>
using namespace std;


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

    gravity.x = 0;
    gravity.y = 0;
    gravity.z = 10.0;
}

void NocMPU::calibrateMPU() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  accXCalibrationValue = a.acceleration.x;
  accYCalibrationValue = a.acceleration.y;
  accZCalibrationValue = a.acceleration.z;

  gyroXCalibrationValue = g.gyro.x;
  gyroXCalibrationValue = g.gyro.y;
  gyroXCalibrationValue = g.gyro.z;
}

void NocMPU::readMpuValues() {
  // Read values from MPU
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  acc.x = a.acceleration.x;
  acc.y = a.acceleration.y;
  acc.z = a.acceleration.z;

  gyro.x = g.gyro.x;
  gyro.y = g.gyro.y;
  gyro.z = g.gyro.z;
}

// Utility function: Cross product of two vectors
NocMPU::Vector3 NocMPU::cross(const NocMPU::Vector3& a, const NocMPU::Vector3& b) {
    return {
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x
    };
}

void NocMPU::kalmanUpdate() {
  currentTime = micros();
  deltaTime = (lastTime - currentTime) / 1000000.0; // Delta time in seconds

  readMpuValues();
  
  Vector3 omega = gyro * deltaTime; // Angular displacement (approx.)
  Vector3 predictedGravity = gravity + cross(omega, gravity);                 // ??????????
  
  gravity.x = (gyroTrust * predictedGravity.x) + (accTrust * acc.x);
  gravity.y = (gyroTrust * predictedGravity.y) + (accTrust * acc.y);
  gravity.z = (gyroTrust * predictedGravity.z) + (accTrust * acc.z);

  gravity.normalize();

  if (gravity.x < 0) 
    Serial.print("  Gravity x: ");
  else
    Serial.print("  Gravity x:  ");
  Serial.print(gravity.x);
  
  if (gravity.z < 0) 
    Serial.print("  Gravity z: ");
  else
    Serial.print("  Gravity z:  ");
  Serial.println(gravity.z);
  
  lastTime = micros();
}

void NocMPU::calculateGravity()   // Made specifically to ca
{
  // TODO: Rotate vector with gyro values
  // TODO: Test angle calculation
  

  return;
}
/*
void NocMPU::calculateSimpleAngle() {
  
  currentTime = micros();
  deltaTime = lastTime - currentTime;

  readMpuValues();

  float oldGravityLength = sqrt(pow(gravityX, 2) + pow(gravityZ, 2));
  float oldGravityAngle = acos(gravityX / oldGravityLength);
  float newGravityAngle = oldGravityAngle + gyroY * (deltaTime / 1000000.0);

  // Rotate old gravity vector
  gravityX = oldGravityLength * cos(newGravityAngle);
  gravityZ = oldGravityLength * sin(newGravityAngle);
  
  Serial.print("  oldGravitylength: ");
  Serial.print(gyroY);
  Serial.print("  gravityX: ");
  Serial.print(gravityX);
  Serial.print("  gravityZ: ");
  Serial.println(gravityZ);

  // float Glen = sqrt(pow(accX, 2) + pow(accZ, 2));   // Length of gravity vector
  
  // float angleRadian = asin(accY / Glen);

  // Conversion from rad to angle
  // angleRadian *= 180;
  // angle = angleRadian / PI;

  lastTime = micros();

  return;
}
*/