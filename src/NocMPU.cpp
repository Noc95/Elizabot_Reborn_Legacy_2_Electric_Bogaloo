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

    delay(3);
}

void NocMPU::calibrateMPU() {

  // sensors_event_t a, g, temp;
  // mpu.getEvent(&a, &g, &temp);
  
  // float gyroXarr[1000];
  // float gyroYarr[1000];
  // float gyroZarr[1000];
  // int sameValueNeededCount = 100;

  // for (int i = 0; i < 1000; i++) {
  //   sensors_event_t a, g, temp;
  //   mpu.getEvent(&a, &g, &temp);
  

  //   if (g.gyro.x == gyroXarr[i-1] and g.gyro.y == gyroYarr[i-1 and g.gyro.z == gyroZarr[i-1]]) {

  //   }

  //   delay(10);
  // }

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  
  // accXCalibrationValue = a.acceleration.x;
  // accYCalibrationValue = a.acceleration.y;
  // accZCalibrationValue = a.acceleration.z;

  gyroXCalibrationValue = -g.gyro.x;
  gyroXCalibrationValue = -g.gyro.y;
  gyroXCalibrationValue = -g.gyro.z;

}

void NocMPU::readMpuValues() {
  // Read values from MPU
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  acc.x = a.acceleration.x - accXCalibrationValue;
  acc.y = a.acceleration.y - accYCalibrationValue;
  acc.z = a.acceleration.z - accZCalibrationValue;

  gyro.x = -g.gyro.x - gyroXCalibrationValue;
  gyro.y = -g.gyro.y - gyroYCalibrationValue;
  gyro.z = -g.gyro.z - gyroZCalibrationValue;

  // if (acc.x < 0)
  //   Serial.print("  Acc x: ");
  // else
  //   Serial.print("  Acc x:  ");
  // Serial.print(acc.x);
  // if (acc.y < 0)
  //   Serial.print("  Acc y: ");
  // else
  //   Serial.print("  Acc y:  ");
  // Serial.print(acc.y);
  // if (acc.z < 0)
  //   Serial.print("  Acc z: ");
  // else
  //   Serial.print("  Acc z:  ");
  // Serial.print(acc.z);

  // if (gyro.x < 0)
  //   Serial.print("  Gyro x: ");
  // else
  //   Serial.print("  Gyro x:  ");
  // Serial.print(gyro.x);
  // if (gyro.y < 0)
  //   Serial.print("  Gyro y: ");
  // else
  //   Serial.print("  Gyro y:  ");
  // Serial.print(gyro.y);
  // if (gyro.z < 0)
  //   Serial.print("  Gyro z: ");
  // else
  //   Serial.print("  Gyro z:  ");
  // Serial.print(gyro.z);

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

  deltaTime = float(currentTime - lastTime) / 1000000.0; // Delta time in seconds

  readMpuValues();
  
  Vector3 omega = gyro * deltaTime * 2; // Angular displacement (approx.)
  Vector3 predictedGravity = gravity + cross(omega, gravity);

  acc.normalize();
  
  gravity.x = (gyroTrust * predictedGravity.x) + (accTrust * acc.x);
  gravity.y = (gyroTrust * predictedGravity.y) + (accTrust * acc.y);
  gravity.z = (gyroTrust * predictedGravity.z) + (accTrust * acc.z);

  gravity.normalize();

  // if (gravity.y < 0) 
  //   Serial.print("  Gravity y: ");
  // else
  //   Serial.print("  Gravity y:  ");
  Serial.print(gravity.y);

  // if (acc.x < 0)
  //   Serial.print("  Acc x: ");
  // else
  //   Serial.print("  Acc x:  ");
  // Serial.print(acc.x);
  
  // if (gravity.z < 0) 
  //   Serial.print("  Gravity z: ");
  // else
  //   Serial.print("  Predicted Gravity z:  ");
  // Serial.print(predictedGravity.x);

  // Serial.print("  omega x:  ");
  // Serial.print(omega.x, 6);
  // Serial.print("  Delta time:  ");
  // Serial.print(deltaTime, 6);
  // Serial.print("  millis:  ");
  // Serial.print(millis());
  
  lastTime = micros();
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
  
  // Serial.print("  oldGravitylength: ");
  // Serial.print(gyroY);
  // Serial.print("  gravityX: ");
  // Serial.print(gravityX);
  // Serial.print("  gravityZ: ");
  // Serial.println(gravityZ);

  // float Glen = sqrt(pow(accX, 2) + pow(accZ, 2));   // Length of gravity vector
  
  // float angleRadian = asin(accY / Glen);

  // Conversion from rad to angle
  // angleRadian *= 180;
  // angle = angleRadian / PI;

  lastTime = micros();

  return;
}
*/
void NocMPU::calculateAngle() {
  // X-axis (Roll): Imagine the axis going through the front and back of the device (nose to tail). Positive roll rotates the device clockwise as viewed from the front.
  // Y-axis (Pitch): Think of the axis going through the sides (left to right). Positive pitch tilts the device upwards (nose up).
  // Z-axis (Yaw): Picture the axis going through the top and bottom. Positive yaw rotates the device clockwise from a top-down view.

  float angleRadian = atan2(gravity.y, gravity.x);
  // Conversion from rad to angle
  angleRadian *= 180;
  angle = angleRadian / PI;
}