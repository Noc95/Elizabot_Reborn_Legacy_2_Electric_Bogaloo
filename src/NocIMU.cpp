#include <Arduino.h>
#include "NocIMU.h"
#include <Arduino_LSM6DS3.h>
#include <vector>
using namespace std;


NocIMU::NocIMU()
{
    // this->imu = IMU;
}

void NocIMU::initializeIMU() {

    if (!IMU.begin()) {
        Serial.println("Failed to initialize IMU");
        while (1) {
            delay(10);
        }
    }
    Serial.println("IMU initialized");

    // imu.setAccelerometerRange(MPU6050_RANGE_8_G);
    // imu.setGyroRange(MPU6050_RANGE_500_DEG);
    // imu.setFilterBandwidth(MPU6050_BAND_21_HZ);

    // gravity.x = 0;
    // gravity.y = 0;
    // gravity.z = 1.0;

    delay(3);
}

void NocIMU::calibrateIMU() {

    IMU.readGyroscope(gyro.x, gyro.y, gyro.z);
    IMU.readAcceleration(acc.x, acc.y, acc.z);
    
    // accXCalibrationValue = a.acceleration.x;
    // accYCalibrationValue = a.acceleration.y;
    // accZCalibrationValue = a.acceleration.z;

    gyroXCalibrationValue = -gyro.x;
    gyroYCalibrationValue = -gyro.y;
    gyroZCalibrationValue = -gyro.z;

}

void NocIMU::readIMUValues() {

    // Read values from imu
    IMU.readGyroscope(gyro.x, gyro.y, gyro.z);
    IMU.readAcceleration(acc.x, acc.y, acc.z);

    /*
    Acc X positiv mot kontakt
    Acc y positiv mot höger (sett mot kontakten)
    Acc x positiv nedåt

    Gyro x roteras korrekt
    Gyro y roteras korrekt
    Gyro z roteras korrekt
    */

    acc.x -= accXCalibrationValue;
    acc.y -= accYCalibrationValue;
    acc.z -= accZCalibrationValue;

    gyro.x -= gyroXCalibrationValue;
    gyro.y -= gyroYCalibrationValue;
    gyro.z -= gyroZCalibrationValue;
    
    // DEGUB
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

void NocIMU::elizabotCalculateAngle() {
    
    currentTime = micros();
    deltaTime = float(currentTime - lastTime) / 1000000.0; // Delta time in seconds

    readIMUValues();
    
    // Calculate the rotation vector
    float omega = gyro.y * deltaTime;

    // Calculate acceleration angle
    float accAngle = atan2(-acc.x, acc.z);

    // Complementary filter
    angle = gyroTrust * (angle + gyro.y * deltaTime) + (1 - gyroTrust) * accAngle;
    
    // DEBUG 
    // if (gravity.x < 0) 
    //   Serial.print("  Gravity x: ");
    // else
    //   Serial.print("  Gravity x:  ");
    // Serial.print(gravity.x);
    // if (gravity.y < 0) 
    //   Serial.print("  Gravity y: ");
    // else
    //   Serial.print("  Gravity y:  ");
    // Serial.print(gravity.y);
    // if (gravity.z < 0) 
    //   Serial.print("  Gravity z: ");
    // else
    //   Serial.print("  Gravity z:  ");
    // Serial.print(gravity.z);
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

    lastTime = micros();

    return;
}
