// #include <Arduino.h>
// #include "NocMPU.h"
// #include <Adafruit_MPU6050.h>
// #include <vector>
// using namespace std;


// NocMPU::NocMPU() 
// {
//     Adafruit_MPU6050 mpu;
//     this->mpu = mpu;
// }

// void NocMPU::initializeMPU() {

//     if (!mpu.begin()) {
//         Serial.println("Failed to find MPU6050 chip");
//         while (1) {
//             delay(10);
//         }
//     }
//     Serial.println("MPU6050 Found!");

//     mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
//     mpu.setGyroRange(MPU6050_RANGE_500_DEG);
//     mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

//     gravity.x = 0;
//     gravity.y = 0;
//     gravity.z = 1.0;

//     delay(3);
// }

// void NocMPU::calibrateMPU() {

//   // sensors_event_t a, g, temp;
//   // mpu.getEvent(&a, &g, &temp);
  
//   // float gyroXarr[1000];
//   // float gyroYarr[1000];
//   // float gyroZarr[1000];
//   // int sameValueNeededCount = 100;

//   // for (int i = 0; i < 1000; i++) {
//   //   sensors_event_t a, g, temp;
//   //   mpu.getEvent(&a, &g, &temp);
  

//   //   if (g.gyro.x == gyroXarr[i-1] and g.gyro.y == gyroYarr[i-1 and g.gyro.z == gyroZarr[i-1]]) {

//   //   }

//   //   delay(10);
//   // }

//   sensors_event_t a, g, temp;
//   mpu.getEvent(&a, &g, &temp);
  
//   // accXCalibrationValue = a.acceleration.x;
//   // accYCalibrationValue = a.acceleration.y;
//   // accZCalibrationValue = a.acceleration.z;

//   gyroXCalibrationValue = -g.gyro.x;
//   gyroYCalibrationValue = -g.gyro.y;
//   gyroZCalibrationValue = -g.gyro.z;

// }

// void NocMPU::readMpuValues() {
//   // Read values from MPU
//   sensors_event_t a, g, temp;
//   mpu.getEvent(&a, &g, &temp);

//   acc.x = a.acceleration.x - accXCalibrationValue;
//   acc.y = a.acceleration.y - accYCalibrationValue;
//   acc.z = a.acceleration.z - accZCalibrationValue;

//   gyro.x = (-g.gyro.x - gyroXCalibrationValue);// * 500.0 / 32768.0;
//   gyro.y = (-g.gyro.y - gyroYCalibrationValue);// * 500.0 / 32768.0;
//   gyro.z = (-g.gyro.z - gyroZCalibrationValue);// * 500.0 / 32768.0;
  
//   // if (acc.x < 0)
//   //   Serial.print("  Acc x: ");
//   // else
//   //   Serial.print("  Acc x:  ");
//   // Serial.print(a.acceleration.x);
//   // if (acc.y < 0)
//   //   Serial.print("  Acc y: ");
//   // else
//   //   Serial.print("  Acc y:  ");
//   // Serial.print(a.acceleration.y);
//   // if (acc.z < 0)
//   //   Serial.print("  Acc z: ");
//   // else
//   //   Serial.print("  Acc z:  ");
//   // Serial.print(a.acceleration.z);

//   // if (gyro.x < 0)
//   //   Serial.print("  Gyro x: ");
//   // else
//   //   Serial.print("  Gyro x:  ");
//   // Serial.print(g.gyro.x);
//   // if (gyro.y < 0)
//   //   Serial.print("  Gyro y: ");
//   // else
//   //   Serial.print("  Gyro y:  ");
//   // Serial.print(g.gyro.y);
//   // if (gyro.z < 0)
//   //   Serial.print("  Gyro z: ");
//   // else
//   //   Serial.print("  Gyro z:  ");
//   // Serial.print(g.gyro.z);

// }

// // Utility function: Cross product of two vectors
// NocMPU::Vector3 NocMPU::cross(const NocMPU::Vector3& a, const NocMPU::Vector3& b) {
//     return {
//         a.y * b.z - a.z * b.y,
//         a.z * b.x - a.x * b.z,
//         a.x * b.y - a.y * b.x
//     };
// }

// // Function to multiply a 3x3 matrix with a Vector3
// NocMPU::Vector3 NocMPU::multiplyMatrixVector(float matrix[3][3], NocMPU::Vector3 vector) {
//     Vector3 result;

//     // Perform the matrix-vector multiplication
//     result.x = matrix[0][0] * vector.x + matrix[0][1] * vector.y + matrix[0][2] * vector.z;
//     result.y = matrix[1][0] * vector.x + matrix[1][1] * vector.y + matrix[1][2] * vector.z;
//     result.z = matrix[2][0] * vector.x + matrix[2][1] * vector.y + matrix[2][2] * vector.z;

//     return result;
// }

// // Utility function: Cross product of two vectors
// NocMPU::Vector3 NocMPU::rotateVector3(const NocMPU::Vector3& vector, const NocMPU::Vector3& rotationVector) {

//   Vector3 rotatedVector = vector;
//   // Vector3 rotatedVector = {0, 0, 1};
//   // Vector3 debugRotationVector = {0, PI/2, 0};

//   float theta = rotationVector.x;
//   float rotationMatrixX[3][3] = {
//       {cos(theta), 0, 0},
//       {0, cos(theta), -sin(theta)},  // Approximation: cos(theta) ≈ 1, sin(theta) ≈ theta
//       {0, sin(theta), cos(theta)}
//   };
//   // Rotate by x-axis
//   rotatedVector = multiplyMatrixVector(rotationMatrixX, rotatedVector);

//   theta = rotationVector.y;
//   float rotationMatrixY[3][3] = {
//       {cos(theta), 0, sin(theta)},   // Approximation: cos(theta) ≈ 1, sin(theta) ≈ theta
//       {0, cos(theta), 0},
//       {-sin(theta), 0, cos(theta)}
//   };
//   // Rotate by y-axis
//   rotatedVector = multiplyMatrixVector(rotationMatrixY, rotatedVector);

//   theta = rotationVector.z;
//   float rotationMatrixZ[3][3] = {
//       {cos(theta), -sin(theta), 0},  // Approximation: cos(theta) ≈ 1, sin(theta) ≈ theta
//       {sin(theta), cos(theta), 0},
//       {0, 0, cos(theta)}
//   };
//   // Rotate by z-axis
//   rotatedVector = multiplyMatrixVector(rotationMatrixZ, rotatedVector);

//   // Serial.print(" x: ");
//   // Serial.print(rotatedVector.x);
//   // Serial.print(" y: ");
//   // Serial.print(rotatedVector.y);
//   // Serial.print(" z: ");
//   // Serial.print(rotatedVector.z);
    
//   return rotatedVector;
// }

// void NocMPU::kalmanUpdate() {
//   currentTime = micros();

//   deltaTime = float(currentTime - lastTime) / 1000000.0; // Delta time in seconds

//   readMpuValues();
  
//   // Calculate the rotation vector
//   Vector3 omega = gyro * deltaTime; // Angular displacement (approx.)
  
//   // Rotate vector with forbidden algebra magic
//   Vector3 predictedGravity = gravity + cross(omega, gravity);

//   // OR Rotate vector with inefficient algebra "magic"
//   // Vector3 predictedGravity = rotateVector3(gravity, omega);

//   acc.normalize();
  
//   gravity.x = (gyroTrust * predictedGravity.x) + (accTrust * acc.x);
//   gravity.y = (gyroTrust * predictedGravity.y) + (accTrust * acc.y);
//   gravity.z = (gyroTrust * predictedGravity.z) + (accTrust * acc.z);

//   gravity.normalize();

//   // DEBUG 
//   // if (gravity.x < 0) 
//   //   Serial.print("  Gravity x: ");
//   // else
//   //   Serial.print("  Gravity x:  ");
//   // Serial.print(gravity.x);
//   // if (gravity.y < 0) 
//   //   Serial.print("  Gravity y: ");
//   // else
//   //   Serial.print("  Gravity y:  ");
//   // Serial.print(gravity.y);
//   // if (gravity.z < 0) 
//   //   Serial.print("  Gravity z: ");
//   // else
//   //   Serial.print("  Gravity z:  ");
//   // Serial.print(gravity.z);
//   // if (acc.x < 0)
//   //   Serial.print("  Acc x: ");
//   // else
//   //   Serial.print("  Acc x:  ");
//   // Serial.print(acc.x);
//   // if (gravity.z < 0) 
//   //   Serial.print("  Gravity z: ");
//   // else
//   //   Serial.print("  Predicted Gravity z:  ");
//   // Serial.print(predictedGravity.x);

//   lastTime = micros();
// }

// void NocMPU::calculateAngle() {

//   angle = atan2(-gravity.y, gravity.z) * (180.0 / PI);
// }