
#ifndef NOCMPU_H
#define NOCMPU_H

#include <Adafruit_MPU6050.h>


class NocMPU
{
    
public:
    
    float gyroTrust = 0.95;
    float accTrust = 0.05;

    float angle;

    float yaw;
    float pitch;
    float roll;

    void initializeMPU();
    void calibrateMPU();

    void readMpuValues();
    void kalmanUpdate();
    
    void calculateAngle();

    NocMPU();

    struct Vector3 {
        float x, y, z;

        // Overload addition
        Vector3 operator+(const Vector3& other) const {
            return {x + other.x, y + other.y, z + other.z};
        }

        // Overload subtraction
        Vector3 operator-(const Vector3& other) const {
            return {x - other.x, y - other.y, z - other.z};
        }

        // Overload scalar multiplication
        Vector3 operator*(float scalar) const {
            return {x * scalar, y * scalar, z * scalar};
        }

        // Normalize the vector to unit length
        void normalize() {
            float mag = sqrt(x * x + y * y + z * z);
            if (mag > 0) {
                x /= mag;
                y /= mag;
                z /= mag;
            }
        }
    };

    Vector3 cross(const Vector3& a, const Vector3& b);
    Vector3 rotateVector3(const NocMPU::Vector3& a, const NocMPU::Vector3& b);
    Vector3 multiplyMatrixVector(float matrix[3][3], Vector3 vector);

private:

    Adafruit_MPU6050 mpu;

    Vector3 gravity;
    // float gravityX = 0;
    // float gravityY = 0;
    // float gravityZ = 10.0;

    float rotatedGravityX = 0;
    float rotatedGravityY = 0;
    float rotatedGravityZ = 0;

    Vector3 acc;
    Vector3 gyro;

    float accXCalibrationValue = 0;
    float accYCalibrationValue = 0;
    float accZCalibrationValue = 0;

    float gyroXCalibrationValue = 0;
    float gyroYCalibrationValue = 0;
    float gyroZCalibrationValue = 0;

    float deltaTime;
    unsigned long lastTime;     // In Microseconds
    unsigned long currentTime;  // In Microseconds
    
};




#endif