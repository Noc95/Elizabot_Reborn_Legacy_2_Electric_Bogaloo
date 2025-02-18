
#ifndef NocIMU_H
#define NocIMU_H

// #include <Adafruit_MPU6050.h>


class NocIMU
{
    
public:
    
    float gyroTrust = 0.97;
    // float accTrust = 0.05;

    float angle;
    float angle2;

    void initializeIMU();
    void calibrateIMU();

    void elizabotCalculateAngle();  // One in all function to rule them all

    void readIMUValues();
    // void kalmanUpdate();
    
    // void calculateAngle();

    NocIMU();

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
    // Vector3 rotateVector3(const NocIMU::Vector3& a, const NocIMU::Vector3& b);
    // Vector3 multiplyMatrixVector(float matrix[3][3], Vector3 vector);
    void kalmanUpdate();

private:

    int dummy;
    Vector3 gravity;
    // float gravityX = 0;
    // float gravityY = 0;
    // float gravityZ = 10.0;

    float rotatedGravityX = 0;
    float rotatedGravityY = 0;
    float rotatedGravityZ = 0;

    Vector3 acc;
    Vector3 gyro;

    Vector3 gyroRad;

    int rollingFilterCOunt = 5;
    float angleRollingFilterArr[5];    // Size needs to be same as rollingFilterCOunt

    // float gyroX = 0;
    // float gyroY = 0;
    // float gyroZ = 0;

    // float accX = 0;
    // float accY = 0;
    // float accZ = 0;

    float accXCalibrationValue = 0;
    float accYCalibrationValue = 0;
    float accZCalibrationValue = 0;

    float angleCorrection;

    float gyroXCalibrationValue = 0;
    float gyroYCalibrationValue = 0;
    float gyroZCalibrationValue = 0;

    float deltaTime;
    unsigned long lastTime;     // In Microseconds
    unsigned long currentTime;  // In Microseconds

    float deltaTime2;
    unsigned long lastTime2;     // In Microseconds
    unsigned long currentTime2;  // In Microseconds
    

    float angleRollingFilter(float newValue);
};




#endif