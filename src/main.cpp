#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <NocPID.h>

Adafruit_MPU6050 mpu;

void setup(void) {
  Serial.begin(115200);
  while (!Serial)
    delay(10); 

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
  
  delay(100);
}

void loop() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  float gyroCalibrationValue = g.gyro.x;

  while(true)
  {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    float Gx = a.acceleration.x;
    float Gy = a.acceleration.y;
    float Gz = a.acceleration.z;

    float Glen = sqrt(pow(Gx, 2) + pow(Gy, 2) + pow(Gz, 2));

    float angle = asin(Gy / Glen);

    // Conversion from rad to angle
    angle *= 180;
    angle /= PI;



    Serial.println(angle);

    delay(500);
  }

  
}