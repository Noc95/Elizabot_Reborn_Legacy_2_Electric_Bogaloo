#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <NocPID.h>
#include <NocMPU.h>

// --------------------------------------------------------

const int motor1PinA = 0;
const int motor1PinB = 0;
const int motor2PinA = 0;
const int motor2PinB = 0;

int minMotorSignal = 20;
int maxMotorSignal = 255;
int minControlSignal = 0;
int maxControlSignal = 1000;

float angle_kp = 1;
float angle_ki = 0;
float angle_kd = 0;
float angle_maxOutputAbs = 1000;
NocPID anglePID(angle_kp, angle_ki, angle_kd, angle_maxOutputAbs);


// ---------------------------------------------------------

void setup(void) {
  Serial.begin(115200);
  while (!Serial)
    delay(10); 

  pinMode(motor1PinA, OUTPUT);
  pinMode(motor1PinB, OUTPUT);
  pinMode(motor2PinA, OUTPUT);
  pinMode(motor2PinB, OUTPUT);
  
  delay(100);
}

// --------------------------------------------------------

void sendMotorSignal(int motorPinA, int motorPinB, int controlSignal) {

  if (controlSignal < 0) {
    int motorSignal = map(controlSignal, minMotorSignal, maxMotorSignal, -minControlSignal, -maxControlSignal);
    analogWrite(motorPinA, motorSignal);
    digitalWrite(motorPinB, LOW);
  }
  else {
    int motorSignal = map(controlSignal, minMotorSignal, maxMotorSignal, minControlSignal, maxControlSignal);
    digitalWrite(motorPinA, LOW);
    analogWrite(motorPinB, motorSignal);  
  }
}

// --------------------------------------------------------

void loop() {

  sendMotorSignal(motor1PinA, motor1PinB, 0);
  
}