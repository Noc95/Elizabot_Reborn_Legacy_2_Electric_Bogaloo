#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <NocPID.h>
#include <NocMPU.h>

// --------------------------------------------------------

#define MOTOR_1_PIN_A 0
#define MOTOR_1_PIN_B 0
#define MOTOR_2_PIN_A 0
#define MOTOR_2_PIN_B 0

#define MIN_MOTOR_SIGNAL 20
#define MAX_MOTOR_SIGNAL 255
#define MIN_CONTROL_SIGNAL 0
#define MAX_CONTROL_SIGNAL 1000

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

  pinMode(MOTOR_1_PIN_A, OUTPUT);
  pinMode(MOTOR_1_PIN_B, OUTPUT);
  pinMode(MOTOR_2_PIN_A, OUTPUT);
  pinMode(MOTOR_2_PIN_B, OUTPUT);
  
  delay(100);
}

// --------------------------------------------------------

void sendMotorSignals(int motor1Signal, int motor2Signal, int controlSignal) {
  // TODO: Test motor signals

  if (motor1Signal == 0 and motor2Signal == 0) {
    digitalWrite(MOTOR_1_PIN_A, LOW);
    digitalWrite(MOTOR_1_PIN_B, LOW);
    digitalWrite(MOTOR_2_PIN_A, LOW);
    digitalWrite(MOTOR_2_PIN_B, LOW);
  }
  else if (controlSignal > 0) {
    analogWrite(MOTOR_1_PIN_A, motor1Signal);
    digitalWrite(MOTOR_1_PIN_B, LOW);

    digitalWrite(MOTOR_2_PIN_A, LOW);
    analogWrite(MOTOR_2_PIN_B, motor2Signal);
  }
  else {
    digitalWrite(MOTOR_1_PIN_A, LOW);
    digitalWrite(MOTOR_1_PIN_B, motor1Signal);

    analogWrite(MOTOR_2_PIN_A, motor2Signal);
    digitalWrite(MOTOR_2_PIN_B, LOW);
  }
}

void motorSignalMixer(int controlSignal, int turnSignal) {        
  // TODO: Test motor mixer!
  // - Test with turnsignal
  // - Test with controlSignal

  int motor1Signal = 0;
  int motor2Signal = 0;

  int maxTurnSignalAddition = 100;
  // int controlSignalThreshold = 50;
  int motor1TurnAddition = 0;
  int motor2TurnAddition = 0;

  if (controlSignal >= 0) {
    motor1TurnAddition = map(turnSignal, -maxTurnSignalAddition, maxTurnSignalAddition, -MAX_CONTROL_SIGNAL, MAX_CONTROL_SIGNAL);
    motor2TurnAddition = -map(turnSignal, -maxTurnSignalAddition, maxTurnSignalAddition, -MAX_CONTROL_SIGNAL, MAX_CONTROL_SIGNAL);
    motor1Signal = min(int(map(controlSignal, MIN_MOTOR_SIGNAL, MAX_MOTOR_SIGNAL, MIN_CONTROL_SIGNAL, MAX_CONTROL_SIGNAL)) + motor1TurnAddition, MAX_MOTOR_SIGNAL);
    motor2Signal = min(int(map(controlSignal, MIN_MOTOR_SIGNAL, MAX_MOTOR_SIGNAL, MIN_CONTROL_SIGNAL, MAX_CONTROL_SIGNAL)) + motor2TurnAddition, MAX_MOTOR_SIGNAL);
  }
  else {
    motor1TurnAddition = -map(turnSignal, -maxTurnSignalAddition, maxTurnSignalAddition, -MAX_CONTROL_SIGNAL, MAX_CONTROL_SIGNAL);
    motor2TurnAddition = map(turnSignal, -maxTurnSignalAddition, maxTurnSignalAddition, -MAX_CONTROL_SIGNAL, MAX_CONTROL_SIGNAL);
    motor1Signal = max(int(map(controlSignal, -MIN_MOTOR_SIGNAL, -MAX_MOTOR_SIGNAL, MIN_CONTROL_SIGNAL, -MAX_CONTROL_SIGNAL)) + motor1TurnAddition, -MAX_MOTOR_SIGNAL);
    motor2Signal = max(int(map(controlSignal, -MIN_MOTOR_SIGNAL, -MAX_MOTOR_SIGNAL, MIN_CONTROL_SIGNAL, -MAX_CONTROL_SIGNAL)) + motor2TurnAddition, -MAX_MOTOR_SIGNAL);
  }

  sendMotorSignals(motor1Signal, motor2Signal, controlSignal);
  
}

// --------------------------------------------------------

void loop() {
  int gyroCalibrationValue = 0;
  if (gyroCalibrationValue == 0)

  sendMotorSignals(50, 20, 0);
} 
