#include <Arduino.h>
#include <NocPID.h>
#include <NocIMU.h>
#include <NocMotor.h>

// #include "hardware/pwm.h"


// --------------------------------------------------------

#define MOTOR_1_PIN_A 2
#define MOTOR_1_PIN_B 3
#define MOTOR_2_PIN_A 5
#define MOTOR_2_PIN_B 4

#define MIN_MOTOR_SIGNAL 0
#define MAX_MOTOR_SIGNAL 255
#define MIN_CONTROL_SIGNAL 0
#define MAX_CONTROL_SIGNAL 1000

// TODO: calibrate angle at start?

NocMotor leftMotor(13, 4, 5, 10, 9, 1);
NocMotor rightMotor(13, 2, 3, 8, 7, -1);

// Angle PID. Input = angle, output = motor signal
// P~40, I~230, D~3-5
float angle_kp = 0.3;
float angle_ki = 10;
float angle_kd = 0.0001;
float angleMaxOutputAbs = 13; // Max RPS of motors
NocPID anglePID(angle_kp, angle_ki, angle_kd, angleMaxOutputAbs);

// position PID. Input = motor signal, output = angle
float position_kp = 1;
float position_ki = 0;
float position_kd = 0;
float positionMaxOutputAbs = 10;  // Max angle input to the anglePID
NocPID positionPID(position_kp, position_ki, position_kd, positionMaxOutputAbs);

NocIMU imu;

unsigned long sampleTime = 2000; // Microseconds. 2000us = 500Hz
unsigned long lastSampleTime = 0;
unsigned long currentSampleTime = 0;

// ---------------------------------------------------------

void handleEncoderInterrupt() {

  // bool pinAState = gpio_get(motor1.encoderPinA);
  // bool pinBState = gpio_get(motor1.encoderPinB);
  
  if (digitalRead(leftMotor.encoderPinA) == digitalRead(leftMotor.encoderPinB)) {
    leftMotor.encoderCount++;
  } else {
    leftMotor.encoderCount--;
  }
  // delay(1);
  // testcount++;
}

void motor1Interrupt() {
  // handleEncoderInterrupt();
  leftMotor.handleEncoderInterrupt();
}

void motor2Interrupt() {
  rightMotor.handleEncoderInterrupt();
}



void setup(void) {

  // Serial.begin(115200);
  // while (!Serial)
  //   delay(10); 

  pinMode(MOTOR_1_PIN_A, OUTPUT);
  pinMode(MOTOR_1_PIN_B, OUTPUT);
  pinMode(MOTOR_2_PIN_A, OUTPUT);
  pinMode(MOTOR_2_PIN_B, OUTPUT);

  digitalWrite(MOTOR_1_PIN_A, LOW);
  digitalWrite(MOTOR_1_PIN_B, LOW);
  digitalWrite(MOTOR_2_PIN_A, LOW);
  digitalWrite(MOTOR_2_PIN_B, LOW);

  imu.initializeIMU();
  imu.calibrateIMU();

  anglePID.enabled = false;
  positionPID.enabled = false;

  leftMotor.init();
  leftMotor.enabled = true;

  rightMotor.init();
  rightMotor.enabled = true;

  attachInterrupt(digitalPinToInterrupt(leftMotor.encoderPinA), motor1Interrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(rightMotor.encoderPinA), motor2Interrupt, CHANGE);

  delay(1000);
}

// --- Project specific functions -----------------------------------------------------

void sendMotorSignals(int motor1Signal, int motor2Signal) {

  // if (motor1Signal == 0 and motor2Signal == 0) {
  //   digitalWrite(MOTOR_1_PIN_A, LOW);
  //   digitalWrite(MOTOR_1_PIN_B, LOW);
  //   digitalWrite(MOTOR_2_PIN_A, LOW);
  //   digitalWrite(MOTOR_2_PIN_B, LOW);
  // }
  // else if (controlSignal >= 0) {
  //   analogWrite(MOTOR_1_PIN_A, abs(motor1Signal));
  //   digitalWrite(MOTOR_1_PIN_B, LOW);
  //   digitalWrite(MOTOR_2_PIN_A, LOW);
  //   analogWrite(MOTOR_2_PIN_B, abs(motor2Signal));
  // }
  // else {
  //   digitalWrite(MOTOR_1_PIN_A, LOW);
  //   analogWrite(MOTOR_1_PIN_B, abs(motor1Signal));
  //   analogWrite(MOTOR_2_PIN_A, abs(motor2Signal));
  //   digitalWrite(MOTOR_2_PIN_B, LOW);
  // }

  if (motor1Signal > 0) {
    analogWrite(MOTOR_1_PIN_A, abs(motor1Signal));
    analogWrite(MOTOR_1_PIN_B, 0);
  }
  else if (motor1Signal < 0) {
    analogWrite(MOTOR_1_PIN_A, 0);
    analogWrite(MOTOR_1_PIN_B, abs(motor1Signal));
  }
  else {
    analogWrite(MOTOR_1_PIN_A, 0);
    analogWrite(MOTOR_1_PIN_B, 0);
  }

  if (motor2Signal > 0) {
    analogWrite(MOTOR_2_PIN_A, abs(motor2Signal));
    analogWrite(MOTOR_2_PIN_B, 0);
  }
  else if (motor2Signal < 0) {
    analogWrite(MOTOR_2_PIN_A, 0);
    analogWrite(MOTOR_2_PIN_B, abs(motor2Signal));
  }
  else {
    analogWrite(MOTOR_2_PIN_A, 0);
    analogWrite(MOTOR_2_PIN_B, 0); 
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
    motor1Signal = min(int(map(controlSignal, MIN_CONTROL_SIGNAL, MAX_CONTROL_SIGNAL, MIN_MOTOR_SIGNAL, MAX_MOTOR_SIGNAL)) + motor1TurnAddition, MAX_MOTOR_SIGNAL);
    motor2Signal = min(int(map(controlSignal, MIN_CONTROL_SIGNAL, MAX_CONTROL_SIGNAL, MIN_MOTOR_SIGNAL, MAX_MOTOR_SIGNAL)) + motor2TurnAddition, MAX_MOTOR_SIGNAL);
  }
  else {
    motor1TurnAddition = -map(turnSignal, -maxTurnSignalAddition, maxTurnSignalAddition, -MAX_CONTROL_SIGNAL, MAX_CONTROL_SIGNAL);
    motor2TurnAddition = map(turnSignal, -maxTurnSignalAddition, maxTurnSignalAddition, -MAX_CONTROL_SIGNAL, MAX_CONTROL_SIGNAL);
    motor1Signal = max(int(map(controlSignal, MIN_CONTROL_SIGNAL, -MAX_CONTROL_SIGNAL, -MIN_MOTOR_SIGNAL, -MAX_MOTOR_SIGNAL)) + motor1TurnAddition, -MAX_MOTOR_SIGNAL);
    motor2Signal = max(int(map(controlSignal, MIN_CONTROL_SIGNAL, -MAX_CONTROL_SIGNAL, -MIN_MOTOR_SIGNAL, -MAX_MOTOR_SIGNAL)) + motor2TurnAddition, -MAX_MOTOR_SIGNAL);
  }

  // Serial.print("  Turn 1: ");
  // Serial.print(motor1TurnAddition);
  // Serial.print("  Motor 1: ");
  // Serial.print(motor1Signal);
  // Serial.print("  Motor 2: ");
  // Serial.print(motor2Signal);

  sendMotorSignals(motor1Signal, motor2Signal);
  
}



// --- Main function ------------------------------------------

// This is where the main robot code goes
void elizabot() {

  imu.elizabotCalculateAngle();
  leftMotor.calculateRotationSpeed();
  rightMotor.calculateRotationSpeed();


  // Sample time calculations  
  currentSampleTime = micros();
  if ((currentSampleTime - lastSampleTime) > sampleTime) {
    lastSampleTime = micros();


    // --- Get data from human input ---
    anglePID.setPoint = 0;
    

    // --- Activate PIDs when standing up ---
    if (!anglePID.enabled) {
      if (abs(imu.angle) < 5) {
        anglePID.enabled = true;
        leftMotor.enabled = true;
        rightMotor.enabled = true;
      }
    } //  Deactivate PIDs when falling down
    else {
      if (abs(imu.angle) > 50) {
        anglePID.enabled = false;
        positionPID.enabled = false;
        // leftMotor.enabled = false;
        // rightMotor.enabled = false;
      }
    }


    // --- Activate position PID if no user input ---
    // if (anglePID.setPoint == 0) {
    //   positionPID.enabled = true;
    // }
    

    // --- If position PID active -> Hold position ---
    if (positionPID.enabled) {
      anglePID.setPoint = positionPID.output;
    }


    // --- Angle PID input and calculations ---
    anglePID.input = imu.angle;
    anglePID.calculate();


    // --- Motor input and calculations ---
    leftMotor.PIDsetPoint = anglePID.output;
    rightMotor.PIDsetPoint = anglePID.output;
    leftMotor.RunMotorControl();
    rightMotor.RunMotorControl();

    // Serial.println(leftMotor.enabled);


    // --- DEBUG ---
    // Serial.print("  Angle: ");
    // Serial.println(imu.angle);
    // Serial.print("  angle PID output: ");
    // Serial.println(anglePID.output);
    // Serial.print("  corr PID output: ");
    // Serial.print(positionPID.output);
    // Serial.println();
  
  }
}


// --- Benchmark function ---

void timeBenchmark() {

  int iterations = 300;
  int frequency = 0;
  unsigned int startTime = micros();

  for (int i = 0; i < iterations; i++) {

    // --- Code to benchmark here ---

    elizabot();

    // ------------------------------
  }
  unsigned int endTime = micros();

  // Calculate frequency:
  if (endTime - startTime == 0) {
    Serial.print("Can not divide by zero!");
  }
  else {
    frequency = (float(iterations) / (float((endTime - startTime)))) * 1000000.0;
  }

  // Serial.print("  Start time: ");
  // Serial.print(startTime);
  // Serial.print("  End time: ");
  // Serial.print(endTime);

  Serial.print("  Frequency: ");
  Serial.println(frequency);

  // Serial.println(leftMotor.rotationSpeed);
}


// --------------------------------------------------------


void loop() {

  elizabot();
  // leftMotor.PIDsetPoint = 1;
  // timeBenchmark();
} 

// I and D in motor PIDs not working when input is from anglePID?????
// WHAT GIVES?