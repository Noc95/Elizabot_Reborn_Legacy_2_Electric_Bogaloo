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

NocMotor leftMotor(900, 4, 5, 9, 10, -1);

// Angle PID. Input = angle, output = motor signal
// P~40, I~230, D~3-5
float angle_kp = 40;    
float angle_ki = 235;
float angle_kd = 10;
float angleMaxOutputAbs = 1000;
NocPID anglePID(angle_kp, angle_ki, angle_kd, angleMaxOutputAbs);

// Correction PID. Input = motor signal, output = angle
float correction_kp = 0.01;
float correction_ki = 2;
float correction_kd = 0;
float correctionMaxOutputAbs = 10;
NocPID correctionPID(correction_kp, correction_ki, correction_kd, correctionMaxOutputAbs);

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
  handleEncoderInterrupt();
}

void motor2Interrupt() {
  // motor2.handleEncoderInterrupt();
}



void setup(void) {

  Serial.begin(115200);
  while (!Serial)
    delay(10); 

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
  correctionPID.enabled = false;

  leftMotor.init();
  leftMotor.enabled = true;

  attachInterrupt(digitalPinToInterrupt(leftMotor.encoderPinA), motor1Interrupt, CHANGE);

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


  // Sample time calculations  
  currentSampleTime = micros();
  if ((currentSampleTime - lastSampleTime) > sampleTime) {
    lastSampleTime = micros();

    // leftMotor.PIDsetPoint = 5;
    leftMotor.RunMotorControl();
  
    // --- Get data from human input ---
    anglePID.setPoint = 0;
    
    // --- Activate PID when standing up ---
    if (!anglePID.enabled) {
      if (abs(imu.angle) < 5) {
        anglePID.enabled = true;
      }
    } //  Deactivate PID when falling down
    else {
      if (abs(imu.angle) > 50) {
        anglePID.enabled = false;
        correctionPID.enabled = false;
      }
    }

    // Activate correction PID if no user input
    // if (anglePID.setPoint == 0) {
    //   correctionPID.enabled = true;
    // }
    
    // --- Calculate control signal with PID ---
    if (correctionPID.enabled) {
      anglePID.input = correctionPID.output;
    }
    else {
      anglePID.input = imu.angle;
    }
    
    // Calculate motor signal
    anglePID.calculate();

    // Calculate angle correction
    // correctionPID.input = anglePID.output;
    // correctionPID.calculate();
    
    // DEBUG
    // Serial.print("  Angle: ");
    // Serial.print(imu.angle);
    // Serial.print("  angle PID output: ");
    // Serial.print(anglePID.output);
    // Serial.print("  corr PID output: ");
    // Serial.print(correctionPID.output);
    // Serial.println();
  
    // --- Send control signal and turn signal to the motors ---
    // if (anglePID.enabled) {
    //   motorSignalMixer(anglePID.output, 0);
    // }
    // else {
    //   sendMotorSignals(0, 0);
    // }

  }
}


// --- Benchmark function ---

void timeBenchmark() {

  int iterations = 300;
  int frequency = 0;
  unsigned int startTime = micros();

  for (int i = 0; i < iterations; i++) {

    // --- Code to benchmark here ---

    // elizabot();
    // leftMotor.calculateRotationSpeed();
    

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

float testCount = 0;

void loop() {

  elizabot();

  testCount++;

  
  // if (testCount >= 2000) {
  //   testCount = 0;
  //   if (leftMotor.PIDsetPoint == -10) {
  //     leftMotor.PIDsetPoint = 10;
  //   }
  //   else {
  //     leftMotor.PIDsetPoint = -10;
  //   }
  // }

  leftMotor.PIDsetPoint = 10 * sin(2*PI*(1.0/3.0)*millis()/1000) + 5 * sin(PI*(1.0/3.0)*millis()/1000);

  // delay(2);

  // leftMotor.RunMotorControl();

  // leftMotor.calculateRotationSpeed();
  Serial.println(leftMotor.rotationSpeed);
  // Serial.println(leftMotor.PIDsetPoint);

  // counter++;

  // if (counter >= 500) {
  //   Serial.println(motor1.rotationSpeed);
  //   counter = 0;
  // }
  // if (motor1.encoderCount == 5) {
  //   digitalRead(motor1.encoderPinA);
  // }

  // elizabot();
  // timeBenchmark();

  // imu.kalmanUpdate();
  // imu.elizabotCalculateAngle();

  // Serial.print(imu.angle);
  // Serial.print("  ");
  // Serial.print(imu.angle2);
  // Serial.println();
} 

/*

int encoderA = 9;
int encoderB = 10;
int encoderCount = 0;
  pinMode(9, INPUT);
  pinMode(10, INPUT);
  attachInterrupt(digitalPinToInterrupt(9), handleEncoderInterrupt, CHANGE);
  
  rps = ((encoderCount-lastValue)/180.0)*500;
  lastValue = encoderCount;

  
  */