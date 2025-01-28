// #include <Adafruit_MPU6050.h>
// #include <Adafruit_Sensor.h>
#include <Wire.h>
#include <NocPID.h>
// #include <NocMPU.h>
// #include <ESP8266WiFi.h>


// --------------------------------------------------------

#define MOTOR_1_PIN_A 14
#define MOTOR_1_PIN_B 12
#define MOTOR_2_PIN_A 13
#define MOTOR_2_PIN_B 15

#define MIN_MOTOR_SIGNAL 20
#define MAX_MOTOR_SIGNAL 255
#define MIN_CONTROL_SIGNAL 0
#define MAX_CONTROL_SIGNAL 1000

// TODO: calibrate angle at start?

// Angle PID. Input = angle, output = motor signal
float angle_kp = 50;
float angle_ki = 10;
float angle_kd = 0;
float angleMaxOutputAbs = 1000;
NocPID anglePID(angle_kp, angle_ki, angle_kd, angleMaxOutputAbs);

// Correction PID. Input = motor signal, output = angle
float correction_kp = 0.01;
float correction_ki = 2;
float correction_kd = 0;
float correctionMaxOutputAbs = 10;
NocPID correctionPID(correction_kp, correction_ki, correction_kd, correctionMaxOutputAbs);

NocMPU mpu;

// sample time in amount of cycles. Calculated with: period time * cpu frequency (80 MHz)
unsigned long sampleTime = 2000; // Microseconds
// 500 Hz (0.002 seconds * 80000000 Hz)
unsigned long lastSampleTime = 0;
unsigned long currentSampleTime = 0;

const char* ssid = "elizabot";
const char* password = "12345678"; // Password must be at least 8 characters

// WiFiServer server(80); // Port 80, commonly used for HTTP/TCP communication

// ---------------------------------------------------------

void setup(void) {

  Serial.begin(9600);
  while (!Serial)
    delay(10); 

  Serial.print("CPU Frequency: "); 
  Serial.print(ESP.getCpuFreqMHz()); 
  Serial.println(" MHz");

  pinMode(MOTOR_1_PIN_A, OUTPUT);
  pinMode(MOTOR_1_PIN_B, OUTPUT);
  pinMode(MOTOR_2_PIN_A, OUTPUT);
  pinMode(MOTOR_2_PIN_B, OUTPUT);

  digitalWrite(MOTOR_1_PIN_A, LOW);
  digitalWrite(MOTOR_1_PIN_B, LOW);
  digitalWrite(MOTOR_2_PIN_A, LOW);
  digitalWrite(MOTOR_2_PIN_B, LOW);

  mpu.initializeMPU();
  mpu.calibrateMPU();

  WiFi.softAP(ssid, password);
  Serial.println("Hotspot created.");
  Serial.print("SSID: ");
  Serial.println(ssid);
  Serial.print("IP Address: ");
  Serial.println(WiFi.softAPIP());

  // Start the server
  server.begin();
  Serial.println("TCP server started.");
  
  anglePID.enabled = false;
  correctionPID.enabled = false;

  delay(1000);
}

// --- Project specific functions -----------------------------------------------------

void sendMotorSignals(int motor1Signal, int motor2Signal, int controlSignal) {
  // TODO: Test motor signals

  if (motor1Signal == 0 and motor2Signal == 0) {
    digitalWrite(MOTOR_1_PIN_A, LOW);
    digitalWrite(MOTOR_1_PIN_B, LOW);
    digitalWrite(MOTOR_2_PIN_A, LOW);
    digitalWrite(MOTOR_2_PIN_B, LOW);
  }
  else if (controlSignal >= 0) {
    analogWrite(MOTOR_1_PIN_A, abs(motor1Signal));
    digitalWrite(MOTOR_1_PIN_B, LOW);

    digitalWrite(MOTOR_2_PIN_A, LOW);
    analogWrite(MOTOR_2_PIN_B, abs(motor2Signal));
  }
  else {
    digitalWrite(MOTOR_1_PIN_A, LOW);
    analogWrite(MOTOR_1_PIN_B, abs(motor1Signal));

    analogWrite(MOTOR_2_PIN_A, abs(motor2Signal));
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

  sendMotorSignals(motor1Signal, motor2Signal, controlSignal);
  
}

static inline int asm_ccount(void) {

    int r;

    asm volatile ("rsr %0, ccount" : "=r"(r));
    return r;
}

String readClientMessage() {
  
  WiFiClient client = server.available();
  // Serial.print(client);
  if (client) {
    Serial.println("Client connected.");
    if (client.connected()) {
      if (client.available()) {
        String message = client.readStringUntil('\n'); // Read until newline
        Serial.print(message);
        Serial.println();
        return message;
        
      }
    }
    // else {
    //   client.stop(); // Disconnect the client
    //   Serial.println("Client disconnected.");
    //   return "";
    // }
  }
  return "";
}

// --- Main function ------------------------------------------

// This is where the main robot code goes
void elizabot() {

  mpu.kalmanUpdate();
  mpu.calculateAngle();

  // Serial.print("  currentSampleTime - lastSampleTime: ");
  // Serial.println(currentSampleTime - lastSampleTime);

  // Sample time calculations  
  currentSampleTime = micros();
  if ((currentSampleTime - lastSampleTime) > sampleTime) {
    lastSampleTime = micros();
  
    
    // --- Get data from human input ---
    anglePID.setPoint = 0;
    
    // --- Activate PID when standing up ---
    if (!anglePID.enabled) {
      if (abs(mpu.angle) < 5) {
        anglePID.enabled = true;
      }
    } //  Deactivate PID when falling down
    else {
      if (abs(mpu.angle) > 50) {
        anglePID.enabled = false;
        correctionPID.enabled;
      }
    }

    // Activate correction PID if no user input
    if (anglePID.setPoint == 0) {
      correctionPID.enabled = true;
    }
    
    // --- Calculate control signal with PID ---
    if (correctionPID.enabled) {
      anglePID.input = correctionPID.output;
    }
    else {
      anglePID.input = -mpu.angle;
    }
    
    // Calculate motor signal
    anglePID.calculate();

    // Calculate angle correction
    correctionPID.input = anglePID.output;
    correctionPID.calculate();
    
    Serial.print("  Angle: ");
    Serial.print(mpu.angle);
    Serial.print("  angle PID output: ");
    Serial.print(anglePID.output);
    Serial.print("  corr PID output: ");
    Serial.print(correctionPID.output);
    Serial.println();
  
    // --- Send control signal and turn signal to the motors ---
    if (anglePID.enabled) {
      motorSignalMixer(anglePID.output, 0);
    }
    else {
      sendMotorSignals(0, 0, 0);
    }
      

    // anglePID.enabled = true;
  }

  return;
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

}


// --------------------------------------------------------

void loop() {

  elizabot();
  // timeBenchmark();
  // Serial.print(mpu.angle);
  // Serial.println();

  // sendMotorSignals(-500, 500, 1);
  // Serial.println("going!");
  // analogWrite(MOTOR_1_PIN_A, 50);
  // digitalWrite(MOTOR_1_PIN_B, LOW);

  // delay(1000);
} 