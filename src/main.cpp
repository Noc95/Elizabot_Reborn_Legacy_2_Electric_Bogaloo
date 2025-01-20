#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <NocPID.h>
#include <NocMPU.h>
#include <ESP8266WiFi.h>


// --------------------------------------------------------

#define MOTOR_1_PIN_A 14
#define MOTOR_1_PIN_B 12
#define MOTOR_2_PIN_A 13
#define MOTOR_2_PIN_B 15

#define MIN_MOTOR_SIGNAL 30
#define MAX_MOTOR_SIGNAL 255
#define MIN_CONTROL_SIGNAL 0
#define MAX_CONTROL_SIGNAL 1000

float angle_kp = 0.5;
float angle_ki = 5;
float angle_kd = 0;
float angle_maxOutputAbs = 1000;
NocPID anglePID(angle_kp, angle_ki, angle_kd, angle_maxOutputAbs);

NocMPU mpu;

// sample time in amount of cycles. Calculated with: period time * cpu frequency (80 MHz)
unsigned long sampleTime = 2000; // Microseconds
// 500 Hz (0.002 seconds * 80000000 Hz)
unsigned long lastSampleTime = 0;
unsigned long currentSampleTime = 0;

const char* ssid = "elizabot";
const char* password = "12345678"; // Password must be at least 8 characters

WiFiServer server(80); // Port 80, commonly used for HTTP/TCP communication

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
  
  // anglePID.enabled = true;

  delay(100);
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
    analogWrite(MOTOR_1_PIN_A, motor1Signal);
    digitalWrite(MOTOR_1_PIN_B, LOW);

    digitalWrite(MOTOR_2_PIN_A, LOW);
    analogWrite(MOTOR_2_PIN_B, motor2Signal);
  }
  else {
    digitalWrite(MOTOR_1_PIN_A, LOW);
    analogWrite(MOTOR_1_PIN_B, motor1Signal);

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

  // Sample time calculations  
  currentSampleTime = micros();
  if ((currentSampleTime - lastSampleTime) < sampleTime) {
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
      if (abs(mpu.angle) > 80) {
        anglePID.enabled = false;
      }
    }
    
    // --- Calculate control signal with PID ---
    anglePID.input = mpu.angle;
    anglePID.calculate();
    // Serial.print("PID setpoint: ");
    // Serial.print(anglePID.setPoint);
    // Serial.print("     PID output: ");
    // Serial.print(anglePID.output);
  
    // --- Send control signal and turn signal to the motors ---
    motorSignalMixer(anglePID.output, 0);

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

  // elizabot();
  timeBenchmark();

  // Serial.println();
} 