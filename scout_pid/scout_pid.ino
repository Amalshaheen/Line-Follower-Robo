// Enhanced PID Line Follower for ESP32 + L298N + 8 IR Sensors + Scout
// Optimized for maximum speed and minimal oscillations

#include "BluetoothSerial.h"
BluetoothSerial SerialBT;

// === EXPANDED SENSOR CONFIGURATION ===
const int mainSensorPins[] = { 34, 35, 32, 33, 25, 26, 27, 14 }; // 8 sensors
const int scoutSensorPin = 13; // Forward-looking scout sensor
const int sensorCount = 8;

const int enA = 14;
const int in1 = 27;
const int in2 = 26;
const int enB = 12;
const int in3 = 18;
const int in4 = 15;

const int pushButton = 4;
const int statusLED = 2;

// === OPTIMIZED PID CONFIGURATION ===
struct PIDConfig {
  float Kp = 8.0;    // Reduced for smoother response
  float Ki = 0.002;  // Very small integral for drift correction
  float Kd = 25.0;   // Increased derivative to damp oscillations
  int baseSpeed = 120;
  int maxSpeed = 255;
  float speedFactor = 1.0; // Dynamic speed control
};

PIDConfig config;

// === ENHANCED SENSOR WEIGHTS ===
const float sensorWeights[8] = { -4.0, -3.0, -2.0, -1.0, 1.0, 2.0, 3.0, 4.0 };
// Alternative: { -3.5, -2.5, -1.5, -0.5, 0.5, 1.5, 2.5, 3.5 } for even smoother response

// === STATE MANAGEMENT ===
enum RobotState { 
  STATE_LINE_FOLLOWING, 
  STATE_JUNCTION_DETECTED, 
  STATE_TURNING, 
  STATE_RECOVERY 
};

RobotState currentState = STATE_LINE_FOLLOWING;

// Recovery & junction parameters
const int recoverSpeed = 80;
const unsigned long recoverTimeout = 4000;
const unsigned long junctionTimeout = 1000;

// Global variables
float error = 0, lastError = 0, integral = 0;
bool robotActive = false;
unsigned long lineLostTime = 0;
unsigned long junctionDetectedTime = 0;
unsigned long lastPIDTime = 0;
unsigned long lastDebugTime = 0;

// Enhanced limits
const float INTEGRAL_LIMIT = 50.0;
const float DEADBAND_THRESHOLD = 0.15;
const float JUNCTION_THRESHOLD = 6.0; // High error indicates junction

// Path decision storage
int lastJunctionDecision = 0; // -1: left, 0: straight, 1: right

void setup() {
  SerialBT.begin("ESP32_LineFollower");
  SerialBT.println("=== Enhanced 8-Sensor + Scout Line Follower ===");
  SerialBT.println("Commands: KP, KI, KD, BASE, MAX, START, STOP, STATUS");

  // Initialize pins
  for (int i = 0; i < sensorCount; i++) pinMode(mainSensorPins[i], INPUT);
  pinMode(scoutSensorPin, INPUT);
  
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  pinMode(pushButton, INPUT_PULLUP);
  pinMode(statusLED, OUTPUT);

  stopMotors();
  lastPIDTime = micros();
}

void loop() {
  checkButton();
  checkSerialCommands();

  if (robotActive) {
    switch(currentState) {
      case STATE_LINE_FOLLOWING:
        advancedLineFollow();
        checkJunctions();
        break;
      case STATE_JUNCTION_DETECTED:
        handleJunction();
        break;
      case STATE_RECOVERY:
        handleRecovery();
        break;
    }
  } else {
    // Display sensor status when inactive
    if (millis() - lastDebugTime > 500) {
      displaySensorStatus();
      lastDebugTime = millis();
    }
    delay(100);
  }
}

void advancedLineFollow() {
  unsigned long currentTime = micros();
  float elapsedTime = (currentTime - lastPIDTime) / 1000000.0;
  lastPIDTime = currentTime;
  if (elapsedTime > 0.1) elapsedTime = 0.01;

  // Read all sensors
  int sensorValues[8];
  int activeSensors = 0;
  float weightedSum = 0;

  for (int i = 0; i < sensorCount; i++) {
    sensorValues[i] = !digitalRead(mainSensorPins[i]);
    weightedSum += sensorValues[i] * sensorWeights[i];
    activeSensors += sensorValues[i];
  }

  bool scoutOnLine = !digitalRead(scoutSensorPin);

  // === STATE TRANSITION LOGIC ===
  if (activeSensors == 0) {
    currentState = STATE_RECOVERY;
    lineLostTime = millis();
    SerialBT.println("Entering RECOVERY state - line lost");
    return;
  }

  // Calculate normalized error (-1.0 to +1.0)
  error = weightedSum / (activeSensors * 4.0); // Normalized by max weight

  // === DEADBAND & SMOOTHING ===
  if (abs(error) < DEADBAND_THRESHOLD) {
    error = 0;
    integral *= 0.9; // Decay integral in deadband
  }

  // === DYNAMIC SPEED CONTROL ===
  float turnSharpness = abs(error) + abs(error - lastError) / elapsedTime * 0.001;
  config.speedFactor = 1.0 - constrain(turnSharpness * 0.6, 0, 0.5); // Reduce speed up to 50% in turns

  // === ENHANCED TIME-BASED PID ===
  float P = error;
  
  // Conditional integral - only accumulate when error is small-to-medium
  if (abs(error) < 0.7) {
    integral += error * elapsedTime;
  } else {
    integral *= 0.95; // Decay integral during sharp turns
  }
  
  integral = constrain(integral, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);

  float derivative = (error - lastError) / elapsedTime;
  lastError = error;

  // Dynamic D-gain based on speed (more damping at high speed)
  float dynamicKd = config.Kd * (1.0 + config.speedFactor * 0.5);

  float output = (config.Kp * P) + (config.Ki * integral) + (dynamicKd * derivative);

  // === OPTIMIZED MOTOR CONTROL ===
  int adjustedBaseSpeed = config.baseSpeed * config.speedFactor;
  int leftSpeed = constrain(adjustedBaseSpeed - output, -config.maxSpeed, config.maxSpeed);
  int rightSpeed = constrain(adjustedBaseSpeed + output, -config.maxSpeed, config.maxSpeed);

  setMotorSpeed(leftSpeed, rightSpeed);

  // Debug output
  if (millis() - lastDebugTime > 100) {
    debugEnhancedOutput(sensorValues, scoutOnLine, error, output, leftSpeed, rightSpeed, turnSharpness);
    lastDebugTime = millis();
  }
}

void checkJunctions() {
  bool scoutOnLine = !digitalRead(scoutSensorPin);
  int sensorValues[8];
  int activeSensors = 0;
  
  for (int i = 0; i < sensorCount; i++) {
    sensorValues[i] = !digitalRead(mainSensorPins[i]);
    activeSensors += sensorValues[i];
  }

  // Junction detection: Scout sees no line but main array sees strong line
  if (!scoutOnLine && activeSensors >= 4) {
    // Check if it's a true junction (not just a curve)
    float junctionConfidence = 0;
    for (int i = 0; i < sensorCount; i++) {
      junctionConfidence += sensorValues[i] * abs(sensorWeights[i]);
    }
    
    if (junctionConfidence > JUNCTION_THRESHOLD) {
      currentState = STATE_JUNCTION_DETECTED;
      junctionDetectedTime = millis();
      SerialBT.println("JUNCTION DETECTED! Confidence: " + String(junctionConfidence));
    }
  }
}

void handleJunction() {
  SerialBT.println("Handling junction...");
  
  // Read current sensor pattern to decide turn direction
  int leftCount = 0, rightCount = 0;
  for (int i = 0; i < 4; i++) leftCount += !digitalRead(mainSensorPins[i]);
  for (int i = 4; i < 8; i++) rightCount += !digitalRead(mainSensorPins[i]);
  
  // Simple left-priority algorithm - modify as needed
  if (leftCount > 0) {
    SerialBT.println("Turning LEFT at junction");
    // Implement left turn maneuver
    setMotorSpeed(-80, 80);
    delay(300);
  } else if (rightCount > 0) {
    SerialBT.println("Turning RIGHT at junction");
    // Implement right turn maneuver
    setMotorSpeed(80, -80);
    delay(300);
  } else {
    SerialBT.println("Going STRAIGHT at junction");
    // Continue straight briefly
    setMotorSpeed(100, 100);
    delay(200);
  }
  
  // Return to line following
  currentState = STATE_LINE_FOLLOWING;
  integral = 0; // Reset integral after turn
}

void handleRecovery() {
  if (millis() - lineLostTime > recoverTimeout) {
    SerialBT.println("Recovery timeout - STOPPING");
    stopMotors();
    robotActive = false;
    digitalWrite(statusLED, LOW);
    return;
  }

  // Enhanced recovery with scout assistance
  bool scoutOnLine = !digitalRead(scoutSensorPin);
  
  if (scoutOnLine) {
    // Scout found line - move toward it
    setMotorSpeed(60, 60); // Move forward slowly
  } else {
    // Standard recovery based on last error
    if (lastError < 0) {
      setMotorSpeed(-recoverSpeed, recoverSpeed); // Turn left
    } else {
      setMotorSpeed(recoverSpeed, -recoverSpeed); // Turn right
    }
  }

  // Check if we've recovered
  int activeSensors = 0;
  for (int i = 0; i < sensorCount; i++) {
    activeSensors += !digitalRead(mainSensorPins[i]);
  }
  
  if (activeSensors >= 2) {
    currentState = STATE_LINE_FOLLOWING;
    integral = 0; // Reset integral
    SerialBT.println("Recovery successful - returning to line following");
  }
}

void debugEnhancedOutput(int sensorValues[], bool scout, float error, float output, 
                        int leftSpeed, int rightSpeed, float turnSharpness) {
  SerialBT.print("State: ");
  switch(currentState) {
    case STATE_LINE_FOLLOWING: SerialBT.print("FOLLOW"); break;
    case STATE_JUNCTION_DETECTED: SerialBT.print("JUNCTION"); break;
    case STATE_RECOVERY: SerialBT.print("RECOVERY"); break;
  }
  
  SerialBT.print(" | Sensors: ");
  for (int i = 0; i < sensorCount; i++) {
    SerialBT.print(sensorValues[i]);
  }
  SerialBT.print(" S:"); SerialBT.print(scout);
  
  SerialBT.print(" | Error: "); SerialBT.print(error, 3);
  SerialBT.print(" | Output: "); SerialBT.print(output, 1);
  SerialBT.print(" | Sharp: "); SerialBT.print(turnSharpness, 2);
  SerialBT.print(" | SpeedF: "); SerialBT.print(config.speedFactor, 2);
  SerialBT.print(" | Motors: "); SerialBT.print(leftSpeed); 
  SerialBT.print(","); SerialBT.println(rightSpeed);
}

void displaySensorStatus() {
  SerialBT.print("Sensor Status: Main[");
  for (int i = 0; i < sensorCount; i++) {
    SerialBT.print(!digitalRead(mainSensorPins[i]));
  }
  SerialBT.print("] Scout:");
  SerialBT.println(!digitalRead(scoutSensorPin));
}

// [Keep your existing motor control, button checking, and Bluetooth functions the same]
// setMotorSpeed(), stopMotors(), checkButton(), checkSerialCommands(), etc.

// Add this function to your existing Bluetooth command processing:
void processCommand(String command) {
  // [Your existing command processing code]
  // Add these commands if needed:
  if (command == "state") {
    SerialBT.print("Current State: ");
    switch(currentState) {
      case STATE_LINE_FOLLOWING: SerialBT.println("LINE_FOLLOWING"); break;
      case STATE_JUNCTION_DETECTED: SerialBT.println("JUNCTION_DETECTED"); break;
      case STATE_RECOVERY: SerialBT.println("RECOVERY"); break;
    }
  }
}