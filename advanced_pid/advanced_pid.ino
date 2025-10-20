// Enhanced PID Line Follower for ESP32 + L298N + 5 IR Sensors
// With time-based PID, non-blocking recovery, and Bluetooth tuning

#include "BluetoothSerial.h"

BluetoothSerial SerialBT;

const int sensorPins[] = { 34, 35, 32, 33, 25 };
const int sensorCount = 5;

const int enA = 14;
const int in1 = 27;
const int in2 = 26;
const int enB = 12;
const int in3 = 18;
const int in4 = 15;

const int pushButton = 4;
const int statusLED = 2;

// === PID CONFIGURATION STRUCT ===
struct PIDConfig {
  float Kp = 10.0;  // Reduced from 34
  float Ki = 0.0;   // Enabled integral
  float Kd = 0.0;  // Reduced from 150
  int baseSpeed = 100;
  int maxSpeed = 255;
};

PIDConfig config;

// Recovery parameters
const int recoverSpeed = 65;
const unsigned long recoverTimeout = 6000;  // ms

// Global variables
float error = 0, lastError = 0, integral = 0;
bool robotActive = false;
bool recovering = false;
unsigned long lineLostTime = 0;
unsigned long lastPIDTime = 0;
unsigned long lastDebugTime = 0;

// Limits
const float INTEGRAL_LIMIT = 100.0;
const float DEADBAND_THRESHOLD = 0.1;

void setup() {
  SerialBT.begin("ESP32_LineFollower"); // Bluetooth device name
  SerialBT.println("=== Enhanced Line Follower ===");
  SerialBT.println("Commands: KP, KI, KD, BASE, MAX, CAL, START, STOP");
  SerialBT.println("Example: KP 20.5");
  SerialBT.println("Connect to ESP32_LineFollower via Bluetooth");

  // Initialize pins
  for (int i = 0; i < sensorCount; i++) pinMode(sensorPins[i], INPUT);

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
    pidLineFollow();
  } else {
    // Read and display sensors when inactive
    if (millis() - lastDebugTime > 200) {
      lastDebugTime = millis();
    }
    delay(100);
  }
}

void checkButton() {
  if (digitalRead(pushButton) == LOW) {
    delay(50);  // Debounce
    if (digitalRead(pushButton) == LOW) {
      robotActive = !robotActive;
      digitalWrite(statusLED, robotActive ? HIGH : LOW);
      stopMotors();
      integral = 0;  // Reset integral when starting/stopping
      recovering = false;

      String msg = robotActive ? "STARTED" : "STOPPED";
      SerialBT.println("Robot " + msg);

      // Reset timing
      lastPIDTime = micros();

      while (digitalRead(pushButton) == LOW)
        ;  // Wait for release
    }
  }
}

void pidLineFollow() {
  unsigned long currentTime = micros();
  float elapsedTime = (currentTime - lastPIDTime) / 1000000.0;  // Convert to seconds
  lastPIDTime = currentTime;

  // Safety: prevent huge time jumps
  if (elapsedTime > 0.1) elapsedTime = 0.01;

  int sensorValues[5];
  int sensorWeights[5] = { -2, -1, 0, 1, 2 };
  int positionSum = 0, activeSensors = 0;

  // Read calibrated sensors
  for (int i = 0; i < sensorCount; i++) {
    sensorValues[i] = !(digitalRead(sensorPins[i]));
    positionSum += sensorValues[i] * sensorWeights[i];
    activeSensors += sensorValues[i];
  }

  if (activeSensors == 0) {
    if (!recovering) {
      lineLostTime = millis();
      recovering = true;
      integral = 0;  // Reset integral during recovery
      SerialBT.println("Line lost - recovering...");
    }

    // Check if recovery timeout exceeded
    if (millis() - lineLostTime > recoverTimeout) {
      SerialBT.println("Line lost for too long - STOPPING");
      stopMotors();
      robotActive = false;
      digitalWrite(statusLED, LOW);
      return;
    }

    // Recovery maneuver based on last known error
    if (lastError < 0) {
      setMotorSpeed(-recoverSpeed, recoverSpeed);  // Turn left
    } else {
      setMotorSpeed(recoverSpeed, -recoverSpeed);  // Turn right
    }

    return;
  } else {
    if (recovering) {
      SerialBT.println("Line found!");
      recovering = false;
    }
  }

  // Calculate error
  error = (float)positionSum / activeSensors;

  // === Deadband to avoid micro oscillation ===
  if (abs(error) < DEADBAND_THRESHOLD) error = 0;

  // === Time-based PID Calculation ===
  float P = error;
  integral += error * elapsedTime;  // Time-based integral

  // Clamp integral (anti-windup)
  integral = constrain(integral, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);

  float D = (error - lastError) / elapsedTime;  // Time-based derivative
  lastError = error;

  // === PID Output ===
  float output = (config.Kp * P) + (config.Ki * integral) + (config.Kd * D);

  int leftSpeed = constrain(config.baseSpeed - output, -config.maxSpeed, config.maxSpeed);
  int rightSpeed = constrain(config.baseSpeed + output, -config.maxSpeed, config.maxSpeed);

  setMotorSpeed(leftSpeed, rightSpeed);

  // === Debug Output (every 100ms) ===
  if (millis() - lastDebugTime > 100) {
    debugOutput(sensorValues, error, output, leftSpeed, rightSpeed, elapsedTime);
    lastDebugTime = millis();
  }
}

// === BLUETOOTH COMMAND PROCESSING ===
void checkSerialCommands() {
  if (SerialBT.available()) {
    String input = SerialBT.readStringUntil('\n');
    input.trim();

    if (input.length() > 0) {
      processCommand(input);
    }
  }
}

void processCommand(String command) {
  command.toLowerCase();
  int spaceIndex = command.indexOf(' ');

  String cmd = command;
  String valueStr = "";

  if (spaceIndex > 0) {
    cmd = command.substring(0, spaceIndex);
    valueStr = command.substring(spaceIndex + 1);
  }

  if (cmd == "kp" && valueStr != "") {
    config.Kp = valueStr.toFloat();
    SerialBT.println("Kp set to: " + String(config.Kp));
  } else if (cmd == "ki" && valueStr != "") {
    config.Ki = valueStr.toFloat();
    SerialBT.println("Ki set to: " + String(config.Ki));
  } else if (cmd == "kd" && valueStr != "") {
    config.Kd = valueStr.toFloat();
    SerialBT.println("Kd set to: " + String(config.Kd));
  } else if (cmd == "base" && valueStr != "") {
    config.baseSpeed = valueStr.toInt();
    SerialBT.println("Base speed set to: " + String(config.baseSpeed));
  } else if (cmd == "max" && valueStr != "") {
    config.maxSpeed = valueStr.toInt();
    SerialBT.println("Max speed set to: " + String(config.maxSpeed));
  } else if (cmd == "start") {
    robotActive = true;
    digitalWrite(statusLED, HIGH);
    SerialBT.println("Started via Bluetooth");
  } else if (cmd == "stop") {
    robotActive = false;
    digitalWrite(statusLED, LOW);
    stopMotors();
    SerialBT.println("Stopped via Bluetooth");
  } else if (cmd == "status") {
    printStatus();
  } else if (cmd == "help") {
    printHelp();
  } else {
    SerialBT.println("Unknown command: " + command);
    SerialBT.println("Type 'help' for available commands");
  }
}

void printStatus() {
  SerialBT.println("=== CURRENT STATUS ===");
  SerialBT.println("PID Values: Kp=" + String(config.Kp) + " Ki=" + String(config.Ki) + " Kd=" + String(config.Kd));
  SerialBT.println("Speeds: Base=" + String(config.baseSpeed) + " Max=" + String(config.maxSpeed));
  SerialBT.println("Robot: " + String(robotActive ? "RUNNING" : "STOPPED"));
}

void printHelp() {
  SerialBT.println("=== AVAILABLE COMMANDS ===");
  SerialBT.println("KP <value>   - Set proportional gain");
  SerialBT.println("KI <value>   - Set integral gain");
  SerialBT.println("KD <value>   - Set derivative gain");
  SerialBT.println("BASE <value> - Set base speed");
  SerialBT.println("MAX <value>  - Set max speed");
  SerialBT.println("CAL          - Calibrate sensors");
  SerialBT.println("START        - Start robot");
  SerialBT.println("STOP         - Stop robot");
  SerialBT.println("STATUS       - Show current settings");
  SerialBT.println("HELP         - Show this help");
}

// === DEBUG AND MONITORING ===
void debugOutput(int sensorValues[], float error, float output,
                 int leftSpeed, int rightSpeed, float elapsedTime) {
  SerialBT.print("Sensors: ");
  for (int i = 0; i < sensorCount; i++) {
    SerialBT.print(sensorValues[i]);
    SerialBT.print(" ");
  }

  SerialBT.print("| Error: ");
  SerialBT.print(error, 3);

  SerialBT.print(" | PID: ");
  SerialBT.print(config.Kp * error, 1);
  SerialBT.print(",");
  SerialBT.print(config.Ki * integral, 1);
  SerialBT.print(",");
  SerialBT.print(config.Kd * (error - lastError) / elapsedTime, 1);

  SerialBT.print(" | Output: ");
  SerialBT.print(output, 1);

  SerialBT.print(" | Motors: ");
  SerialBT.print(leftSpeed);
  SerialBT.print(",");
  SerialBT.print(rightSpeed);

  SerialBT.print(" | Time: ");
  SerialBT.print(elapsedTime * 1000, 1);
  SerialBT.println("ms");
}



// === MOTOR CONTROL (Unchanged) ===
void setMotorSpeed(int left, int right) {
  // Left motor → ENB, IN3, IN4
  if (right >= 0) {
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
  } else {
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
    right = -right;
  }

  // Right motor → ENA, IN1, IN2
  if (left >= 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    left = -left;
  }

  analogWrite(enA, constrain(right, 0, config.maxSpeed));
  analogWrite(enB, constrain(left, 0, config.maxSpeed));
}

void stopMotors() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  analogWrite(enA, 0);
  analogWrite(enB, 0);
}
