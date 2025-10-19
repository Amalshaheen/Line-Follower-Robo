// Enhanced PID Line Follower for ESP32 + L298N + 5 IR Sensors
// With time-based PID, non-blocking recovery, and serial tuning

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
  float Kp = 15.0;  // Reduced from 34
  float Ki = 0.0;   // Enabled integral
  float Kd = 80.0;  // Reduced from 150
  int baseSpeed = 120;
  int maxSpeed = 200;
};

PIDConfig config;

// Recovery parameters
const int recoverSpeed = 65;
const unsigned long recoverTimeout = 500;  // ms
const int recoverDuration = 50;            // ms

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

// Sensor calibration
int sensorMin[5] = { 4095, 4095, 4095, 4095, 4095 };
int sensorMax[5] = { 0, 0, 0, 0, 0 };
bool calibrated = false;

void setup() {
  Serial.begin(115200);
  Serial.println("=== Enhanced Line Follower ===");
  Serial.println("Commands: KP, KI, KD, BASE, MAX, CAL, START, STOP");
  Serial.println("Example: KP 20.5");

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

  // Auto-calibrate on startup
  calibrateSensors();

  lastPIDTime = micros();
}

void loop() {
  checkButton();
  checkSerialCommands();

  if (robotActive) {
    pidLineFollow();
  } else {
    // Read and display sensors when inactive
    if (millis() - lastDebugTime > 1000) {
      readAndPrintSensors();
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
      Serial.println("Robot " + msg);

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
    sensorValues[i] = readCalibratedSensor(i);
    positionSum += sensorValues[i] * sensorWeights[i];
    activeSensors += sensorValues[i];
  }

  // === Non-blocking Line Lost Recovery ===
  if (activeSensors == 0) {
    if (!recovering) {
      lineLostTime = millis();
      recovering = true;
      integral = 0;  // Reset integral during recovery
      Serial.println("Line lost - recovering...");
    }

    // Check if recovery timeout exceeded
    if (millis() - lineLostTime > recoverTimeout) {
      Serial.println("Line lost for too long - STOPPING");
      stopMotors();
      robotActive = false;
      digitalWrite(statusLED, LOW);
      return;
    }

    // Recovery maneuver based on last known error
    if (lastError > 0) {
      setMotorSpeed(-recoverSpeed, recoverSpeed);  // Turn left
    } else {
      setMotorSpeed(recoverSpeed, -recoverSpeed);  // Turn right
    }

    return;
  } else {
    if (recovering) {
      Serial.println("Line found!");
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

  // === Dynamic Speed Scaling ===
  // Less aggressive scaling
  float speedFactor = 1.0 - min(abs(error) / 2.5, 0.4);
  int adjustedBase = config.baseSpeed * speedFactor + 80;

  int leftSpeed = constrain(adjustedBase - output, -config.maxSpeed, config.maxSpeed);
  int rightSpeed = constrain(adjustedBase + output, -config.maxSpeed, config.maxSpeed);

  setMotorSpeed(leftSpeed, rightSpeed);

  // === Debug Output (every 100ms) ===
  if (millis() - lastDebugTime > 100) {
    debugOutput(sensorValues, error, output, leftSpeed, rightSpeed, elapsedTime);
    lastDebugTime = millis();
  }
}

// === SENSOR CALIBRATION FUNCTIONS ===
void calibrateSensors() {
  Serial.println("Calibrating sensors... Move over line and background");
  digitalWrite(statusLED, HIGH);

  // Reset calibration
  for (int i = 0; i < sensorCount; i++) {
    sensorMin[i] = 4095;
    sensorMax[i] = 0;
  }

  // Calibrate for 3 seconds
  unsigned long startTime = millis();
  while (millis() - startTime < 3000) {
    for (int i = 0; i < sensorCount; i++) {
      int value = analogRead(sensorPins[i]);
      if (value < sensorMin[i]) sensorMin[i] = value;
      if (value > sensorMax[i]) sensorMax[i] = value;
    }
    delay(10);
  }

  calibrated = true;
  digitalWrite(statusLED, LOW);

  Serial.println("Calibration complete:");
  for (int i = 0; i < sensorCount; i++) {
    Serial.print("S");
    Serial.print(i);
    Serial.print(": ");
    Serial.print(sensorMin[i]);
    Serial.print("-");
    Serial.print(sensorMax[i]);
    Serial.print(" ");
  }
  Serial.println();
}

int readCalibratedSensor(int sensorIndex) {
  int rawValue = analogRead(sensorPins[sensorIndex]);

  if (!calibrated) {
    // Fallback to digital reading if not calibrated
    return rawValue > 2000 ? 1 : 0;  // Simple threshold
  }

  // Map to 0-1000 range and apply threshold
  int calibratedValue = map(rawValue, sensorMin[sensorIndex], sensorMax[sensorIndex], 0, 1000);
  calibratedValue = constrain(calibratedValue, 0, 1000);

  return calibratedValue > 600 ? 1 : 0;  // Threshold for line detection
}

// === SERIAL COMMAND PROCESSING ===
void checkSerialCommands() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
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
    Serial.println("Kp set to: " + String(config.Kp));
  } else if (cmd == "ki" && valueStr != "") {
    config.Ki = valueStr.toFloat();
    Serial.println("Ki set to: " + String(config.Ki));
  } else if (cmd == "kd" && valueStr != "") {
    config.Kd = valueStr.toFloat();
    Serial.println("Kd set to: " + String(config.Kd));
  } else if (cmd == "base" && valueStr != "") {
    config.baseSpeed = valueStr.toInt();
    Serial.println("Base speed set to: " + String(config.baseSpeed));
  } else if (cmd == "max" && valueStr != "") {
    config.maxSpeed = valueStr.toInt();
    Serial.println("Max speed set to: " + String(config.maxSpeed));
  } else if (cmd == "cal") {
    calibrateSensors();
  } else if (cmd == "start") {
    robotActive = true;
    digitalWrite(statusLED, HIGH);
    Serial.println("Started via serial");
  } else if (cmd == "stop") {
    robotActive = false;
    digitalWrite(statusLED, LOW);
    stopMotors();
    Serial.println("Stopped via serial");
  } else if (cmd == "status") {
    printStatus();
  } else if (cmd == "help") {
    printHelp();
  } else {
    Serial.println("Unknown command: " + command);
    Serial.println("Type 'help' for available commands");
  }
}

void printStatus() {
  Serial.println("=== CURRENT STATUS ===");
  Serial.println("PID Values: Kp=" + String(config.Kp) + " Ki=" + String(config.Ki) + " Kd=" + String(config.Kd));
  Serial.println("Speeds: Base=" + String(config.baseSpeed) + " Max=" + String(config.maxSpeed));
  Serial.println("Robot: " + String(robotActive ? "RUNNING" : "STOPPED"));
  Serial.println("Calibrated: " + String(calibrated ? "YES" : "NO"));
}

void printHelp() {
  Serial.println("=== AVAILABLE COMMANDS ===");
  Serial.println("KP <value>   - Set proportional gain");
  Serial.println("KI <value>   - Set integral gain");
  Serial.println("KD <value>   - Set derivative gain");
  Serial.println("BASE <value> - Set base speed");
  Serial.println("MAX <value>  - Set max speed");
  Serial.println("CAL          - Calibrate sensors");
  Serial.println("START        - Start robot");
  Serial.println("STOP         - Stop robot");
  Serial.println("STATUS       - Show current settings");
  Serial.println("HELP         - Show this help");
}

// === DEBUG AND MONITORING ===
void debugOutput(int sensorValues[], float error, float output,
                 int leftSpeed, int rightSpeed, float elapsedTime) {
  Serial.print("Sensors: ");
  for (int i = 0; i < sensorCount; i++) {
    Serial.print(sensorValues[i]);
    Serial.print(" ");
  }

  Serial.print("| Error: ");
  Serial.print(error, 3);

  Serial.print(" | PID: ");
  Serial.print(config.Kp * error, 1);
  Serial.print(",");
  Serial.print(config.Ki * integral, 1);
  Serial.print(",");
  Serial.print(config.Kd * (error - lastError) / elapsedTime, 1);

  Serial.print(" | Output: ");
  Serial.print(output, 1);

  Serial.print(" | Motors: ");
  Serial.print(leftSpeed);
  Serial.print(",");
  Serial.print(rightSpeed);

  Serial.print(" | Time: ");
  Serial.print(elapsedTime * 1000, 1);
  Serial.println("ms");
}

void readAndPrintSensors() {
  Serial.print("Raw Sensors: ");
  for (int i = 0; i < sensorCount; i++) {
    int raw = analogRead(sensorPins[i]);
    int calibrated = readCalibratedSensor(i);
    Serial.print("S");
    Serial.print(i);
    Serial.print(":");
    Serial.print(raw);
    Serial.print("(");
    Serial.print(calibrated);
    Serial.print(") ");
  }
  Serial.println();
}

// === MOTOR CONTROL (Unchanged) ===
void setMotorSpeed(int left, int right) {
  // Left motor → ENB, IN3, IN4
  if (left >= 0) {
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
  } else {
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
    left = -left;
  }

  // Right motor → ENA, IN1, IN2
  if (right >= 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    right = -right;
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
