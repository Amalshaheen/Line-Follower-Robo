// PID Line Follower for ESP32 + L298N + 5 IR Sensors
// Simplified version with only essential parameters + Ki added

#include "BluetoothSerial.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to enable it
#endif

BluetoothSerial SerialBT;

const int sensorPins[] = {34, 35, 32, 33, 25};
const int sensorCount = 5;

const int enA = 14;
const int in1 = 27;
const int in2 = 26;
const int enB = 12;
const int in3 = 18;
const int in4 = 15;

const int pushButton = 4;
const int statusLED = 2;

// === PID PARAMETERS ===
float Kp = 34;
float Ki = 0;
float Kd = 150;
int baseSpeed = 100;

// Fixed parameters
const int maxSpeed = 200;
const int recoverSpeed = 65;
const int recoverDelay = 5;

float error = 0, lastError = 0, integral = 0;
bool robotActive = false;

// Integral windup limit
const float INTEGRAL_LIMIT = 100.0;

// Bluetooth variables
unsigned long lastBtPrint = 0;
const int BT_PRINT_INTERVAL = 100; // ms

void setup() {
  Serial.begin(115200);

  // Initialize Bluetooth
  SerialBT.begin("LineFollower_BT");
  Serial.println("Bluetooth Started! Pair with 'LineFollower_BT'");

  for (int i = 0; i < sensorCount; i++) pinMode(sensorPins[i], INPUT);

  pinMode(enA, OUTPUT); pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT); pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT); pinMode(in4, OUTPUT);

  pinMode(pushButton, INPUT_PULLUP);
  pinMode(statusLED, OUTPUT);

  stopMotors();
  Serial.println("PID Line Follower - Kp, Ki, Kd enabled");

  if (SerialBT.hasClient()) {
    SerialBT.println("Ready - Commands: START, STOP, DATA, CONFIG, SENSORS, KP, KI, KD, SPEED");
  }
}

void loop() {
  if (SerialBT.available()) {
    String command = SerialBT.readString();
    command.trim();
    handleBTCommand(command);
  }

  if (digitalRead(pushButton) == LOW) {
    delay(100);
    if (digitalRead(pushButton) == LOW) {
      robotActive = !robotActive;
      digitalWrite(statusLED, robotActive ? HIGH : LOW);
      stopMotors();
      String msg = robotActive ? "STARTED" : "STOPPED";
      Serial.println("Robot " + msg);
      if (SerialBT.hasClient()) SerialBT.println("ROBOT_" + msg);
      delay(500);
    }
  }

  if (robotActive) pidLineFollow();
  else { readSensors(); delay(300); }
}

void pidLineFollow() {
  int sensorValues[5];
  int sensorWeights[5] = {-2, -1, 0, 1, 2};
  int positionSum = 0, activeSensors = 0;

  // Read sensors (0 = white, 1 = black)
  for (int i = 0; i < sensorCount; i++) {
    sensorValues[i] = digitalRead(sensorPins[i]) == 0 ? 1 : 0;
    positionSum += sensorValues[i] * sensorWeights[i];
    activeSensors += sensorValues[i];
  }

  // === Line lost recovery ===
  if (activeSensors == 0) {
    integral = 0; // reset integral when line is lost
    if (lastError > 0) setMotorSpeed(-recoverSpeed, recoverSpeed);
    else setMotorSpeed(recoverSpeed, -recoverSpeed);
    delay(recoverDelay);
    return;
  }

  error = (float)positionSum / activeSensors;

  // === Deadband to avoid micro oscillation ===
  if (abs(error) < 0.1) error = 0;

  // === PID terms ===
  float P = error;
  integral += error;

  // Clamp integral (anti-windup)
  if (integral > INTEGRAL_LIMIT) integral = INTEGRAL_LIMIT;
  if (integral < -INTEGRAL_LIMIT) integral = -INTEGRAL_LIMIT;

  float D = error - lastError;
  lastError = error;

  // === PID output ===
  float output = (Kp * P) + (Ki * integral) + (Kd * D);

  // === Dynamic speed scaling ===
  float speedFactor = 1.0 - min(abs(error) / 2.0, 1.0);
  int adjustedBase = baseSpeed * speedFactor + 40;

  int leftSpeed = constrain(adjustedBase - output, -maxSpeed, maxSpeed);
  int rightSpeed = constrain(adjustedBase + output, -maxSpeed, maxSpeed);

  setMotorSpeed(leftSpeed, rightSpeed);
}

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

  analogWrite(enA, constrain(right, 0, maxSpeed));
  analogWrite(enB, constrain(left, 0, maxSpeed));
}

void stopMotors() {
  digitalWrite(in1, LOW); digitalWrite(in2, LOW);
  digitalWrite(in3, LOW); digitalWrite(in4, LOW);
  analogWrite(enA, 0); analogWrite(enB, 0);
}

void readSensors() {
  Serial.print("Sensors: ");
  for (int i = 0; i < sensorCount; i++) {
    Serial.print(digitalRead(sensorPins[i]));
    Serial.print(" ");
  }
  Serial.println();
}

// === Bluetooth Functions ===
void handleBTCommand(String command) {
  command.toUpperCase();

  if (command == "START") {
    robotActive = true;
    digitalWrite(statusLED, HIGH);
    SerialBT.println("ROBOT_STARTED");
  }
  else if (command == "STOP") {
    robotActive = false;
    digitalWrite(statusLED, LOW);
    stopMotors();
    SerialBT.println("ROBOT_STOPPED");
  }
  else if (command == "DATA") {
    int sensorValues[5];
    for (int i = 0; i < sensorCount; i++) {
      sensorValues[i] = digitalRead(sensorPins[i]) == 0 ? 1 : 0;
    }
    sendBTData(error, 0, 0, 0, sensorValues);
  }
  else if (command == "CONFIG") {
    sendBTConfig();
  }
  else if (command == "SENSORS") {
    readSensorsBT();
  }
  else if (command.startsWith("KP")) {
    Kp = command.substring(2).toFloat();
    SerialBT.println("KP_SET:" + String(Kp, 1));
    sendBTConfig();
  }
  else if (command.startsWith("KI")) {
    Ki = command.substring(2).toFloat();
    SerialBT.println("KI_SET:" + String(Ki, 2));
    sendBTConfig();
  }
  else if (command.startsWith("KD")) {
    Kd = command.substring(2).toFloat();
    SerialBT.println("KD_SET:" + String(Kd, 1));
    sendBTConfig();
  }
  else if (command.startsWith("SPEED")) {
    baseSpeed = command.substring(5).toInt();
    baseSpeed = constrain(baseSpeed, 0, 255);
    SerialBT.println("BASE_SPEED_SET:" + String(baseSpeed));
    sendBTConfig();
  }
  else {
    SerialBT.println("UNKNOWN_COMMAND: " + command);
    SerialBT.println("Available: START, STOP, DATA, CONFIG, SENSORS, KP, KI, KD, SPEED");
  }
}

void sendBTData(float error, float output, int leftSpeed, int rightSpeed, int sensorValues[]) {
  String data = "DATA:" +
                String(error, 2) + "," +
                String(output, 2) + "," +
                String(leftSpeed) + "," +
                String(rightSpeed) + ",";
                
  for (int i = 0; i < sensorCount; i++) {
    data += String(sensorValues[i]);
    if (i < sensorCount - 1) data += ",";
  }
  
  SerialBT.println(data);
}

void sendBTConfig() {
  String config = "CONFIG:Kp=" + String(Kp, 1) +
                  ",Ki=" + String(Ki, 2) +
                  ",Kd=" + String(Kd, 1) +
                  ",BaseSpeed=" + String(baseSpeed);
  SerialBT.println(config);
}

void readSensorsBT() {
  String sensorReadings = "SENSORS:";
  for (int i = 0; i < sensorCount; i++) {
    sensorReadings += String(digitalRead(sensorPins[i]));
    if (i < sensorCount - 1) sensorReadings += ",";
  }
  SerialBT.println(sensorReadings);
}
