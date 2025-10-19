// PID Line Follower for ESP32 + L298N + 5 IR Sensors
// Simplified version with only essential parameters + Ki added

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


void setup() {
  Serial.begin(115200);

  for (int i = 0; i < sensorCount; i++) pinMode(sensorPins[i], INPUT);

  pinMode(enA, OUTPUT); pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT); pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT); pinMode(in4, OUTPUT);

  pinMode(pushButton, INPUT_PULLUP);
  pinMode(statusLED, OUTPUT);

  stopMotors();
  Serial.println("PID Line Follower - Kp, Ki, Kd enabled");


}

void loop() {
  if (digitalRead(pushButton) == LOW) {
    delay(100);
    if (digitalRead(pushButton) == LOW) {
      robotActive = !robotActive;
      digitalWrite(statusLED, robotActive ? HIGH : LOW);
      stopMotors();
      String msg = robotActive ? "STARTED" : "STOPPED";
      Serial.println("Robot " + msg);
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
