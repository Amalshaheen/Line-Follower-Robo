// Sensor pins
const int sensorPins[] = {34, 35, 32, 33, 25};
const int sensorCount = 5;

// L298N Motor Driver pins (with IN3 = 18)
const int enA = 14;
const int in1 = 27;
const int in2 = 26;
const int enB = 12;
const int in3 = 18;  // Using GPIO 18 instead of 13
const int in4 = 15;

// Push button and LED
const int pushButton = 4;
const int statusLED = 2;

// Motor speeds (0-255)
int motorSpeed = 200;
int turnSpeed = 150;
bool robotActive = false;

void setup() {
  Serial.begin(115200);
  
  // Initialize sensor pins
  for (int i = 0; i < sensorCount; i++) {
    pinMode(sensorPins[i], INPUT);
  }
  
  // Initialize motor pins
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  
  // Initialize button and LED
  pinMode(pushButton, INPUT_PULLUP);
  pinMode(statusLED, OUTPUT);
  
  stopMotors();
  Serial.println("Line Follower Ready - IN3 on GPIO 18");
  Serial.println("Press button to start/stop");
  Serial.println("Sensor Logic: 0=Black Line, 1=White Surface");
}

void loop() {
  // Check push button
  if (digitalRead(pushButton) == LOW) {
    delay(50); // Debounce
    if (digitalRead(pushButton) == LOW) {
      robotActive = !robotActive;
      digitalWrite(statusLED, robotActive ? HIGH : LOW);
      Serial.println(robotActive ? "Robot STARTED" : "Robot STOPPED");
      stopMotors();
      delay(500); // Prevent multiple toggles
    }
  }
  
  if (robotActive) {
    lineFollow();
  } else {
    // When stopped, you can still read sensors for testing
    readSensors();
    delay(500);
  }
}

void lineFollow() {
  int sensorValues[5];
  
  // Read all sensors
  for (int i = 0; i < sensorCount; i++) {
    sensorValues[i] = digitalRead(sensorPins[i]);
  }
  
  // Display sensor values for debugging
  Serial.print("Sensors: ");
  for (int i = 0; i < sensorCount; i++) {
    Serial.print(sensorValues[i]);
    Serial.print(" ");
  }
  
  // Line following logic - FLIPPED (0=Black Line, 1=White Surface)
  if (sensorValues[2] == 0) {
    // Center sensor on line - move forward
    moveForward();
    Serial.println("Forward");
  }
  else if (sensorValues[1] == 0) {
    // Left of center - slight left turn
    turnLeft();
    Serial.println("Left");
  }
  else if (sensorValues[3] == 0) {
    // Right of center - slight right turn
    turnRight();
    Serial.println("Right");
  }
  else if (sensorValues[0] == 0) {
    // Far left - sharp left turn
    sharpLeft();
    Serial.println("Sharp Left");
  }
  else if (sensorValues[4] == 0) {
    // Far right - sharp right turn
    sharpRight();
    Serial.println("Sharp Right");
  }
  else {
    // No line detected - stop
    stopMotors();
    Serial.println("No line - Stopped");
  }
}

void readSensors() {
  Serial.print("Sensor Test: ");
  for (int i = 0; i < sensorCount; i++) {
    int value = digitalRead(sensorPins[i]);
    Serial.print("S");
    Serial.print(i + 1);
    Serial.print(":");
    Serial.print(value);
    Serial.print(" ");
  }
  Serial.println();
}

// Motor control functions (unchanged)
void moveForward() {
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  analogWrite(enA, motorSpeed);
  analogWrite(enB, motorSpeed);
}

void turnLeft() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  analogWrite(enA, turnSpeed);
  analogWrite(enB, turnSpeed);
}

void turnRight() {
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  analogWrite(enA, turnSpeed);
  analogWrite(enB, turnSpeed);
}

void sharpLeft() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  analogWrite(enA, turnSpeed);
  analogWrite(enB, motorSpeed);
}

void sharpRight() {
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  analogWrite(enA, motorSpeed);
  analogWrite(enB, turnSpeed);
}

void stopMotors() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  analogWrite(enA, 0);
  analogWrite(enB, 0);
}