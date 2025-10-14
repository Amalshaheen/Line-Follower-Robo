// === Motor Pins === (Updated to match your working setup)
#define ENA 14
#define IN1 27
#define IN2 26
#define ENB 12
#define IN3 18
#define IN4 15

// === IR Sensor Pins === (Same as your working setup)
#define IR1 34
#define IR2 35
#define IR3 32
#define IR4 33
#define IR5 25

// === Push button and LED ===
const int pushButton = 4;
const int statusLED = 2;

// === PWM setup ===
#define PWM_FREQ 20000
#define PWM_RES 8
#define CH_A 0
#define CH_B 1

// === Control parameters ===
float Kp = 25;          // Reduced from 40 for smoother operation
int baseSpeed = 180;    // Base motor speed (0–255)
bool robotActive = false;

void setup() {
  Serial.begin(115200);

  // Motor setup
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  ledcSetup(CH_A, PWM_FREQ, PWM_RES);
  ledcAttachPin(ENA, CH_A);
  ledcSetup(CH_B, PWM_FREQ, PWM_RES);
  ledcAttachPin(ENB, CH_B);

  // IR sensor setup
  pinMode(IR1, INPUT);
  pinMode(IR2, INPUT);
  pinMode(IR3, INPUT);
  pinMode(IR4, INPUT);
  pinMode(IR5, INPUT);
  
  // Initialize button and LED
  pinMode(pushButton, INPUT_PULLUP);
  pinMode(statusLED, OUTPUT);
  
  stopMotors();
  Serial.println("PID Line Follower Ready");
  Serial.println("Press button to start/stop");
  Serial.println("Sensor Logic: 0=Black Line, 1=White Surface");
}

void loop() {
  // Check push button (same as your working code)
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
    pidLineFollow();
  } else {
    // When stopped, you can still read sensors for testing
    readSensors();
    delay(500);
  }
}

// --- Improved error calculation with more cases ---
int calculateError() {
  int s1 = digitalRead(IR1);
  int s2 = digitalRead(IR2);
  int s3 = digitalRead(IR3);
  int s4 = digitalRead(IR4);
  int s5 = digitalRead(IR5);

  // Extended error cases for better line tracking
  if (s1 == 0 && s2 == 0 && s3 == 0 && s4 == 0 && s5 == 0) return 0;   // All white - use last error or stop
  if (s1 == 1 && s1 == 1 && s1 == 1 && s1 == 1 && s1 == 1) return -10; // All black - lost line
  
  // Standard line following cases with weights
  if (s3 == 0) return 0;  // Perfectly on line
  
  if (s2 == 0) return -1; // Slight left
  if (s4 == 0) return 1;  // Slight right
  
  if (s1 == 0) return -2; // Medium left
  if (s5 == 0) return 2;  // Medium right
  
  // Edge cases for sharp turns
  if (s1 == 0 && s2 == 0) return -3; // Sharp left
  if (s4 == 0 && s5 == 0) return 3;  // Sharp right
  
  return 0; // Default case
}

void pidLineFollow() {
  // Calculate error
  int error = calculateError();

  // Calculate proportional correction
  float correction = Kp * (error / 2.0); // Normalize error

  // Adjust motor speeds with constraints
  int leftMotorSpeed = baseSpeed + correction;
  int rightMotorSpeed = baseSpeed - correction;

  // Constrain motor speeds to valid PWM range
  leftMotorSpeed = constrain(leftMotorSpeed, 0, 255);
  rightMotorSpeed = constrain(rightMotorSpeed, 0, 255);

  // Apply speeds
  motorA(leftMotorSpeed);
  motorB(rightMotorSpeed);

  // Debug output
  Serial.printf("Error: %2d | Correction: %6.1f | L:%3d R:%3d\n", 
                error, correction, leftMotorSpeed, rightMotorSpeed);
}

// --- Motor control functions (compatible with your L298N) ---
void motorA(int speed) {
  if (speed > 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  } else if (speed < 0) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    speed = -speed;
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
  }
  ledcWrite(CH_A, speed);
}

void motorB(int speed) {
  if (speed > 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  } else if (speed < 0) {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    speed = -speed;
  } else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
  }
  ledcWrite(CH_B, speed);
}

void stopMotors() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  ledcWrite(CH_A, 0);
  ledcWrite(CH_B, 0);
}

void readSensors() {
  Serial.print("Sensor Test: ");
  for (int i = 0; i < 5; i++) {
    int pin;
    switch(i) {
      case 0: pin = IR1; break;
      case 1: pin = IR2; break;
      case 2: pin = IR3; break;
      case 3: pin = IR4; break;
      case 4: pin = IR5; break;
    }
    int value = digitalRead(pin);
    Serial.print("S");
    Serial.print(i + 1);
    Serial.print(":");
    Serial.print(value);
    Serial.print(" ");
  }
  Serial.println();
}