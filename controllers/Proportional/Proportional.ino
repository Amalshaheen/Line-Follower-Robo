// === Motor Pins (same as before) ===
#define ENA 18
#define IN1 21
#define IN2 22
#define ENB 19
#define IN3 23
#define IN4 17

// === IR Sensor Pins ===
#define IR1 34
#define IR2 35
#define IR3 32
#define IR4 33
#define IR5 25

// === PWM setup ===
#define PWM_FREQ 20000
#define PWM_RES 8
#define CH_A 0
#define CH_B 1

// === Control parameters ===
float Kp = 40;          // Proportional gain (tune this)
int baseSpeed = 180;    // Base motor speed (0–255)

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
}

// --- Motor helper functions ---
void motorA(int speed) {
  if (speed > 0) { digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW); }
  else if (speed < 0) { digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH); speed = -speed; }
  else { digitalWrite(IN1, LOW); digitalWrite(IN2, LOW); }

  ledcWrite(CH_A, constrain(speed, 0, 255));
}

void motorB(int speed) {
  if (speed > 0) { digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW); }
  else if (speed < 0) { digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH); speed = -speed; }
  else { digitalWrite(IN3, LOW); digitalWrite(IN4, LOW); }

  ledcWrite(CH_B, constrain(speed, 0, 255));
}

// --- Calculate position error ---
int calculateError(int s1, int s2, int s3, int s4, int s5) {
  // Assign weights to sensors: leftmost = -2, rightmost = +2
  if (s1 == 0 && s2 == 0 && s3 == 1 && s4 == 0 && s5 == 0) return 0;  // center
  if (s1 == 0 && s2 == 1 && s3 == 1 && s4 == 0 && s5 == 0) return -1; // slightly left
  if (s1 == 1 && s2 == 1 && s3 == 0 && s4 == 0 && s5 == 0) return -2; // far left
  if (s1 == 0 && s2 == 0 && s3 == 1 && s4 == 1 && s5 == 0) return +1; // slightly right
  if (s1 == 0 && s2 == 0 && s3 == 0 && s4 == 1 && s5 == 1) return +2; // far right

  // If no line detected, return previous error or 0
  return 0;
}

void loop() {
  // Read sensors
  int s1 = digitalRead(IR1);
  int s2 = digitalRead(IR2);
  int s3 = digitalRead(IR3);
  int s4 = digitalRead(IR4);
  int s5 = digitalRead(IR5);

  // Calculate error
  int error = calculateError(s1, s2, s3, s4, s5);

  // Calculate correction
  float correction = Kp * error;

  // Adjust motor speeds
  int leftMotorSpeed  = baseSpeed + correction;
  int rightMotorSpeed = baseSpeed - correction;

  // Apply speeds
  motorA(leftMotorSpeed);
  motorB(rightMotorSpeed);

  // Debug
  Serial.printf("Sensors: %d %d %d %d %d | Error: %d | L:%d R:%d\n",
                s1, s2, s3, s4, s5, error, leftMotorSpeed, rightMotorSpeed);

  delay(30); // adjust loop timing
}
