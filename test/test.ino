// === Pin configuration ===
// Motor A (L298N)
#define ENA 18
#define IN1 21
#define IN2 22

// Motor B (L298N)
#define ENB 19
#define IN3 23
#define IN4 17

// IR Sensors
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

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n--- ESP32 Test: Motors + IR Sensors ---");

  // Motor pins
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // Sensor pins
  pinMode(IR1, INPUT);
  pinMode(IR2, INPUT);
  pinMode(IR3, INPUT_PULLUP);
  pinMode(IR4, INPUT_PULLUP);
  pinMode(IR5, INPUT_PULLUP);

  // PWM setup
  ledcSetup(CH_A, PWM_FREQ, PWM_RES);
  ledcAttachPin(ENA, CH_A);
  ledcSetup(CH_B, PWM_FREQ, PWM_RES);
  ledcAttachPin(ENB, CH_B);

  Serial.println("Setup complete.");
}

void motorA(int speed) {
  if (speed > 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    ledcWrite(CH_A, speed);
  } else if (speed < 0) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    ledcWrite(CH_A, -speed);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    ledcWrite(CH_A, 0);
  }
}

void motorB(int speed) {
  if (speed > 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    ledcWrite(CH_B, speed);
  } else if (speed < 0) {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    ledcWrite(CH_B, -speed);
  } else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    ledcWrite(CH_B, 0);
  }
}

void loop() {
  // --- Read sensors ---
  int s1 = digitalRead(IR1);
  int s2 = digitalRead(IR2);
  int s3 = digitalRead(IR3);
  int s4 = digitalRead(IR4);
  int s5 = digitalRead(IR5);

  Serial.printf("IR: %d %d %d %d %d\n", s1, s2, s3, s4, s5);

  // --- Motor test ---
  Serial.println("Motors Forward");
  motorA(200);
  motorB(200);
  delay(1500);

  Serial.println("Stop");
  motorA(0);
  motorB(0);
  delay(800);

  Serial.println("Motors Reverse");
  motorA(-200);
  motorB(-200);
  delay(1500);

  Serial.println("Stop");
  motorA(0);
  motorB(0);
  delay(1500);
}
