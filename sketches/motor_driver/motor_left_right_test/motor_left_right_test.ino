// Motor A (left)
#define AIN1 5  // PWM
#define AIN2 4

// Motor B (right)
#define BIN1 7
#define BIN2 6  // PWM

void setup() {
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);

  Serial.begin(9600);
}

void loop() {
  // --- MOTOR A FORWARD ---
  Serial.println("Motor A forward");
  analogWrite(AIN1, 255);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);
  delay(2000);

  // --- STOP ---
  Serial.println("Stop");
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  delay(1000);

  // --- MOTOR B FORWARD ---
  Serial.println("Motor B forward");
  analogWrite(BIN1, 255);
  digitalWrite(BIN2, LOW);
  delay(2000);

  // --- STOP ---
  Serial.println("Stop");
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);
  delay(1000);

  // --- MOTOR A REVERSE ---
  Serial.println("Motor A reverse");
  analogWrite(AIN2, 255);
  digitalWrite(AIN1, LOW);
  delay(2000);

  // --- STOP ---
  Serial.println("Stop");
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  delay(1000);

  // --- MOTOR B REVERSE ---
  Serial.println("Motor B reverse");
  analogWrite(BIN2, 255);
  digitalWrite(BIN1, LOW);
  delay(2000);

  // --- STOP ---
  Serial.println("Stop");
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);
  delay(1000);
}