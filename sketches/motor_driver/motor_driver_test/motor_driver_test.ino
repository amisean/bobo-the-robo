// === Motor Pin Definitions (PHASE/ENABLE mode) ===

// Left Motor (Motor A)
#define A_PHASE 4
#define A_ENABLE 5

// Right Motor (Motor B)
#define B_PHASE 7
#define B_ENABLE 6

// === Motor Flip Settings ===
#define FLIP_LEFT  false
#define FLIP_RIGHT true  // Set to true if your right motor spins the wrong way

// === Per-direction speed scaling (tune empirically) ===
#define LEFT_FORWARD_SCALE   1.00
#define LEFT_REVERSE_SCALE   1.00
#define RIGHT_FORWARD_SCALE  1.00
#define RIGHT_REVERSE_SCALE  1.00

void setup() {
  Serial.begin(9600);
  delay(1000);  // Give Serial Monitor time to connect

  pinMode(A_PHASE, OUTPUT);
  pinMode(A_ENABLE, OUTPUT);
  pinMode(B_PHASE, OUTPUT);
  pinMode(B_ENABLE, OUTPUT);

  // MODE pin is wired to VCC physically — no code needed
}

void setMotor(int enablePin, int phasePin, int speed, bool flip, bool isLeft) {
  speed = constrain(speed, -255, 255);
  bool direction = (speed >= 0);
  int pwm = abs(speed);

  if (flip) direction = !direction;

  // Apply direction-specific scaling
  if (isLeft) {
    pwm *= direction ? LEFT_FORWARD_SCALE : LEFT_REVERSE_SCALE;
  } else {
    pwm *= direction ? RIGHT_FORWARD_SCALE : RIGHT_REVERSE_SCALE;
  }

  // Debug print for right motor
  if (!isLeft) {
    Serial.print("Right pwm = ");
    Serial.println(pwm);
  }

  digitalWrite(phasePin, direction);
  analogWrite(enablePin, pwm);
}

void loop() {
  // --- FORWARD ---
  Serial.println("Forward");
  setMotor(A_ENABLE, A_PHASE, 255, FLIP_LEFT, true);    // Left motor
  setMotor(B_ENABLE, B_PHASE, 255, FLIP_RIGHT, false);  // Right motor
  delay(2000);

  // --- STOP ---
  Serial.println("Stop");
  setMotor(A_ENABLE, A_PHASE, 0, FLIP_LEFT, true);
  setMotor(B_ENABLE, B_PHASE, 0, FLIP_RIGHT, false);
  delay(1000);

  // --- REVERSE ---
  Serial.println("Reverse");
  setMotor(A_ENABLE, A_PHASE, -255, FLIP_LEFT, true);
  setMotor(B_ENABLE, B_PHASE, -255, FLIP_RIGHT, false);
  delay(2000);

  // --- STOP ---
  Serial.println("Stop");
  setMotor(A_ENABLE, A_PHASE, 0, FLIP_LEFT, true);
  setMotor(B_ENABLE, B_PHASE, 0, FLIP_RIGHT, false);
  delay(1000);
}