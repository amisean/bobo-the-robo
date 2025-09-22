#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include "math.h"
#include <NewPing.h>
#include <DRV8835MotorShield.h>

#define TRIGGER_PIN 9
#define ECHO_PIN 8
#define MAX_DISTANCE 75

#define Kp  20
#define Ki  25
#define Kd  0
#define sampleTime  0.005
#define targetAngle -0.5

float alpha = 0.9934;           // complementary filter weight
unsigned long prevTime = 0;

MPU6050 mpu(0x68);  // I2C address (0x68 is default)
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);
DRV8835MotorShield motors;

int16_t accY, accZ, gyroX;
volatile float motorPower, gyroRate;
volatile float accAngle, gyroAngle, currentAngle, prevAngle=0, error, prevError=0, errorSum=0;
volatile byte count=0;
int distanceCm;

void init_PID() {  
  // initialize Timer1
  cli();          // disable global interrupts
  TCCR1A = 0;     // set entire TCCR1A register to 0
  TCCR1B = 0;     // same for TCCR1B    
  // set compare match register to set sample time 5ms
  OCR1A = 9999;    
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS11 bit for prescaling by 8
  TCCR1B |= (1 << CS11);
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);
  sei();          // enable global interrupts
}

void setup() {
  pinMode(13, OUTPUT);  // status LED

  Serial.begin(9600);   // Serial debugging output
  delay(1000);  // Let Serial settle (especially useful with USB-to-serial adapters)
  motors.flipM2(true);  // invert M2

  mpu.initialize();
  mpu.setYAccelOffset(-493);
  mpu.setZAccelOffset(857);
  mpu.setXGyroOffset(132);

  init_PID();
}

void loop() {
  accY = mpu.getAccelerationY();
  accZ = mpu.getAccelerationZ();  
  gyroX = mpu.getRotationX();

  // Use DRV8835 library to set motor speed
  int pwm = constrain((int)motorPower, -255, 255);
  if (abs(pwm) < 30) pwm = 0;
  motors.setM1Speed(pwm);
  motors.setM2Speed(pwm);

  // Debug output
  Serial.print("Angle: ");
  Serial.print(currentAngle);
  Serial.print("  Motor: ");
  Serial.println(motorPower);

  // Optional: measure distance every 100ms
  /*
  if ((count % 20) == 0) {
    distanceCm = sonar.ping_cm();
  }
  if ((distanceCm < 20) && (distanceCm != 0)) {
    motors.setM1Speed(-motorPower);
    motors.setM2Speed(motorPower);
  }
  */
}

// The ISR will be called every 5 milliseconds
ISR(TIMER1_COMPA_vect) {
  accAngle = atan2(accY, accZ) * RAD_TO_DEG;
  gyroRate = gyroX / 131.0;

  currentAngle = alpha * (currentAngle + gyroRate * sampleTime) + (1 - alpha) * accAngle;

  error = currentAngle - targetAngle;

  // Only integrate error if motor power is not saturated
  if (abs(motorPower) < 255) {
    errorSum += error;
    errorSum = constrain(errorSum, -300, 300);
  }

  motorPower = Kp * error + Ki * errorSum * sampleTime - Kd * gyroRate;

  if (abs(motorPower) < 30) motorPower = 0;

  prevAngle = currentAngle;

  count++;
  if (count == 200) {
    count = 0;
    digitalWrite(13, !digitalRead(13));
  }
}