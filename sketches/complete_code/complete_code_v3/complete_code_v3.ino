#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include "math.h"
#include <DRV8835MotorShield.h>

#define Kp  80
#define Ki  80
#define Kd  0.5
#define sampleTime  0.005
#define targetAngle -1.2

float alpha = 0.9934;           // --- Complementary filter weight ---
unsigned long prevTime = 0;

MPU6050 mpu(0x68);  // --- I2C address (0x68 is default) ---
DRV8835MotorShield motors;

// --- Declare global variables
int16_t accY, accZ, gyroX;
volatile float motorPower, gyroRate;
volatile float prevMotorPower = 0;
volatile float accAngle, gyroAngle, currentAngle, prevAngle=0, error, prevError=0, errorSum=0;
volatile byte count=0;
int distanceCm;
float pid_p = 0;
float pid_i = 0;
float pid_d = 0;

void init_PID() {  
  // Initialize Timer1
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
  motors.flipM2(true);  // invert M2
  mpu.initialize();
  
  // Apply offsets
  mpu.setYAccelOffset(-493);
  mpu.setZAccelOffset(857);
  mpu.setXGyroOffset(132);

  init_PID();

  Serial.begin(9600);
}

void loop() {
  // --- Read acceleration and gyroscope values ---
  accY = mpu.getAccelerationY();
  accZ = mpu.getAccelerationZ();  
  gyroX = mpu.getRotationX();

  // --- Use DRV8835 library to set motor speed ---
  int pwm = constrain((int)motorPower, -400, 400);
  if (abs(pwm) < 50) pwm = 0;
  motors.setM1Speed(pwm);
  motors.setM2Speed(pwm);
}

// --- The Interrupt Service Routine will be called every 5 ms
ISR(TIMER1_COMPA_vect) {

  // --- Calculate angle from accelerometer ---
  accAngle = atan2(accY, accZ) * RAD_TO_DEG;

  // --- Calculate angle from gyroscope ---
  gyroRate = gyroX / 131.0; // degrees/sec

  // --- Combine with complementary filter ---
  currentAngle = alpha * (prevAngle + gyroRate * sampleTime) + (1 - alpha) * accAngle;

  // --- FAILBREAK: if angle too steep, stop motors ---
  if (abs(currentAngle) > 30) {
    motorPower = 0;
    motors.setM1Speed(0);
    motors.setM2Speed(0);
    return;  // Skip rest of ISR
  }

  // --- Calculate error ---
  error = currentAngle - targetAngle;
  errorSum += error * sampleTime;
  // --- Anti-Windup for Ki ---
  if (motorPower >= 400 || motorPower <= -400 || abs(error) > 10) {
    pid_i = 0;  // reset I-term when output is saturated or large error
  }
  errorSum = constrain(errorSum, -450, 450);

  // --- Calculate P, I and D values ---
  pid_p = Kp * error;
  pid_i += Ki * error * sampleTime;
  pid_d = Kd * (currentAngle - prevAngle) / sampleTime;

  // --- Calculate output from PID values ---
  motorPower = pid_p + pid_i - pid_d;

  prevAngle = currentAngle;
    
  // --- Toggle the led on pin13 every second ---
  count++;
  if(count == 200)  {
    count = 0;
    digitalWrite(13, !digitalRead(13));
  }
}