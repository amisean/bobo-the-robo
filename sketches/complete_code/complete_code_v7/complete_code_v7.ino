#include <Wire.h>
#include <I2Cdev.h>
#include <MPU6050.h>
#include <math.h>
#include <DRV8835MotorShield.h>
#include <util/atomic.h>
#include <string.h>  // for memcpy

#define Kp  80 // proportional coefficient
#define Ki  60 // integration coefficient
#define Kd  0.5 // differential coefficient
#define targetAngle -1.2

MPU6050 mpu(0x68); // I2C address (0x68 is default)
DRV8835MotorShield motors(4, 5, 7, 6); // set output pins (M1DIR, M1PWM, M2DIR, M2PWM)

// declare global variables
const float Ts = 0.005f; // sample time 5ms
float alpha = 0.9938; // filter coefficient
const float INV_GYRO = 1.0f / 131.0f; // convert raw gyroX (LSB) to °/s for the ±250 °/s range (MPU6050 scale)
volatile int16_t accY, accZ, gyroX;
volatile int16_t motorCmd = 0;   // final command for motors, -400..400
volatile float motorPower, gyroRate;
volatile float accAngle, currentAngle, prevAngle=0, error, errorSum=0;
volatile byte count=0;
float pid_p = 0;
float pid_i = 0;
float pid_d = 0;

// serial plotter support
struct PlotSample {
  float angle, error, P, I, D, out;
  bool ready;
};
volatile PlotSample plotSample = {0};
volatile uint32_t isrTicks = 0; // counts ISR ticks (5 ms each)
const uint8_t PRINT_DECIMATION = 0; // print every Nth ISR (reduce bandwidth)
volatile uint8_t printDecimCnt = 0;

void init_PID() {  
  // initialise Timer1 to provide a 5 ms scheduler for PID
  cli(); // stop interrupts so register writes aren’t interrupted
  TCCR1A = 0; // clear Timer1 control A -> no PWM outputs from Timer1
  TCCR1B = 0; // clear Timer1 control B -> timer stopped while we set it up
  OCR1A  = 9999; // compare value (TOP in CTC mode)
  TCCR1B |= (1 << WGM12); // CTC mode (Mode 4): count up to OCR1A, then reset to 0
  TCCR1B |= (1 << CS11); // prescaler = 8 (CS12:0 = 010)
  TIMSK1 |= (1 << OCIE1A); // enable “compare A match” interrupt
  sei(); // re-enable global interrupts
}

void setup() {
  pinMode(13, OUTPUT); // status LED
  motors.flipM2(true); // invert M2, depends on wiring
  mpu.initialize();
  mpu.setYAccelOffset(-493); // apply offset
  mpu.setZAccelOffset(857); // apply offset
  mpu.setXGyroOffset(132); // apply offset
  init_PID(); // start 5 ms control interrupts
  Serial.begin(115200); // set baud rate (for debugging)
  Serial.println("P;I;D;angle;error;out");
}

void loop() {
  // read acceleration and gyroscope values
  int16_t ay = mpu.getAccelerationY();
  int16_t az = mpu.getAccelerationZ();  
  int16_t gx = mpu.getRotationX();
  // publish to shareds atomically
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    accY = ay;
    accZ = az;
    gyroX = gx;
  }
  // set motor speed
  int16_t cmd; // grab the latest command atomically (16-bit read on 8-bit MCU)
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { cmd = motorCmd; }
  if (abs(cmd) < 50) cmd = 0; // optional deadband
  motors.setM1Speed(cmd);
  motors.setM2Speed(cmd);
  // print latest plotsample if ready (non-blocking)
  PlotSample local;
  bool doPrint = false;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    if (plotSample.ready) {
      // copy from volatile safely
      memcpy(&local, (const void*)&plotSample, sizeof(PlotSample));
      plotSample.ready = false; // mark consumed
      doPrint = true;
    }
  }
  if (doPrint) {
    // csv print:
    Serial.print(local.P,4);        Serial.print(';');
    Serial.print(local.I,4);        Serial.print(';');
    Serial.print(local.D,4);        Serial.print(';');
    Serial.print(local.angle,4);    Serial.print(';');
    Serial.print(local.error,4);    Serial.print(';');
    Serial.println(local.out,4);
  }
}

// interrupt service routine will be called every 5 ms
ISR(TIMER1_COMPA_vect) {
  isrTicks++; // advance tick counter every ISR
  // sensor fusion
  accAngle = atan2(accY, accZ) * RAD_TO_DEG; // accel tilt (deg)
  gyroRate = gyroX * INV_GYRO;
  currentAngle = alpha * (prevAngle + gyroRate * Ts)
               + (1.0f - alpha) * accAngle;
  // SAFETY: bail out if we’re too far
  if (fabs(currentAngle) > 30.0f) {
    errorSum = 0; // also clear I to recover cleanly
    pid_i = 0;
    motorPower = 0;
    motorCmd = 0;
    prevAngle = currentAngle;
    // still update LED timer
    count++;
    if (count == 200) {
      count = 0;
      digitalWrite(13, !digitalRead(13));
    }
    return; // do not produce plot samples while tripped
  }
  // PID
  error = currentAngle - targetAngle; // calculate error
  errorSum += error * Ts; // integrate error over time
  errorSum = constrain(errorSum,
                       -400.0f / max(1.0f, (float)Ki),
                        400.0f / max(1.0f, (float)Ki)); // anti-windup: clamp integrator
  pid_p = Kp * error; // proportional
  pid_i = Ki * errorSum; // integral
  pid_d = Kd * gyroRate; // derivative on measurement
  motorPower = pid_p + pid_i - pid_d; // calculate output from PID values
  motorPower = constrain(motorPower, -400.0f, 400.0f); // saturate output to driver’s expected range (Pololu lib: -400..400)
  motorCmd = (int16_t)motorPower; // publish int16 command atomically for loop()
  prevAngle = currentAngle;
  // toggle the led on pin13 every second (200 * 5 ms)
  count++;
  if(count == 200)  {
    count = 0;
    digitalWrite(13, !digitalRead(13));
  }
  // prepare decimated data for plotting
  printDecimCnt++;
  if (printDecimCnt >= PRINT_DECIMATION) {
    printDecimCnt = 0;
    PlotSample s;
    s.angle = currentAngle;
    s.error = error;
    s.P = pid_p;
    s.I = pid_i;
    s.D = pid_d;
    s.out = motorPower;
    s.ready = true;
    // publish snapshot for loop() to print.
    // (no need for ATOMIC_BLOCK here; we are in ISR with interrupts disabled.)
    plotSample = s;
  }
}