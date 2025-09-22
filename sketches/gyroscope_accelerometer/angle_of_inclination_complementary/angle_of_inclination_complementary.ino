#include <Wire.h>
#include <I2Cdev.h>
#include <MPU6050.h>
#include <math.h>

MPU6050 mpu(0x68); // I2C address (0x68 is default)

int16_t accY, accZ, gyroX;
float accAngle = 0;
float gyroAngle = 0;
float currentAngle = 0;
float prevAngle = 0;

float alpha = 0.9938; // complementary filter weight
float dt = 0.01; // sample time in seconds (10 ms)
unsigned long prevTime = 0;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  mpu.initialize();
  // apply offsets
	  mpu.setYAccelOffset(-493);
	  mpu.setZAccelOffset(857);
	  mpu.setXGyroOffset(132);
}

void loop() {
  accY = mpu.getAccelerationY();
  accZ = mpu.getAccelerationZ();
  gyroX= mpu.getRotationX();
  accAngle = atan2(accY, accZ) * RAD_TO_DEG; // calculate angle from accelerometer
  float gyroRate = map(gyroX, -32768, 32767, -250, 250); // degrees/sec
  gyroAngle += gyroRate * dt; // calculate angle from gyroscope
  // combine with complementary filter
  currentAngle = alpha * (prevAngle + gyroRate * dt) + (1 - alpha) * accAngle;
  prevAngle = currentAngle;
  // wait 1 second before printing
  if (millis() - prevTime >= 1000) {
    Serial.print("Tilt angle: ");
    Serial.println(currentAngle);
    prevTime = millis();
  }
  delay(10); // 10 ms sampling
}