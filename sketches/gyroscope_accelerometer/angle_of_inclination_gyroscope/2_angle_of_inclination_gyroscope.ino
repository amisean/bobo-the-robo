#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"

MPU6050 mpu(0x68);  // Manual I2C address

int16_t gyroX;
float gyroRate;
float gyroAngle = 0;

unsigned long currTime, prevTime = 0, loopTime;

void setup() {
  Serial.begin(9600);
  Wire.begin();

  // Wake up MPU6050 before using it
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);     // PWR_MGMT_1 register
  Wire.write(0x00);     // Wake up
  Wire.endTransmission(true);

  mpu.initialize();
}

void loop() {
  currTime = millis();
  loopTime = currTime - prevTime;
  prevTime = currTime;

  gyroX = mpu.getRotationX();
  gyroRate = gyroX / 131.0;  // convert to °/s for ±250 dps
  gyroAngle = gyroAngle + gyroRate * loopTime / 1000.0;

  Serial.println(gyroAngle);
}