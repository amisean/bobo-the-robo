#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include "math.h"

MPU6050 mpu(0x68);  // manual I2C address

int16_t accY, accZ;
float accAngle;

void setup() {
  Wire.begin();
  Serial.begin(9600);
  
  // Wake up MPU6050 (it starts in sleep mode)
  mpu.initialize();
  mpu.setSleepEnabled(false);  

  // Check connection
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed");
    while (1); // Stop here
  }

  Serial.println("MPU6050 ready");
}

void loop() {
  accZ = mpu.getAccelerationZ();
  accY = mpu.getAccelerationY();

  accAngle = atan2(accY, accZ) * RAD_TO_DEG;

  if (!isnan(accAngle)) {
    Serial.println(accAngle);
  }

  delay(100);  // optional small delay
}