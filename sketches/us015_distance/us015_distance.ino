#include <NewPing.h>

#define TRIGGER_PIN 9
#define ECHO_PIN    8
#define MAX_DISTANCE 200

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);

int getStableDistance(int samples = 5) {
  long sum = 0;
  int validCount = 0;
  for (int i = 0; i < samples; i++) {
    int d = sonar.ping_cm();
    if (d > 0 && d <= MAX_DISTANCE) {
      sum += d;
      validCount++;
    }
    delay(50);
  }
  return (validCount > 0) ? (sum / validCount) : -1;
}

void setup() {
  Serial.begin(9600);
}

void loop() {
  int distance = getStableDistance(5);
  Serial.print("Distance: ");
  if (distance > 0) {
    Serial.print(distance);
    Serial.println(" cm");
  } else {
    Serial.println("Out of range or no echo");
  }
  delay(1000);
}