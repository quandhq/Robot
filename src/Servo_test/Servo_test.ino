#include <Servo.h>

Servo testServo;

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  testServo.attach(9);
  testServo.write(90);
  delay(500);
}

void loop() {
  digitalWrite(LED_BUILTIN, HIGH);
  testServo.write(90);   // center
  delay(2000);
  digitalWrite(LED_BUILTIN, LOW);
  testServo.write(45);   // down
  delay(2000);
  digitalWrite(LED_BUILTIN, HIGH);
  testServo.write(135);  // up
  delay(2000);
  digitalWrite(LED_BUILTIN, LOW);
}
