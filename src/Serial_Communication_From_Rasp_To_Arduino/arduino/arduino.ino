#include <Wire.h>

// --- Hardware Pins ---
int enA = 9; int in1 = 8; int in2 = 7;
int enB = 3; int in3 = 5; int in4 = 4;


const int number_of_byte = 3;

char myBuffer[number_of_byte];

void setup() {
  Serial.begin(9600);
  Serial.println("ready!");

  pinMode(enA, OUTPUT); pinMode(in1, OUTPUT); pinMode(in2, OUTPUT);
  pinMode(enB, OUTPUT); pinMode(in3, OUTPUT); pinMode(in4, OUTPUT);
}

void driveMotors(char* myBuffer) {
  uint8_t left_motor_speed = myBuffer[0] <= 255 && myBuffer[0] >=0 ? myBuffer[0] : 0;
  uint8_t right_motor_speed = myBuffer[1] <= 255 && myBuffer[1] >=0 ? myBuffer[1] : 0;
  uint8_t direction = myBuffer[2] == 0 || myBuffer[2] == 1 ? myBuffer[2] : 0;
  if(direction == 0)
  {
    digitalWrite(in1, HIGH); digitalWrite(in2, LOW);
    digitalWrite(in3, HIGH); digitalWrite(in4, LOW);
    analogWrite(enA, left_motor_speed);
    analogWrite(enB, right_motor_speed);
  }
  else
  {
    digitalWrite(in1, LOW); digitalWrite(in2, HIGH);
    digitalWrite(in3, LOW); digitalWrite(in4, HIGH);
    analogWrite(enA, left_motor_speed);
    analogWrite(enB, right_motor_speed);
  }

}

void loop() {
  if (Serial.available() > 0) {
    // Store the return value to verify the data integrity
    size_t count = Serial.readBytes(myBuffer, number_of_byte);

    if (count == number_of_byte) {
      driveMotors(myBuffer);
    } else {
      // Handle partial read/timeout error
    }
  }
}
