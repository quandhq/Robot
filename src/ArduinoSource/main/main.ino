#include <MPU6050_tockn.h>
#include <Wire.h>

// --- Hardware Pins ---
int enA = 9; int in1 = 8; int in2 = 7;
int enB = 3; int in3 = 5; int in4 = 4;

MPU6050 mpu6050(Wire);

// --- Systems Architecture Parameters ---
float targetAngle =   0; // YOUR MEASURED BIAS: The "True Zero"
float Kp = 45.0;           // Proportional: Strength of the correction
float Kd = 1.8;            // Derivative: Dampening the oscillation
int minPower = 55;         // Stiction threshold for yellow motors
int kickOffset = 10;       // COMPENSATION FOR 2.86° SLOP: Snaps gears shut

float lastAngle = 0;

int ledPin = 13; // Built-in LED

void setup() {
  Serial.begin(9600);
  Wire.begin();
  mpu6050.begin();
  
  pinMode(ledPin, OUTPUT);
  pinMode(enA, OUTPUT); pinMode(in1, OUTPUT); pinMode(in2, OUTPUT);
  pinMode(enB, OUTPUT); pinMode(in3, OUTPUT); pinMode(in4, OUTPUT);

  // --- STATE 1: PREPARING ---
  digitalWrite(ledPin, HIGH); 
  Serial.println("PREPARING: Hold the robot steady. Calibration starts in 3 seconds...");
  delay(3000); // 3-second startup delay

  // --- STATE 2: CALIBRATING ---
  digitalWrite(ledPin, LOW); // LED OFF means "Stay Still!"
  Serial.println("CALIBRATING: Processing Ground Truth...");
  
  mpu6050.calcGyroOffsets(true);
  Serial.println();
  Serial.println("CALIBRATING: taking averaging...");
  float totalAngle = 0;
  for(int i = 0; i < 100; i++) {
    mpu6050.update();
    totalAngle += mpu6050.getAngleX();
    delay(10);
  }
  
  targetAngle = totalAngle / 100.0;
  lastAngle = targetAngle;

  // --- STATE 3: ACTIVE ---
  digitalWrite(ledPin, HIGH); // LED ON means "Motors Live!"
  Serial.println("SYSTEM ACTIVE: Release the robot now.");
}

void loop() {
  mpu6050.update();
  float currentAngle = mpu6050.getAngleX();

  // 1. Calculate Error based on your First-Principles Bias
  float error = currentAngle - targetAngle;

  // 2. Calculate Velocity (Derivative) for dampening
  float errorRate = currentAngle - lastAngle;
  lastAngle = currentAngle;

  // 3. Control Law (PD Controller)
  float output = (error * Kp) + (errorRate * Kd);

  // 4. Architect's Deadband Logic
  // We use 0.8 degrees as a "Quiet Zone" to ignore tiny sensor noise
  if (abs(error) < 0.8) {
    stopMotors();
  } else {
    driveMotors(output);
  }
}

void driveMotors(float output) {
  // Use the kickOffset to jump across the 2.86 degree gear backlash
  int speed = constrain(abs(output) + minPower + kickOffset, 0, 255);

  if (output > 0) { // System needs to move Forward
    digitalWrite(in1, LOW);  digitalWrite(in2, HIGH);
    digitalWrite(in3, LOW);  digitalWrite(in4, HIGH);
  } else if (output < 0) { // System needs to move Backward
    digitalWrite(in1, HIGH); digitalWrite(in2, LOW);
    digitalWrite(in3, HIGH); digitalWrite(in4, LOW);
  }

  analogWrite(enA, speed);
  analogWrite(enB, speed);
}

void stopMotors() {
  analogWrite(enA, 0);
  analogWrite(enB, 0);
  // Optional: Set pins LOW to ensure no "creep"
  digitalWrite(in1, LOW); digitalWrite(in2, LOW);
  digitalWrite(in3, LOW); digitalWrite(in4, LOW);
}