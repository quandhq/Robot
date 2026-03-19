#include <MPU6050_tockn.h>
#include <Wire.h>

// --- TB6612FNG Hardware Pins ---
const int enA = 9;   // PWMA
const int ain1 = 8;  // ain1
const int ain2 = 7;  // ain2
const int enB = 3;   // PWMB
const int bin1 = 5;  // bin1
const int bin2 = 4;  // bin2
// Note: STBY is hardwired to 5V per your setup

MPU6050 mpu6050(Wire);

// --- PID Parameters (Adjusted for JGB37-520 Torque) ---
float targetAngle = 0; 
float Kp = 40.0;      // Slightly lower to reduce "shaking"
float Kd = 1.8;       // Lowered to match the lower Kp
float Ki = 180.0;     // Lowered significantly to stop the "wandering" 
int minPower = 45;    // Metal gears only need ~45 to start moving
float trim = -4;     // Start at 0 and only adjust by 0.1 at a time

// --- Real-Time Control Variables ---
unsigned long lastTime;
const int sampleTime = 10; // 10ms = 100Hz (The industry standard for small robots)

float lastAngle = 0;
float errorIntegral = 0;
int ledPin = 13; 

void setup() {
  Serial.begin(115200); // Increased speed to reduce CPU "blocking" time
  Wire.begin();
  mpu6050.begin();
  
  pinMode(ledPin, OUTPUT);
  pinMode(enA, OUTPUT); pinMode(ain1, OUTPUT); pinMode(ain2, OUTPUT);
  pinMode(enB, OUTPUT); pinMode(bin1, OUTPUT); pinMode(bin2, OUTPUT);
  digitalWrite(ledPin, HIGH); 

  // Calibration sequence...
  delay(3000); 
  digitalWrite(ledPin, LOW); 
  mpu6050.calcGyroOffsets(true);
  
  // Take average for targetAngle
  float totalAngle = 0;
  for(int i = 0; i < 100; i++) {
    mpu6050.update();
    totalAngle += mpu6050.getAngleX();
    delay(10);
  }
  targetAngle = totalAngle / 100.0;
  lastAngle = targetAngle;
  lastTime = millis();

  digitalWrite(ledPin, HIGH); 
}

void loop() {
  unsigned long now = millis();
  int timeChange = now - lastTime;

  // ENSURE DETERMINISTIC TIMING (The 100Hz Heartbeat)
  if (timeChange >= sampleTime) {
    mpu6050.update();
    float currentAngle = mpu6050.getAngleX();
    // DEBUG: Print this to see if the numbers are actually changing
    Serial.print(targetAngle);
    Serial.print(" - ");
    Serial.print(currentAngle);
    Serial.print(" - ");
    float error = currentAngle - (targetAngle + trim);
    Serial.print(error);

    // 1. SAFETY ENVELOPE (The Floor Check)
    if (abs(error) > 45.0) {
      stopMotors();
      errorIntegral = 0; // Wipe memory instantly
      lastTime = now;
      return; 
    }

    // 2. DERIVATIVE (Velocity)
    float errorRate = currentAngle - lastAngle;
    lastAngle = currentAngle;

    // 3. INTEGRAL (Memory with Anti-Windup)
    // Only integrate when within the "Recoverable Zone" (5 degrees)
    if (abs(error) < 20.0) {
      errorIntegral += error;
    } else {
      errorIntegral = 0; 
    }

    // 4. CONTROL LAW
    // Note: Ki is now multiplied by 0.01 inside the logic
    float output = (error * Kp) + (errorRate * Kd) + (errorIntegral * Ki * 0.01);

    // 5. DEADBAND & EXECUTION
    if (abs(error) < 0.05) {
      stopMotors();
      errorIntegral = 0;
    } else {
      driveMotors(output);
    }

    lastTime = now;
  }
}

void driveMotors(float output) {
  // If the error is tiny, don't jitter the motors
  if (abs(output) < 0.05) {
    stopMotors();
    return;
  }
  // DEADZONE JUMP: If the motor needs to move, 
  // we start it at minPower immediately so the gears engage.
  int speed = abs(output) + minPower; 
  speed = constrain(speed, 0, 255);

  Serial.print(" - ");
  Serial.println(speed);

  if (output > 0) { // drive backward
    // Motor A backward
    digitalWrite(ain1, LOW);
    digitalWrite(ain2, HIGH);
    analogWrite(enA, speed); 

    // Motor B backward
    digitalWrite(bin1, HIGH);
    digitalWrite(bin2, LOW);
    analogWrite(enB, speed);
  } else if (output < 0) { // drive forward
    // Motor A Forward
    digitalWrite(ain1, HIGH);
    digitalWrite(ain2, LOW);
    analogWrite(enA, speed); 

    // Motor B Forward
    digitalWrite(bin1, LOW);
    digitalWrite(bin2, HIGH);
    analogWrite(enB, speed);
  } else {
    stopMotors();
  }
}

void stopMotors() {
  analogWrite(enA, 0);
  analogWrite(enB, 0);
  digitalWrite(ain1, LOW); digitalWrite(ain2, LOW);
  digitalWrite(bin1, LOW); digitalWrite(bin2, LOW);
}