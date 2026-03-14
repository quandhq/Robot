#include <MPU6050_tockn.h>
#include <Wire.h>

// --- Hardware Pins ---
int enA = 9; int in1 = 8; int in2 = 7;
int enB = 3; int in3 = 5; int in4 = 4;

MPU6050 mpu6050(Wire);

// --- Systems Architecture Parameters ---
float targetAngle = 0; 
float Kp = 60;           // Bring Kp back up slightly now that minPower is lower
float Kd = 3.5;           // High damping to stop the jiggles
float Ki = 250.0;         // High willpower, but slightly lower to avoid pendulum swings
int minPower = 150;       // THIS IS THE KEY: Bring this back down!
int kickOffset = 20;      
float trim = -0.5;        // Increased nudge to stop the drift

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
  pinMode(enA, OUTPUT); pinMode(in1, OUTPUT); pinMode(in2, OUTPUT);
  pinMode(enB, OUTPUT); pinMode(in3, OUTPUT); pinMode(in4, OUTPUT);
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
    Serial.println(error);

    // 1. SAFETY ENVELOPE (The Floor Check)
    if (abs(error) > 40.0) {
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
    if (abs(error) < 10.0) {
      errorIntegral += error;
    } else {
      errorIntegral = 0; 
    }

    // 4. CONTROL LAW
    // Note: Ki is now multiplied by 0.01 inside the logic
    float output = (error * Kp) + (errorRate * Kd) + (errorIntegral * Ki * 0.01);

    // 5. DEADBAND & EXECUTION
    if (abs(error) < 0.1) { // Tighter deadband for shorter L
      stopMotors();
      errorIntegral = 0;
    } else {
      driveMotors(output);
    }

    lastTime = now;
  }
}

void driveMotors(float output) {
  // Now speed can actually scale between 120 and 255
  int speed = constrain(abs(output) + minPower + kickOffset, 0, 255);

  if (output > 0.1) {
    digitalWrite(in1, HIGH); digitalWrite(in2, LOW);
    digitalWrite(in3, HIGH); digitalWrite(in4, LOW);
    analogWrite(enA, speed);
    analogWrite(enB, speed);
  } else if (output < -0.1) {
    digitalWrite(in1, LOW);  digitalWrite(in2, HIGH);
    digitalWrite(in3, LOW);  digitalWrite(in4, HIGH);
    analogWrite(enA, speed);
    analogWrite(enB, speed);
  } else {
    stopMotors();
  }
}

void stopMotors() {
  analogWrite(enA, 0);
  analogWrite(enB, 0);
  digitalWrite(in1, LOW); digitalWrite(in2, LOW);
  digitalWrite(in3, LOW); digitalWrite(in4, LOW);
}