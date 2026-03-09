#include <MPU6050_tockn.h>
#include <Wire.h>

MPU6050 mpu6050(Wire);

float totalAngle = 0;
int readings = 0;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  mpu6050.begin();
  
  Serial.println("--- System Architect: Bias Analysis ---");
  Serial.println("Keep the robot perfectly still on a level surface.");
  
  mpu6050.calcGyroOffsets(true);
}

void loop() {
  mpu6050.update();
  
  float currentAngle = mpu6050.getAngleX();
  totalAngle += currentAngle;
  readings++;

  // Calculate the average drift/bias every 100 readings
  if (readings >= 100) {
    float averageBias = totalAngle / 100;
    
    Serial.print("Current Angle: ");
    Serial.print(currentAngle);
    Serial.print(" | Calculated Average Bias: ");
    Serial.println(averageBias);
    
    // Reset for next batch
    totalAngle = 0;
    readings = 0;
  }
  
  delay(10); 
}