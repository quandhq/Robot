#include <MPU6050_tockn.h>
#include <Wire.h>

// --- TB6612FNG Hardware Pins ---
const int enA = 9;   // PWMA
const int ain1 = 8;  // ain1
const int ain2 = 7;  // ain2
const int enB = 3;   // PWMB
const int bin1 = 5;  // bin1
const int bin2 = 4;  // bin2
// Note: STBY is hardwired to 5V

MPU6050 mpu6050(Wire);

// --- PID Parameters (Adjusted for JGB37-520 Torque) ---
float targetAngle = 0; 
float Kp = 50.0;      // Slightly lower to reduce "shaking"//50 IS FINE
float Kd = 4.0;       // Lowered to match the lower Kp
float Ki = 200.0;     // stop the "drifting"  //200 is fine 
int minPower = 10;    // lower to 10 min to stop the oscillating because of the motor gear slap
float trim = 1.0;     // Start at 0 and only adjust by 0.1 at a time -> CHANGE THIS TO FIND THE BALANCE POINT

// --- Real-Time Control Variables ---
unsigned long lastTime;
#define SAMPLE_TIME 10 // 10ms = 100Hz

float lastAngle = 0;
float errorIntegral = 0;
int ledPin = 13; 

// --- Telemetry ---
unsigned long lastTelemetry;

float externalTargetOffset = 0; // The Pi will change this

// --------------DEBUG COMMANDING--------------------------
long check_time = 0;
float command[3] = {-2, 0, 2};
uint8_t index = 0;
void debug_commanding()
{
  if(millis() - check_time >= 1000)
  {
    if(index >= 3)
    {
      index = 0;
    }
    externalTargetOffset = command[index];
    ++index;
    check_time = millis();
  }
}
//---------------------------end---------------------


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
    float sample = mpu6050.getAngleX(); 
    push(sample);
    totalAngle += sample;
    delay(10);
  }
  targetAngle = totalAngle / 100.0;
  lastAngle = targetAngle;
  lastTime = millis();
  lastTelemetry = millis();
  check_time = millis();

  // ONLY send this after calibration is 100% finished
  Serial.println("ready!"); //<<<<IMPORTANT, NOT A DEBUGGING 
  digitalWrite(ledPin, HIGH); // Visual confirmation
}



float currentAngle = 0;
void loop() {
  // debug_commanding();
  //Listen for Pi commanding
  if(Serial.available() >= 1)
  {
    int8_t receivedOffset = Serial.read();
    // Pi send from -5 to 5
    externalTargetOffset = abs((float)receivedOffset) <= 5 ? (float)receivedOffset : 0;
  }

  unsigned long now = millis();
  int timeChange = now - lastTime;

  mpu6050.update();
  float sample = mpu6050.getAngleX();
  push(sample);

  // ENSURE DETERMINISTIC TIMING (The 100Hz Heartbeat)
  if (timeChange >= SAMPLE_TIME) {
    // mpu6050.update();
    // float sample = mpu6050.getAngleX();
    // push(sample);
    currentAngle = sample;//get_average();
    // DEBUG: Print this to see if the numbers are actually changing
    // Serial.print("target: ");
    // Serial.print(targetAngle);
    // Serial.print(" - ");
    // Serial.print("current: ");
    // Serial.print(currentAngle);
    // Serial.print(" - ");
    float error = currentAngle - (targetAngle + trim + externalTargetOffset);
    // Serial.print("Error: ");
    // Serial.print(error);
    // Serial.println();
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

    // 3. INTEGRAL (The "Leaky" Version)
    if (abs(error) < 10.0) {
        errorIntegral += error;
    } else {
        errorIntegral *= 0.9; 
    }

    // 4. CONTROL LAW (Calculate the math BEFORE checking the deadband)
    float output = (error * Kp) + (errorRate * Kd) + (errorIntegral * Ki * 0.01);

    // 5. DEADBAND & EXECUTION
    if (abs(error) < 0.05) {
        stopMotors();
    } else {
        driveMotors(output);
    }

    lastTime = now;
  }

  //Notify Rasp that arduino's loop function is still running
  if(now - lastTelemetry > 50)
  {
    Serial.println(get_average());
    lastTelemetry = now;
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

  // Serial.print(" - ");
  // Serial.println(speed);

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

// -------------------------------circular buffer--------------------------------------------
const uint8_t N = 20;
const uint8_t AVERAGE_NUM = 8;  //number of lastest elements to get average of , AVERAGE_NUM <= N
float circularBuffer[N] = {};
uint8_t count = 0; //number of elements that circularBuffer currently has
uint8_t head = 0;  //latest position to write to
uint8_t tail = 0;  //oldest position to read from
double NUMERATOR = 0;
double DENOMINATOR = 0;
int RESYNC_COUNTER = 0;
#define RESYNC_COUNTER_CYCLE 1000

void push(float value)
{
    if(RESYNC_COUNTER >= RESYNC_COUNTER_CYCLE)
    {
        resync();
        RESYNC_COUNTER = 0;
    }
    float outgoing_value = 0;
    NUMERATOR += value;
    ++RESYNC_COUNTER;
    if(DENOMINATOR >= AVERAGE_NUM)
    {
        outgoing_value = circularBuffer[(head-1-(AVERAGE_NUM-1)+N)%N];
    }
    else
    {
        ++DENOMINATOR;
    }
    NUMERATOR -= outgoing_value;
    circularBuffer[head] = value;
    head = (head+1)%N;
    if(count < N)
    {
        ++count;
    }
    else
    {
        tail = (tail+1)%N;
    }
}

float pop()
{
    if(count==0)
    {
        return 0;
    }
    float res = circularBuffer[tail];
    tail = (tail+1)%N;
    --count;
    return res;
}


float get_average()
{
    if(count == 0) return 0;
    // uint8_t start = 0;
    // float numerator = 0;
    // unsigned int num = AVERAGE_NUM;
    // if(count < AVERAGE_NUM)
    // {
    //     num = count;
    // }
    // start = (head-1-(num-1)+N)%N;
    // while(num > 0)
    // {
    //     numerator += circularBuffer[start];
    //     start = (start+1)%N;
    //     --num;
    // }
    return NUMERATOR/DENOMINATOR;
}

void resync() //to re-calculate the NUMERATOR again to get rid of accumulation of floating point error
{
    double new_NUMERATOR = 0;
    uint8_t start = (head-1-(AVERAGE_NUM-1)+N)%N;
    for(uint8_t i=0; i<AVERAGE_NUM; ++i)
    {
      new_NUMERATOR += circularBuffer[start];
      start = (start+1)%N;
    }
    NUMERATOR = new_NUMERATOR;
    return;
}
//--------------------------------end------------------------------------------------