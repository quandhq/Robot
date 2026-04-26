// #include <MPU6050_tockn.h>
#include <Wire.h>


//---------read MPU data------------------------
#define MPU6050_ADDR 0x68 //I2C address of the MPU6050
#define ACCEL_XOUT_H 0x3B
#define PWR_MGMT_1 0x6B
#define ACCEL_CONFIG 0x1C //register for configurating force range
#define PLUS_MINUS_2G 0x00 //value to configurate register ACCEL_CONFIG
#define FORCE_RESOLUTION 32768
#define _180_DIVIDED_BY_PI 57.2958
#define GYRO_X_REGISTER 0x43
#define GYRO_CONFIG 0x1B    //configurate gyro range
#define GYRO_RANGE_PLUS_MINUS_250 0x00
#define GYRO_RESOLUTION 32768
// --- DATA STRUCTURES ---
typedef struct {
    float x;
    float y;
    float z;
} AccelData;

typedef struct {
    float x;
    float y;
    float z;
    float gyro_rate;
} MpuData;

MpuData mpu_data{0, 0, 0, 0};

int FORCE_RANGE = 2;


// --- TB6612FNG Hardware Pins ---
const int enA = 9;   // PWMA
const int ain1 = 8;  // ain1
const int ain2 = 7;  // ain2
const int enB = 3;   // PWMB
const int bin1 = 5;  // bin1
const int bin2 = 4;  // bin2
// Note: STBY is hardwired to 5V

// MPU6050 mpu6050(Wire);

// --- PID Parameters (Adjusted for JGB37-520 Torque) ---
float targetAngle = 0; 
float Kp = 50.0;      // Slightly lower to reduce "shaking"//50 IS FINE
float Ki = 200.0;     // stop the "drifting"  //200 is fine 
float Kd = 4.0;       // Lowered to match the lower Kp
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
float smoothedChangingTargetOffset = 0; // To slowly change to external target angle

// --------------DEBUG COMMANDING--------------------------
long check_time = 0;
float command[3] = {-1, 0, 1};
uint8_t index = 0;
void debug_commanding()
{
  int interval = (index == 2) ? 2000 : 2000; 
  if(millis() - check_time >= interval)
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



//>>>>>>>>>>>>>>>>>>>>MPU6050 hardware communication
float GYRO_X_OFFSET = 0;

void update_mpu()
{
  //Tell sensor that we want to read starting from x-axis high byte
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(ACCEL_XOUT_H); //Start at the first registe
  Wire.endTransmission(false);

  // Request 14 bytes: 6 for Accel, 2 for Temp, 6 for Gyro
  Wire.requestFrom(MPU6050_ADDR, 14, true);
  if(Wire.available() == 14)
  {
    // Bit-shift high and low bytes into 16-bit integers
    float scale_factor = (float)FORCE_RANGE/FORCE_RESOLUTION;
    int16_t raw_x = (Wire.read() << 8) | Wire.read();
    int16_t raw_y = (Wire.read() << 8) | Wire.read();
    int16_t raw_z = (Wire.read() << 8) | Wire.read(); 

    mpu_data.x = (float)raw_x * scale_factor;
    mpu_data.y = (float)raw_y * scale_factor;
    mpu_data.z = (float)raw_z * scale_factor;

    // Temp (skip 2 bytes)
    Wire.read(); 
    Wire.read();
    //read gyro x
    int16_t raw_gyro_x = 0;
    raw_gyro_x = Wire.read() << 8 | Wire.read();
    mpu_data.gyro_rate = (raw_gyro_x - GYRO_X_OFFSET) / 131.0; //(float)250 / (float)GYRO_RESOLUTION;
    //do not use gyro y,z............
    Wire.read(); Wire.read(); // Gyro Y
    Wire.read(); Wire.read(); // Gyro Z

  }
}

AccelData read_mpu6050_accel()
{
  AccelData data = {0,0,0};
  //Tell sensor that we want to read starting from x-axis high byte
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(ACCEL_XOUT_H);
  Wire.endTransmission(false);

  //Request 6 bytes (X, Y, Z are 2bytes each);
  Wire.requestFrom(MPU6050_ADDR, 6, true);
  if(Wire.available() == 6)
  {
    // Bit-shift high and low bytes into 16-bit integers
    float scale_factor = (float)FORCE_RANGE/FORCE_RESOLUTION;
    int16_t raw_x = (Wire.read() << 8) | Wire.read();
    int16_t raw_y = (Wire.read() << 8) | Wire.read();
    int16_t raw_z = (Wire.read() << 8) | Wire.read(); 

    data.x = (float)raw_x * scale_factor;
    data.y = (float)raw_y * scale_factor;
    data.z = (float)raw_z * scale_factor;
  }
  
  return data;
}

float get_raw_pitch_from_acceleromete(MpuData data)
{
  return atan2(data.y, data.z) * _180_DIVIDED_BY_PI;
}


float read_gyro_x_raw()
{
  Wire.beginTransmission(MPU6050_ADDR); // MPU6050 address
  Wire.write(GYRO_X_REGISTER);             // Starting register for Gyro X
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR, 2, true); // Request 2 bytes
  int16_t raw_gyro_x = 0;
  if(Wire.available() == 2)
  {
    raw_gyro_x = Wire.read() << 8 | Wire.read();
  }
  return raw_gyro_x;
}


void setup_mpu6050()
{
  //>>>>>>>>>>>Wake up the MPU6-50
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(PWR_MGMT_1); //PWR_MGMT_1 register
  Wire.write(0);
  Wire.endTransmission(true);
  //<<<<<<<<<<<<<<<<<<<<<<<<<<

  //>>>>>>>>>>>>>>>>configurate the force range -2g -> +2g
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(ACCEL_CONFIG);
  switch(FORCE_RANGE)
  {
    case 2:
      Wire.write(PLUS_MINUS_2G);
      break;
    default:
      Wire.write(PLUS_MINUS_2G);
      break;
  }
  Wire.endTransmission(true);
  //<<<<<<<<<<<<<<<<<<<<

  //Configure the Gyro Range
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(GYRO_CONFIG); // Gyro Config register
  Wire.write(GYRO_RANGE_PLUS_MINUS_250); // Set to 0 (Selects +/- 250 deg/s)
  Wire.endTransmission();
}
//<<<<<<<<<<<<<<<<<<<<end:MPU6050 hardware communication

//>>>>>>>>>>>>>>>>>>>>MATH and FILTER for MPU6050 data
float get_accel_norm(float x, float y, float z)
{
  return sqrt(x*x+y*y+z*z);
}

bool is_data_clean(float norm)
{
  //1g is ideal, we allow for 5% error
  return 0.95 < norm && norm < 1.05;
}
//<<<<<<<<<<<<<<<<end:MATH and FILTER for MPU6050 data

float Angle_fused = 0;

void setup() {
  Serial.begin(115200); // Increased speed to reduce CPU "blocking" time
  Wire.begin();
  // mpu6050.begin(); //Initialize the mpu with library code first so that the custom initialization will not be cleared out
  setup_mpu6050();  //Configurate MPU6050 registers
  Serial.println("MPU6050 Initialized. Starting Norm Filter Test...");
  delay(1000);

  
  pinMode(ledPin, OUTPUT);
  pinMode(enA, OUTPUT); pinMode(ain1, OUTPUT); pinMode(ain2, OUTPUT);
  pinMode(enB, OUTPUT); pinMode(bin1, OUTPUT); pinMode(bin2, OUTPUT);
  digitalWrite(ledPin, HIGH); 

  // Calibration sequence...
  delay(3000); 
  digitalWrite(ledPin, LOW); 
  // mpu6050.calcGyroOffsets(true);

  //Get average gyro x offet
  Serial.println("Calibrating gyro... Do not move the robot!");
  double gyro_x_sum = 0;
  for(int i=0; i<2000; ++i)
  {
    gyro_x_sum += read_gyro_x_raw();
    delay(1);
  }
  GYRO_X_OFFSET = gyro_x_sum/2000;
  Serial.print("Calibration complete. Offset: ");
  Serial.println(GYRO_X_OFFSET);


  
  // Take average for targetAngle using the accelerometer
  float totalAngle = 0;
  for(int i = 0; i < 100; i++) {
    // mpu6050.update();
    update_mpu();
    float sample = get_raw_pitch_from_acceleromete(mpu_data);
    push(sample);
    totalAngle += sample;
    delay(10);
  }
  targetAngle = totalAngle / 100.0;
  lastAngle = targetAngle;
  Angle_fused = targetAngle;
  lastTime = millis();
  lastTelemetry = millis();
  check_time = millis();

  // Wait for configuration parameters from Rasp
  bool configurationReceived = false;
  while(configurationReceived == false)
  {
    // ONLY send this after calibration is 100% finished, and sent it repeatedly so that rasp can catch the message
    Serial.println("ready!"); //<<<<IMPORTANT, NOT A DEBUGGING
    delay(500);

    if(Serial.available() >= 1)
    {
      // Expected format: "Kp,Ki,Kd,minPower,trim"
      // Example: "50.0,200.0,4.0,10,1.0"
      String configStr = Serial.readStringUntil('\n');
      configStr.trim(); //Cleans up \r or trailing spaces
      int comma1 = configStr.indexOf(',');
      int comma2 = configStr.indexOf(',', comma1+1);
      int comma3 = configStr.indexOf(',', comma2+1);
      int comma4 = configStr.indexOf(',', comma3+1);
      if(comma1 > 0 && comma2 > 0 && comma3 > 0 && comma4 > 0)
      {
        Kp = configStr.substring(0, comma1).toFloat();
        Ki = configStr.substring(comma1+1, comma2).toFloat();
        Kd = configStr.substring(comma2+1, comma3).toFloat();
        minPower = configStr.substring(comma3+1, comma4).toInt();
        trim = configStr.substring(comma4+1).toFloat();
        Serial.print("Configuration: ");
        Serial.print(Kp); Serial.print('|');
        Serial.print(Ki); Serial.print('|');
        Serial.print(Kd); Serial.print('|');
        Serial.print(minPower); Serial.print('|');
        Serial.println(trim);
        configurationReceived = true;
      }
    }
  }
  // Visual confirmation that setup is done
  digitalWrite(ledPin, HIGH); 
  delay(500);
  digitalWrite(ledPin, LOW);
  delay(500);
  digitalWrite(ledPin, HIGH); 
  delay(500);
  digitalWrite(ledPin, LOW);
  delay(500);
  digitalWrite(ledPin, HIGH); 
}

float MAX_RAMP_SPEED = 0.05;

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

  // mpu6050.update();
  update_mpu();
  float accel_angle = get_raw_pitch_from_acceleromete(mpu_data); //acceleromete
  push(accel_angle);

  // ENSURE DETERMINISTIC TIMING (The 100Hz Heartbeat)
  if (timeChange >= SAMPLE_TIME) {

    //>>>>>>>>>>>>>>>>get the trusted sensor

    //>>>>>>>>>>>>>>>>>TEST NORM
    // Read x,y,z    
    // Apply the math
    float current_norm = get_accel_norm(mpu_data.x, mpu_data.y, mpu_data.z);
    bool trust_sensor = is_data_clean(current_norm);

    // Output the result
    // Serial.print("Norm: ");
    // Serial.print(current_norm);
    // Serial.print("g | Trust: ");
    // Serial.println(trust_sensor ? "YES" : "NO - VIBRATION DETECTED");
    
    // delay(50); // 20Hz loop for easy reading in the Serial Monitor
    //<<<<<<<<<<<<<<<<<<<<<end:TEST NORM

    //>>>>>>calculate the individual angles
    float gyro_rate = mpu_data.gyro_rate;
    float gyro_angle = Angle_fused + gyro_rate*SAMPLE_TIME/1000;      //gyroscope, dt = 10ms
    if(trust_sensor == true)
    {
      Angle_fused = 0.98*gyro_angle + 0.02*accel_angle;
    }
    else
    {
      Angle_fused = gyro_angle;
    }
    //<<<<<<end:calculate the individual angles



    //<<<<<<<<<<<<<<<<end:get the trusted sensor


    if(externalTargetOffset == 0) //if command is 0, reset the smoothedChangingTargetOffset to 0 to force robot to balance immediately
    {
      smoothedChangingTargetOffset = externalTargetOffset;
    }

    //---slowly change to externalTargetOffset---
    float diff = externalTargetOffset - smoothedChangingTargetOffset;
    diff = (diff < -MAX_RAMP_SPEED ? -MAX_RAMP_SPEED : (diff > MAX_RAMP_SPEED ? MAX_RAMP_SPEED : diff));
    smoothedChangingTargetOffset += diff;
    // mpu6050.update();
    // float sample = mpu6050.getAngleX();
    // push(sample);
    currentAngle = Angle_fused;//get_average();
    // DEBUG: Print this to see if the numbers are actually changing
    // Serial.print("target: ");
    // Serial.print(targetAngle);
    // Serial.print(" - ");
    // Serial.print("current: ");
    // Serial.print(currentAngle);
    // Serial.print(" - ");
    float error = currentAngle - (targetAngle + trim + smoothedChangingTargetOffset);
    // Serial.print("Error: ");
    // Serial.print(error);
    // Serial.println();

    //---if the command is 0, raise the Kp and MAX_RAMP_SPEED to boost the torque to bring the robot back to balance 

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

    lastTime += SAMPLE_TIME;
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