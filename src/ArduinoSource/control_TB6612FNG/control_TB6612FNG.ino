// Motor A (Left)
int enA = 9;
int ain1 = 8; 
int ain2 = 7;

// Motor B (Right)
int enB = 3;
int bin1 = 5; 
int bin2 = 4;

void setup() {
  pinMode(enA, OUTPUT);
  pinMode(ain1, OUTPUT);
  pinMode(ain2, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(bin1, OUTPUT);
  pinMode(bin2, OUTPUT);

  // Use a lower speed (100) for the first desktop test
  int testSpeed = 100; 

  // Motor A Forward
  digitalWrite(ain1, LOW);
  digitalWrite(ain2, HIGH);
  analogWrite(enA, testSpeed); 

  // Motor B Forward
  digitalWrite(bin1, LOW);
  digitalWrite(bin2, HIGH);
  analogWrite(enB, testSpeed);
}

void loop() {
  // Keeping them spinning
}