// Motor A connections
int enA = 9;
int in1 = 8;
int in2 = 7;
// Motor B connections
int enB = 3;
int in3 = 5;
int in4 = 4;

void setup() {
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  // Updated for Forward Movement
  digitalWrite(in1, LOW);   // Swapped
  digitalWrite(in2, HIGH);  // Swapped
  analogWrite(enA, 200); 

  digitalWrite(in3, LOW);   // Swapped
  digitalWrite(in4, HIGH);  // Swapped
  analogWrite(enB, 200);
}

void loop() {
  // Just keeping them spinning for the test
}