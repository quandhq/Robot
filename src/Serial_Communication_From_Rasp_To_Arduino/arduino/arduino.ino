void setup() {
  Serial.begin(9600);
}

void loop() {
  if(Serial.available())
  {
    String data_received = Serial.readStringUntil('\n');
    if(data_received[data_received.length()-2] == '0')
    {
      Serial.println("Begin!");
    }
    else
    {
      Serial.print("You sent me: ");
      Serial.println(data_received); 
    }
  }
}
