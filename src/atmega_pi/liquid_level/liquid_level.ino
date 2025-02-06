int liquid_level = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(2,INPUT);

}

void loop() {
  // put your main code here, to run repeatedly:
  liquid_level = digitalRead(2);
  
  // Print the liquid level status
  if (liquid_level == HIGH) 
  {
    Serial.println("Liquid detected!");
  } 
  else 
  {
    Serial.println("No liquid detected.");
  }
  delay(1000);

}
