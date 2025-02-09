#define liquid_detection_pin 2 

void setup() {
  // put your setup code here, to run once:
  Serial.begin(112500);
  pinMode(liquid_detection_pin, INPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  // read the sensor value from the pin
  int sensorValue = digitalRead(liquid_detection_pin);
  
  // print the sensor value for debugging
  Serial.print("Pin 2 value: ");
  Serial.println(sensorValue);

  // check the sensor value and print appropriate message
  if (sensorValue == 0) 
  {  // if 0, liquid is detected (active low)
    Serial.println("Liquid Detected!");
  }
  else 
  {  // if HIGH, no liquid detected
    Serial.println("No Liquid!");
    while (true)
    {
      // left blank to exit the program cos once it detect should stop
    }
  }

  // delay for stability
  delay(500);  
}
