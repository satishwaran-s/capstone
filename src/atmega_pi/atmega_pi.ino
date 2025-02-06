int counter = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200); 
  while (!Serial)
  {}
}

void loop() {
  // to send to rpi
//  Serial.println("hello from the arduino");
//  delay(1000); // delay of 1 second

// to receive data from rpi
  if(Serial.available() > 0)
  {
    String message = Serial.readStringUntil('\n');
    message = message + " " + String(counter);
    counter++;
    Serial.println(message);
  }
  
}
