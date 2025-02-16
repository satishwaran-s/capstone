int liquid_level = 0;

const int in1 = 2;
const int in2 = 3;
const int pwm_pin = 4;
const int liquid_sensor = 5;

void setup() {
  Serial.begin(115200);
  pinMode(liquid_sensor, INPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(pwm_pin, OUTPUT);
}

void loop() {
  
liquid_level = digitalRead(liquid_sensor);
  
  // Print the liquid level status
  if (liquid_level == LOW) 
  {
    Serial.println("Liquid detected!");
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    analogWrite(pwm_pin, 255);
  } 
  else 
  {
    Serial.println("No liquid detected.");
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    analogWrite(pwm_pin, 0);
  }
  delay(1000);
   
}
