//pins
int motor1pin1 = 2;
int motor1pin2 = 3;
int motor1speed = 6;

void setup()
{
  //Serial.begin(115200);
  pinMode(motor1pin1, OUTPUT);
  pinMode(motor1pin2, OUTPUT);
  pinMode(motor1speed, OUTPUT);
}

void loop()
{
  analogWrite(motor1speed, 200);

  digitalWrite(motor1pin1, LOW);
  digitalWrite(motor1pin2, LOW);
  delay(3000);


  digitalWrite(motor1pin1, HIGH);
  digitalWrite(motor1pin2, LOW);
  delay(3000);
}
