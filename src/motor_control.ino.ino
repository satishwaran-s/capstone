// Motor A Pins
const int motorAIn1 = 43;  
const int motorAIn2 = 45;  
const int motorAPWM = 9;  

// Motor B Pins
const int motorBIn3 = 47;  
const int motorBIn4 = 49;  
const int motorBPWM = 10;  

// Encoder Pins
const int encoderA_A = 18;  // Motor A Encoder A Phase
const int encoderA_B = 19;  // Motor A Encoder B Phase
const int encoderB_A = 20;   // Motor B Encoder A Phase 
const int encoderB_B = 21;   // Motor B Encoder B Phase

// Variables for encoder tracking
volatile int encoderCountA = 0;  
volatile int encoderCountB = 0;  
int motorSpeed = 150;  // Motor speed (0-255 for PWM)

// Interrupt Service Routines for encoders
void encoderISR_A_A() {
  if (digitalRead(encoderA_A) == digitalRead(encoderA_B)) {
    encoderCountA++;
  } else {
    encoderCountA--;
  }
}

void encoderISR_A_B() {
  if (digitalRead(encoderA_A) == digitalRead(encoderA_B)) {
    encoderCountA--;
  } else {
    encoderCountA++;
  }
}

void encoderISR_B_A() {
  if (digitalRead(encoderB_A) == digitalRead(encoderB_B)) {
    encoderCountB++;
  } else {
    encoderCountB--;
  }
}

void encoderISR_B_B() {
  if (digitalRead(encoderB_A) == digitalRead(encoderB_B)) {
    encoderCountB--;
  } else {
    encoderCountB++;
  }
}

void setup() {
  // Motor A setup
  pinMode(motorAIn1, OUTPUT);
  pinMode(motorAIn2, OUTPUT);
  pinMode(motorAPWM, OUTPUT);

  // Motor B setup
  pinMode(motorBIn3, OUTPUT);
  pinMode(motorBIn4, OUTPUT);
  pinMode(motorBPWM, OUTPUT);

  // Encoder setup
  pinMode(encoderA_A, INPUT_PULLUP);
  pinMode(encoderA_B, INPUT_PULLUP);
  pinMode(encoderB_A, INPUT_PULLUP);
  pinMode(encoderB_B, INPUT_PULLUP);

  // Attach interrupts for encoders
  attachInterrupt(digitalPinToInterrupt(encoderA_A), encoderISR_A_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderA_B), encoderISR_A_B, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderB_A), encoderISR_B_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderB_B), encoderISR_B_B, CHANGE);

  Serial.begin(115200);  // For monitoring the encoder count
}

void loop() {
  // Set motor speed
  analogWrite(motorAPWM, motorSpeed);
  analogWrite(motorBPWM, motorSpeed);

  // Move reverse (for testing or as needed)
  digitalWrite(motorAIn1, HIGH);   // Reverse for Motor A
  digitalWrite(motorAIn2, LOW);
  digitalWrite(motorBIn3, HIGH);   // Reverse for Motor B
  digitalWrite(motorBIn4, LOW);

  // Print encoder counts (for reverse motion)
  Serial.print("Motor A Encoder Count: ");
  Serial.print(encoderCountA);
  Serial.print(" | Motor B Encoder Count: ");
  Serial.println(encoderCountB);

  // Delay for reverse motion
  delay(5000);

  // Move forward
  digitalWrite(motorAIn1, LOW);    // Forward for Motor A
  digitalWrite(motorAIn2, HIGH);
  digitalWrite(motorBIn3, LOW);    // Forward for Motor B
  digitalWrite(motorBIn4, HIGH);

  // Print encoder counts (for forward motion)
  Serial.print("Motor A Encoder Count: ");
  Serial.print(encoderCountA);
  Serial.print(" | Motor B Encoder Count: ");
  Serial.println(encoderCountB);

  // Delay for forward motion
  delay(5000);

  // Stop after reaching target
  if (encoderCountA >= 1000 || encoderCountB >= 1000) {
    digitalWrite(motorAIn1, LOW);
    digitalWrite(motorAIn2, LOW);
    digitalWrite(motorBIn3, LOW);
    digitalWrite(motorBIn4, LOW);
    analogWrite(motorAPWM, 0);
    analogWrite(motorBPWM, 0);
    Serial.println("Target reached, stopping motors.");
    while (true);  // Stop further execution
  }

// TO STOP MOTORS RUN THIS COMMAND!!!!
//analogWrite(motorAPWM, 0);    // Stop Motor A
//  analogWrite(motorBPWM, 0);    // Stop Motor B
}
