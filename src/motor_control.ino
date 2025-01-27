// Motor Pins
const int motorIn1 = 43;  // IN1 on L298N
const int motorIn2 = 45;  // IN2 on L298N

// Encoder Pins
const int encoderPinA = 53;  // Yellow wire (Encoder A Phase)
const int encoderPinB = 51;  // Green wire (Encoder B Phase)

// Variables for encoder tracking
volatile int encoderCount = 0;  // Tracks the encoder pulses
int motorSpeed = 150;           // Motor speed (0-255 for PWM)

// Interrupt Service Routines for encoder
void encoderISR_A() {
  if (digitalRead(encoderPinA) == digitalRead(encoderPinB)) {
    encoderCount++;
  } else {
    encoderCount--;
  }
}

void encoderISR_B() {
  if (digitalRead(encoderPinA) == digitalRead(encoderPinB)) {
    encoderCount--;
  } else {
    encoderCount++;
  }
}

void setup() {
  // Motor pins setup
  pinMode(motorIn1, OUTPUT);
  pinMode(motorIn2, OUTPUT);

  // Encoder pins setup
  pinMode(encoderPinA, INPUT_PULLUP);
  pinMode(encoderPinB, INPUT_PULLUP);

  // Attach interrupts for encoder pins
  attachInterrupt(digitalPinToInterrupt(encoderPinA), encoderISR_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPinB), encoderISR_B, CHANGE);

  Serial.begin(9600);  // For monitoring the encoder count
}

void loop() {
  // Example: Forward direction
  digitalWrite(motorIn1, HIGH);
  digitalWrite(motorIn2, LOW);

  // Print encoder count to Serial Monitor
  Serial.print("Encoder Count: ");
  Serial.println(encoderCount);

  // Delay to avoid flooding the Serial Monitor
  delay(500);

  // Example: Stop motor after some rotations
  if (encoderCount >= 1000) {
    digitalWrite(motorIn1, LOW);
    digitalWrite(motorIn2, LOW);
    Serial.println("Target reached, stopping motor.");
    while (true); // Stop further execution
  }
}
