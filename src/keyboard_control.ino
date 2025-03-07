// Motor A Pins
const int motorAIn1 = 47;
const int motorAIn2 = 49;
const int motorAPWM = 10;

// Motor B Pins
const int motorBIn3 = 43;
const int motorBIn4 = 45;
const int motorBPWM = 9;

// Pump Control Pins
const int pumpIn1 = 5;
const int pumpIn2 = 4;
const int pumpPWM = 3;
const int liquid_sensor = 2;

// Encoder Pins
const int encoderARightPin = 18;
const int encoderBRightPin = 19;
const int encoderALeftPin = 20;
const int encoderBLeftPin = 21;

// Encoder counts
volatile int leftEncoderCount = 0;
volatile int rightEncoderCount = 0;

// Constants for Wheel and Encoder Parameters
const float wheelDiameter = 0.065;  // meters
const int ppr = 44;  // Pulses per revolution
const float wheelCircumference = 3.14159 * wheelDiameter;  // meters

// Motor and Encoder Control Variables
int baseSpeed = 180;  // base speed for motors

void setup() {
    // Motor and Pump Pins Setup
    pinMode(motorAIn1, OUTPUT);
    pinMode(motorAIn2, OUTPUT);
    pinMode(motorAPWM, OUTPUT);
    pinMode(motorBIn3, OUTPUT);
    pinMode(motorBIn4, OUTPUT);
    pinMode(motorBPWM, OUTPUT);
    pinMode(pumpIn1, OUTPUT);
    pinMode(pumpIn2, OUTPUT);
    pinMode(pumpPWM, OUTPUT);
    pinMode(liquid_sensor, INPUT);

    // Encoder Pins Setup
    pinMode(encoderALeftPin, INPUT);
    pinMode(encoderBLeftPin, INPUT);
    pinMode(encoderARightPin, INPUT);
    pinMode(encoderBRightPin, INPUT);

    // Attach interrupts for encoder readings
    attachInterrupt(digitalPinToInterrupt(encoderALeftPin), leftEncoder, RISING);
    attachInterrupt(digitalPinToInterrupt(encoderARightPin), rightEncoder, RISING);

    // Start Serial Communication
    Serial.begin(115200);
}

void loop() {
    // Check for incoming serial commands
    if (Serial.available() > 0) {
        char command = Serial.read();
        switch (command) {
            case 'F': case 'f': moveForward(); break;
            case 'B': case 'b': moveBackward(); break;
            case 'L': case 'l': turnLeft(); break;
            case 'R': case 'r': turnRight(); break;
            case 'S': case 's': stopMotors(); break;
            case 'P': case 'p': startPump(); break;
            case 'O': case 'o': stopPump(); break;
        }
    }

    // Calculate and print the distance traveled by each wheel
    float leftDistance = ((leftEncoderCount * 0.01989) / ppr) * wheelCircumference;
    float rightDistance = ((rightEncoderCount * 0.019) / ppr) * wheelCircumference;

//    Serial.print("L:");Serial.print(leftEncoderCount);
//    Serial.print(",R:");
//    Serial.println(rightEncoderCount);

//    Serial.print("Left Encoder Count: ");
//    Serial.print(leftEncoderCount);
    Serial.print("L:");
    Serial.print(leftDistance);
//    Serial.print("\tRight Encoder Count: ");
//    Serial.print(rightEncoderCount);
    Serial.print(" R:");
    Serial.println(rightDistance);



    delay(100);  // Update every 100ms
}

// Motor control functions
void moveForward() {
    analogWrite(motorAPWM, baseSpeed);
    analogWrite(motorBPWM, baseSpeed);
    digitalWrite(motorAIn1, LOW);
    digitalWrite(motorAIn2, HIGH);
    digitalWrite(motorBIn3, LOW);
    digitalWrite(motorBIn4, HIGH);
}

void moveBackward() {
    analogWrite(motorAPWM, baseSpeed);
    analogWrite(motorBPWM, baseSpeed);
    digitalWrite(motorAIn1, HIGH);
    digitalWrite(motorAIn2, LOW);
    digitalWrite(motorBIn3, HIGH);
    digitalWrite(motorBIn4, LOW);
}

void turnRight() {
    analogWrite(motorAPWM, baseSpeed);
    analogWrite(motorBPWM, baseSpeed);
    digitalWrite(motorAIn1, LOW);
    digitalWrite(motorAIn2, HIGH);
    digitalWrite(motorBIn3, HIGH);
    digitalWrite(motorBIn4, LOW);
}

void turnLeft() {
    analogWrite(motorAPWM, baseSpeed);
    analogWrite(motorBPWM, baseSpeed);
    digitalWrite(motorAIn1, HIGH);
    digitalWrite(motorAIn2, LOW);
    digitalWrite(motorBIn3, LOW);
    digitalWrite(motorBIn4, HIGH);
}

void stopMotors() {
    analogWrite(motorAPWM, 0);
    analogWrite(motorBPWM, 0);
    digitalWrite(motorAIn1, LOW);
    digitalWrite(motorAIn2, LOW);
    digitalWrite(motorBIn3, LOW);
    digitalWrite(motorBIn4, LOW);
}

// Pump control functions
void startPump() {
    digitalWrite(pumpIn1, HIGH);
    digitalWrite(pumpIn2, LOW);
    analogWrite(pumpPWM, 255);  // Full speed for the pump
}

void stopPump() {
    digitalWrite(pumpIn1, LOW);
    digitalWrite(pumpIn2, LOW);
    analogWrite(pumpPWM, 0);  // Stop the pump
}

// Encoder interrupt handlers
void leftEncoder() {
    leftEncoderCount++;
}

void rightEncoder() {
    rightEncoderCount++;
}
