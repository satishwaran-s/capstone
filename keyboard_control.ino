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
const int encoderB_A = 20;  // Motor B Encoder A Phase 
const int encoderB_B = 21;  // Motor B Encoder B Phase

// Variables for encoder tracking
volatile int encoderCountA = 0;  
volatile int encoderCountB = 0;  
int motorSpeed = 150;  // Default speed (0-255 for PWM)

void encoderISR_A_A() { encoderCountA += (digitalRead(encoderA_A) == digitalRead(encoderA_B)) ? 1 : -1; }
void encoderISR_A_B() { encoderCountA += (digitalRead(encoderA_A) == digitalRead(encoderA_B)) ? -1 : 1; }
void encoderISR_B_A() { encoderCountB += (digitalRead(encoderB_A) == digitalRead(encoderB_B)) ? 1 : -1; }
void encoderISR_B_B() { encoderCountB += (digitalRead(encoderB_A) == digitalRead(encoderB_B)) ? -1 : 1; }

void setup() {
    pinMode(motorAIn1, OUTPUT);
    pinMode(motorAIn2, OUTPUT);
    pinMode(motorAPWM, OUTPUT);
    pinMode(motorBIn3, OUTPUT);
    pinMode(motorBIn4, OUTPUT);
    pinMode(motorBPWM, OUTPUT);

    pinMode(encoderA_A, INPUT_PULLUP);
    pinMode(encoderA_B, INPUT_PULLUP);
    pinMode(encoderB_A, INPUT_PULLUP);
    pinMode(encoderB_B, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(encoderA_A), encoderISR_A_A, CHANGE);
    attachInterrupt(digitalPinToInterrupt(encoderA_B), encoderISR_A_B, CHANGE);
    attachInterrupt(digitalPinToInterrupt(encoderB_A), encoderISR_B_A, CHANGE);
    attachInterrupt(digitalPinToInterrupt(encoderB_B), encoderISR_B_B, CHANGE);

    Serial.begin(115200);  // Serial for ROS2 communication
}

void loop() {
    if (Serial.available() > 0) {
        char command = Serial.read();
        
        switch (command) {
            case 'F': moveForward(); break;
            case 'B': moveBackward(); break;
            case 'L': turnLeft(); break;
            case 'R': turnRight(); break;
            case 'S': stopMotors(); break;
        }
    }
}

// Function Definitions
void moveForward() {
    analogWrite(motorAPWM, motorSpeed);
    analogWrite(motorBPWM, motorSpeed);
    digitalWrite(motorAIn1, LOW);
    digitalWrite(motorAIn2, HIGH);
    digitalWrite(motorBIn3, LOW);
    digitalWrite(motorBIn4, HIGH);
    Serial.println("Moving Forward");
}

void moveBackward() {
    analogWrite(motorAPWM, motorSpeed);
    analogWrite(motorBPWM, motorSpeed);
    digitalWrite(motorAIn1, HIGH);
    digitalWrite(motorAIn2, LOW);
    digitalWrite(motorBIn3, HIGH);
    digitalWrite(motorBIn4, LOW);
    Serial.println("Moving Backward");
}

void turnLeft() {
    analogWrite(motorAPWM, motorSpeed / 2);
    analogWrite(motorBPWM, motorSpeed);
    digitalWrite(motorAIn1, LOW);
    digitalWrite(motorAIn2, HIGH);
    digitalWrite(motorBIn3, HIGH);
    digitalWrite(motorBIn4, LOW);
    Serial.println("Turning Left");
}

void turnRight() {
    analogWrite(motorAPWM, motorSpeed);
    analogWrite(motorBPWM, motorSpeed / 2);
    digitalWrite(motorAIn1, HIGH);
    digitalWrite(motorAIn2, LOW);
    digitalWrite(motorBIn3, LOW);
    digitalWrite(motorBIn4, HIGH);
    Serial.println("Turning Right");
}

void stopMotors() {
    analogWrite(motorAPWM, 0);
    analogWrite(motorBPWM, 0);
    digitalWrite(motorAIn1, LOW);
    digitalWrite(motorAIn2, LOW);
    digitalWrite(motorBIn3, LOW);
    digitalWrite(motorBIn4, LOW);
    Serial.println("Motors Stopped");
}
