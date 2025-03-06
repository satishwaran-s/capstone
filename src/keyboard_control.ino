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
const int encoderALeftPin = 18;
const int encoderBLeftPin = 19;
const int encoderARightPin = 20;
const int encoderBRightPin = 21;

// Encoder counts
volatile int leftEncoderCount = 0;
volatile int rightEncoderCount = 0;

// PID Control Variables (optional)
float Kp = 0.8, Ki = 0.02, Kd = 0.5;
float leftError, rightError, leftPrevError = 0, rightPrevError = 0;
float leftIntegral = 0, rightIntegral = 0;
float leftDerivative, rightDerivative;
int baseSpeed = 200;

// Wheel and Encoder Parameters
const float wheelRadius = 0.0325;
const int ticksPerRevolution = 44;
const float wheelCircumference = 2 * 3.14159 * wheelRadius;
const float cmPerTick = wheelCircumference / ticksPerRevolution;
const float wheelBase = 0.3;

// Encoder interrupt handlers
void encoderALeftISR() {
    if (digitalRead(encoderBLeftPin) == HIGH) leftEncoderCount++;
    else leftEncoderCount--;
}

void encoderARightISR() {
    if (digitalRead(encoderBRightPin) == HIGH) rightEncoderCount++;
    else rightEncoderCount--;
}

void setup() {
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
    pinMode(encoderALeftPin, INPUT);
    pinMode(encoderBLeftPin, INPUT);
    pinMode(encoderARightPin, INPUT);
    pinMode(encoderBRightPin, INPUT);
    
    attachInterrupt(digitalPinToInterrupt(encoderALeftPin), encoderALeftISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(encoderARightPin), encoderARightISR, CHANGE);
    
    Serial.begin(115200);
}

void loop() {
    updateOdometry(leftEncoderCount, rightEncoderCount);

    if (Serial.available() > 0) {
        char command = Serial.read();
        switch (command) {
            case 'F': moveForward(); break;
            case 'B': moveBackward(); break;
            case 'L': turnLeft(); break;
            case 'R': turnRight(); break;
            case 'S': stopMotors(); break;
            case 'P': startPump(); break;
            case 'O': stopPump(); break;
        }
    }

    delay(100);
}

void updateOdometry(int leftTicks, int rightTicks) {
    float leftDistance = leftTicks * cmPerTick;
    float rightDistance = rightTicks * cmPerTick;
    float deltaDistance = (leftDistance + rightDistance) / 2;
    float deltaAngle = (rightDistance - leftDistance) / wheelBase;

//    Serial.print("Odometry: ");
//    Serial.print(deltaDistance);
//    Serial.print(", ");
//    Serial.println(deltaAngle);

    Serial.print("Encoder_left: ");
    Serial.print(rightEncoderCount);
    Serial.print(" , Encoder_right: ");
    Serial.println(leftEncoderCount);
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

void startPump() {
    digitalWrite(pumpIn1, HIGH);
    digitalWrite(pumpIn2, LOW);
    analogWrite(pumpPWM, 255);
}

void stopPump() {
    digitalWrite(pumpIn1, LOW);
    digitalWrite(pumpIn2, LOW);
    analogWrite(pumpPWM, 0);
}
