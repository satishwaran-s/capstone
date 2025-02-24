// Motor A Pins
const int motorAIn1 = 43;  
const int motorAIn2 = 45;  
const int motorAPWM = 9;  

// Motor B Pins
const int motorBIn3 = 47;  
const int motorBIn4 = 49;  
const int motorBPWM = 10;  

// Encoder Pins
const int encoderA_A = 18;  
const int encoderA_B = 19;  
const int encoderB_A = 20;  
const int encoderB_B = 21;  

// Encoder Variables
volatile int encoderCountA = 0;  
volatile int encoderCountB = 0;  
int motorSpeed = 150;
const int speedStep = 25;

// Timer Variables for Speed Calculation
unsigned long prevMillis = 0;
const int interval = 100; // Time interval for speed calculation (in ms)
float speedA = 0, speedB = 0; // RPM
float kp = 1.5;  // Proportional gain (tune this value)

// Interrupt Service Routines (ISR) for Encoders
void encoderISR_A_A() { encoderCountA += (digitalRead(encoderA_A) == digitalRead(encoderA_B)) ? 1 : -1; }
void encoderISR_B_A() { encoderCountB += (digitalRead(encoderB_A) == digitalRead(encoderB_B)) ? 1 : -1; }

void setup() {
    pinMode(motorAIn1, OUTPUT);
    pinMode(motorAIn2, OUTPUT);
    pinMode(motorAPWM, OUTPUT);
    pinMode(motorBIn3, OUTPUT);
    pinMode(motorBIn4, OUTPUT);
    pinMode(motorBPWM, OUTPUT);

    pinMode(encoderA_A, INPUT_PULLUP);
    pinMode(encoderB_A, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(encoderA_A), encoderISR_A_A, CHANGE);
    attachInterrupt(digitalPinToInterrupt(encoderB_A), encoderISR_B_A, CHANGE);

    Serial.begin(115200);
}

void loop() {
    if (millis() - prevMillis >= interval) {
        prevMillis = millis();
        calculateSpeed();
        adjustMotorSpeed();
    }

    if (Serial.available() > 0) {
        char command = Serial.read();
        switch (command) {
            case 'F': moveForward(); break;
            case 'B': moveBackward(); break;
            case 'L': turnLeft(); break;
            case 'R': turnRight(); break;
            case 'S': stopMotors(); break;
            case '+': increaseSpeed(); break;
            case '-': decreaseSpeed(); break;
        }
    }
}

// Function to Calculate Motor Speed (RPM)
void calculateSpeed() {
    speedA = (encoderCountA / 360.0) * 60.0 / (interval / 1000.0);  // Assuming 360 CPR
    speedB = (encoderCountB / 360.0) * 60.0 / (interval / 1000.0);
    encoderCountA = 0;
    encoderCountB = 0;
}

// Function to Adjust Motor Speed Using Proportional Control
void adjustMotorSpeed() {
    int error = speedA - speedB;
    int correction = kp * error; 

    int motorASpeed = constrain(motorSpeed - correction, 0, 255);
    int motorBSpeed = constrain(motorSpeed + correction, 0, 255);

    analogWrite(motorAPWM, motorASpeed);
    analogWrite(motorBPWM, motorBSpeed);
}

// Motion Functions
void moveForward() {
    digitalWrite(motorAIn1, LOW);
    digitalWrite(motorAIn2, HIGH);
    digitalWrite(motorBIn3, LOW);
    digitalWrite(motorBIn4, HIGH);
    Serial.println("Moving Forward");
}

void moveBackward() {
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

// Increase speed function
void increaseSpeed() {
    if (motorSpeed + speedStep <= 255) {
        motorSpeed += speedStep;
        Serial.print("Speed Increased: ");
        Serial.println(motorSpeed);
    } else {
        motorSpeed = 255;
        Serial.println("Speed at Maximum (255)");
    }
}

// Decrease speed function
void decreaseSpeed() {
    if (motorSpeed - speedStep >= 0) {
        motorSpeed -= speedStep;
        Serial.print("Speed Decreased: ");
        Serial.println(motorSpeed);
    } else {
        motorSpeed = 0;
        Serial.println("Speed at Minimum (0)");
    }
}
