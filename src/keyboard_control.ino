
// Motor A Pins
const int motorAIn1 = 47;
const int motorAIn2 = 49;
const int motorAPWM = 10;

// Motor B Pins
const int motorBIn3 = 43;
const int motorBIn4 = 45;
const int motorBPWM = 9;

// Encoder Pins
const int encoderA_A = 18;
const int encoderA_B = 19;
const int encoderB_A = 20;
const int encoderB_B = 21;

// Pump Control Pins
const int pumpIn1 = 5;
const int pumpIn2 = 4;
const int pumpPWM = 3;
const int liquid_sensor = 2;

// Encoder Variables
volatile int encoderCountA = 0;
volatile int encoderCountB = 0;
int motorSpeed = 150;
const int speedStep = 25;
bool pumpActive = false;
bool isTurning = false;


// Timer Variables for Speed Calculation
unsigned long prevMillis = 0;
const int interval = 100;
float speedA = 0, speedB = 0;
float kp = 1.5;

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
    
    pinMode(pumpIn1, OUTPUT);
    pinMode(pumpIn2, OUTPUT);
    pinMode(pumpPWM, OUTPUT);
    pinMode(liquid_sensor, INPUT);


    
    Serial.begin(115200);
}

void loop() {
    if (millis() - prevMillis >= interval && !isTurning) {
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
            case 'P': startPump(); break;
            case 'O': stopPump(); break;
            default:
              Serial.print("unknown command");
              Serial.println(command);
              break;
        }
    }
    
    if (pumpActive) {
        int liquid_level = digitalRead(liquid_sensor);
        if (liquid_level == LOW) {
            Serial.println("Liquid detected! Pumping...");
            digitalWrite(pumpIn1, HIGH);
            digitalWrite(pumpIn2, LOW);
            analogWrite(pumpPWM, 255);
        } else {
            Serial.println("No liquid detected. Stopping pump.");
            stopPump();
        }
    }
}

void calculateSpeed() {
    speedA = (encoderCountA / 360.0) * 60.0 / (interval / 1000.0);
    speedB = (encoderCountB / 360.0) * 60.0 / (interval / 1000.0);
    encoderCountA = 0;
    encoderCountB = 0;
}

void adjustMotorSpeed() {
    int error = speedA - speedB;
    int correction = kp * error;
    int motorASpeed = constrain(motorSpeed - correction, 0, 255);
    int motorBSpeed = constrain(motorSpeed + correction, 0, 255);
    analogWrite(motorAPWM, motorASpeed);
    analogWrite(motorBPWM, motorBSpeed);
}

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


void turnRight() {
    isTurning = true;
    analogWrite(motorAPWM, motorSpeed);
    analogWrite(motorBPWM, motorSpeed);
    digitalWrite(motorAIn1, LOW);
    digitalWrite(motorAIn2, HIGH);
    digitalWrite(motorBIn3, HIGH);
    digitalWrite(motorBIn4, LOW);
    Serial.println("Turning Left");
}

void turnLeft() {
    isTurning = true;
    analogWrite(motorAPWM, motorSpeed);
    analogWrite(motorBPWM, motorSpeed);
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

void increaseSpeed() {
    motorSpeed = min(motorSpeed + speedStep, 255);
    Serial.print("Speed Increased: ");
    Serial.println(motorSpeed);
}

void decreaseSpeed() {
    motorSpeed = max(motorSpeed - speedStep, 0);
    Serial.print("Speed Decreased: ");
    Serial.println(motorSpeed);
}

void startPump() {
    pumpActive = true;
    Serial.println("Pump Activated");
}

void stopPump() {
    pumpActive = false;
    digitalWrite(pumpIn1, LOW);
    digitalWrite(pumpIn2, LOW);
    analogWrite(pumpPWM, 0);
    Serial.println("Pump Stopped");
}
