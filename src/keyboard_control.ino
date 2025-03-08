// motor A pins
const int motorAIn1 = 47;
const int motorAIn2 = 49;
const int motorAPWM = 10;

// motor B pins
const int motorBIn3 = 43;
const int motorBIn4 = 45;
const int motorBPWM = 9;

// pump control pins
const int pumpIn1 = 5;
const int pumpIn2 = 4;
const int pumpPWM = 3;
const int liquid_sensor = 2;

// encoder pins
const int encoderARightPin = 18; // yellow
const int encoderBRightPin = 19; // green
const int encoderALeftPin = 20; // yellow
const int encoderBLeftPin = 21; // green

// encoder counts
volatile int leftEncoderCount = 0;
volatile int rightEncoderCount = 0;

unsigned long lastTime = 0;
unsigned long debounceDelay = 15;  // debounce time in ms

// motor and wheel specs
const float wheelDiameter = 0.065;  // 65mm = 0.065m
const int ppr = 11;  // pulses per revolution 
const int gear_ratio = 150;  // 150:1 gear reduction

const int scale_factor = 18;

// calculated constants
const float wheelCircumference = 3.1416 * wheelDiameter;
const float counts_per_wheel_rev = ppr * 4 * gear_ratio;  // quadrature encoding cos of AB dual phase
const float distance_per_count = wheelCircumference / counts_per_wheel_rev;

int baseSpeed = 255;  // base speed for motors

void setup() {
    Serial.begin(115200);

    pinMode(encoderALeftPin, INPUT_PULLUP);  
    pinMode(encoderBLeftPin, INPUT_PULLUP);
    pinMode(encoderARightPin, INPUT_PULLUP);
    pinMode(encoderBRightPin, INPUT_PULLUP);

    // motors and pump pins setup
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

    // enoder pins setup
    pinMode(encoderALeftPin, INPUT);
    pinMode(encoderBLeftPin, INPUT);
    pinMode(encoderARightPin, INPUT);
    pinMode(encoderBRightPin, INPUT);

    // attach interrupts for encoder readings
    attachInterrupt(digitalPinToInterrupt(encoderALeftPin), leftEncoder, RISING);
    attachInterrupt(digitalPinToInterrupt(encoderARightPin), rightEncoder, RISING);
}

void loop() {
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

     float leftDistance = leftEncoderCount * (distance_per_count * scale_factor);
     float rightDistance = rightEncoderCount * (distance_per_count * scale_factor);
    // send the distances in the format expected by ros2 python script
    Serial.print("L:");
    Serial.print(leftDistance, 4);  // send left distance with 4 decimal places
    Serial.print(" R:");
    Serial.println(rightDistance, 4); // send right distance with 4 decimal places
  
    delay(100); // send data every 100 ms cos we need 10 hz for /odom
}

// motor control functions
void moveForward() {
    setMotorSpeeds(175, baseSpeed);
    digitalWrite(motorAIn1, LOW);
    digitalWrite(motorAIn2, HIGH);
    digitalWrite(motorBIn3, LOW);
    digitalWrite(motorBIn4, HIGH);
}

void moveBackward() {
    setMotorSpeeds(baseSpeed, baseSpeed);
    digitalWrite(motorAIn1, HIGH);
    digitalWrite(motorAIn2, LOW);
    digitalWrite(motorBIn3, HIGH);
    digitalWrite(motorBIn4, LOW);
}

void turnRight() {
    setMotorSpeeds(baseSpeed, baseSpeed);
    digitalWrite(motorAIn1, LOW);
    digitalWrite(motorAIn2, HIGH);
    digitalWrite(motorBIn3, HIGH);
    digitalWrite(motorBIn4, LOW);
}

void turnLeft() {
    setMotorSpeeds(baseSpeed, baseSpeed);
    digitalWrite(motorAIn1, HIGH);
    digitalWrite(motorAIn2, LOW);
    digitalWrite(motorBIn3, LOW);
    digitalWrite(motorBIn4, HIGH);
}

void stopMotors() {
    setMotorSpeeds(0, 0);
    digitalWrite(motorAIn1, LOW);
    digitalWrite(motorAIn2, LOW);
    digitalWrite(motorBIn3, LOW);
    digitalWrite(motorBIn4, LOW);
}

// helper function to control motor speeds
void setMotorSpeeds(int speedA, int speedB) {
    analogWrite(motorAPWM, speedA);
    analogWrite(motorBPWM, speedB);
}

// pump control functions
void startPump() {
    digitalWrite(pumpIn1, HIGH);
    digitalWrite(pumpIn2, LOW);
    analogWrite(pumpPWM, 255);  // full speed for the pump
}

void stopPump() {
    digitalWrite(pumpIn1, LOW);
    digitalWrite(pumpIn2, LOW);
    analogWrite(pumpPWM, 0);  // off the pump
}

void leftEncoder() {
    unsigned long currentTime = millis();
    if (currentTime - lastTime > debounceDelay) {
        leftEncoderCount++;
        lastTime = currentTime;
    }
}

void rightEncoder() {
    unsigned long currentTime = millis();
    if (currentTime - lastTime > debounceDelay) {
        rightEncoderCount++;
        lastTime = currentTime;
    }
}
