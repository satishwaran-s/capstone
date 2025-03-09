

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
const int encoderALeftPin = 21; // yellow
const int encoderBLeftPin = 20; // green

// encoder counts
volatile int leftEncoderCount = 0;
volatile int rightEncoderCount = 0;

unsigned long lastTime = 0;
// Global variables (corrected)
//unsigned long lastTimeLeft = 0;
//unsigned long lastTimeRight = 0;
const unsigned long debounceDelay = 15;  // debounce time in ms

// motor and wheel specs
const float wheelDiameter = 0.065;  // 65mm = 0.065m
const int ppr = 11;  // pulses per revolution 
const int gear_ratio = 150;  // 150:1 gear reduction

const float wheelbase = 0.33;
const int scale_factor = 100;

// calculated constants
const float wheelCircumference = 3.1416 * wheelDiameter;
const float counts_per_wheel_rev = ppr * 4 * gear_ratio;  // quadrature encoding cos of AB dual phase
const float distance_per_count = wheelCircumference / counts_per_wheel_rev;

int baseSpeed = 255;  // base speed for motors

// PID control variables
float Kp = 2.0, Ki = 0.1, Kd = 0.05;
float errorPrev = 0, errorIntegral = 0;
unsigned long lastPidTime = 0;

bool isRotating = false;  // Variable to determine whether the robot is rotating or moving straight

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
        handleSerialCommand();
    }

    // Calculate odometry
    float leftDist = leftEncoderCount * (distance_per_count * scale_factor);
    float rightDist = rightEncoderCount * (distance_per_count * scale_factor);
    
    // Compute linear and angular velocities
    float linear = (isRotating) ? 0 : (leftDist + rightDist) / 2.0;
    float angular = (rightDist - leftDist) / wheelbase;

    // Serial output for debugging and visualization
    Serial.print("D:");
    Serial.print(linear, 4);  // send left distance with 4 decimal places
    Serial.print(" A:");
    Serial.println(angular, 4); // send right distance with 4 decimal places

    // Apply PID control to adjust robot's motion
    updatePid();

    delay(100); // send data every 100 ms to match odometry update frequency
}

void updatePid() {
    unsigned long currentTime = millis();
    float dt = (currentTime - lastPidTime) / 1000.0;
    lastPidTime = currentTime;

    if (dt > 0 && dt < 1.0) {
        // Compute error for both linear and angular
        float leftDist = leftEncoderCount * (distance_per_count * scale_factor);
        float rightDist = rightEncoderCount * (distance_per_count * scale_factor);
        
        float error = leftDist - rightDist;
        float proportional = Kp * error;
        errorIntegral += Ki * error * dt;  // Accumulate error over time
        float derivative = Kd * (error - errorPrev) / dt;

        // Adjust speed for motors based on PID output
        int correction = proportional + errorIntegral + derivative;
        errorPrev = error;

        int leftSpeed = constrain(baseSpeed - correction, 0, 255);
        int rightSpeed = constrain(baseSpeed + correction, 0, 255);

        // Adjust motor speeds based on PID feedback
        setMotorSpeeds(leftSpeed, rightSpeed);
    }
}

void handleSerialCommand() {
    char command = Serial.read();
    switch (command) {
        case 'F': case 'f': 
            isRotating = false;  // Disable rotation mode
            moveForward(); 
            break;
        case 'B': case 'b': 
            isRotating = false;  // Disable rotation mode
            moveBackward(); 
            break;
        case 'L': case 'l': 
            isRotating = true;  // Enable rotation mode
            turnLeft(); 
            break;
        case 'R': case 'r': 
            isRotating = true;  // Enable rotation mode
            turnRight(); 
            break;
        case 'S': case 's': 
            stopMotors(); 
            break;
        case 'P': case 'p': 
            startPump(); 
            break;
        case 'O': case 'o': 
            stopPump(); 
            break;
    }
}

// Motor control functions
void moveForward() {
    setMotorSpeeds(baseSpeed, baseSpeed);
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

// Helper function to control motor speeds
void setMotorSpeeds(int speedA, int speedB) {
    analogWrite(motorAPWM, speedA);
    analogWrite(motorBPWM, speedB);
}

// Pump control functions
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


//// Left encoder ISR
//void leftEncoder() {
//    unsigned long currentTime = millis();
//    if (currentTime - lastTimeLeft > debounceDelay) {
//        leftEncoderCount++;
//        lastTimeLeft = currentTime;
//    }
//}
//
//// Right encoder ISR
//void rightEncoder() {
//    unsigned long currentTime = millis();
//    if (currentTime - lastTimeRight > debounceDelay) {
//        rightEncoderCount++;
//        lastTimeRight = currentTime;
//    }
//}

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
