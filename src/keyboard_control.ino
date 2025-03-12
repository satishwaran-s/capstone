// Motor A pins
const int motorAIn4 = 47;
const int motorAIn3 = 49;
const int motorAPWM = 10;

// Motor B pins
const int motorBIn2 = 43;
const int motorBIn1 = 45;
const int motorBPWM = 9;

// Pump control pins
const int pumpIn1 = 5;
const int pumpIn2 = 4;
const int pumpPWM = 3;
const int liquid_sensor = 2;  // Liquid sensor pin

// Encoder pins
const int encoderALeftPin = 21;  // yellow
const int encoderBLeftPin = 20;  // green
const int encoderARightPin = 18; // yellow
const int encoderBRightPin = 19; // green

// Encoder counts (signed for direction)
volatile int leftEncoderCount = 0;
volatile int rightEncoderCount = 0;
float totalLinearDistance = 0.0; // Store total linear distance

// Motor and wheel specs
const float wheelDiameter = 0.065;  // 65mm = 0.065m
const int ppr = 11;                 // Pulses per revolution 
const int gear_ratio = 150;         // 150:1 gear reduction
const float wheelbase = 0.33;       // Distance between wheels in meters
const float scale_factor = 2.0;   // Scaling factor for distance calculation

// State tracking
bool isRotating = false;            // Whether robot is turning
bool isMoving = false;              // Whether motors are active
int motorState = 0;                 // 0=stop, 1=forward, 2=backward, 3=left, 4=right

// Calculated constants
const float wheelCircumference = PI * wheelDiameter;
const float counts_per_wheel_rev = ppr * 4 * gear_ratio;  // Quadrature encoding
const float distance_per_count = wheelCircumference / counts_per_wheel_rev;

int baseSpeed = 180;  // Base motor speed

bool pumpActive = false;  // Track if pump should be active

// PID constants
float Kp = 2.0, Ki = 0.1, Kd = 0.05;
float errorPrev = 0, errorIntegral = 0;
unsigned long lastPidTime = 0;

// Last sent values to avoid duplicate output when stopped
float lastLinear = 0;
float lastAngular = 0;
unsigned long lastUpdateTime = 0;

// Variables for final angular rotation
float totalAngularRotation = 0.0;  // Store final cumulative angular rotation

// Function Prototypes
void handleLeftEncoder();
void handleRightEncoder();
void handleSerialCommand();
void startPump();
void stopPump();
void updatePid();
void resetPid();
void moveForward();
void moveBackward();
void turnLeft();
void turnRight();
void stopMotors();
void setMotorSpeeds(int leftSpeed, int rightSpeed);

void setup() {
    Serial.begin(115200);
    
    // Initialize motor control pins
    pinMode(motorAIn4, OUTPUT);
    pinMode(motorAIn3, OUTPUT);
    pinMode(motorAPWM, OUTPUT);
    pinMode(motorBIn2, OUTPUT);
    pinMode(motorBIn1, OUTPUT);
    pinMode(motorBPWM, OUTPUT);
    
    // Initialize pump control pins
    pinMode(pumpIn1, OUTPUT);
    pinMode(pumpIn2, OUTPUT);
    pinMode(pumpPWM, OUTPUT);
    pinMode(liquid_sensor, INPUT);

    // Initialize encoder pins
    pinMode(encoderALeftPin, INPUT_PULLUP);
    pinMode(encoderBLeftPin, INPUT_PULLUP);
    pinMode(encoderARightPin, INPUT_PULLUP);
    pinMode(encoderBRightPin, INPUT_PULLUP);
    
    // Attach encoder interrupts with CHANGE for better quadrature detection
    attachInterrupt(digitalPinToInterrupt(encoderALeftPin), handleLeftEncoder, CHANGE);
    attachInterrupt(digitalPinToInterrupt(encoderARightPin), handleRightEncoder, CHANGE);

    // Initialize motors to stopped state
    stopMotors();
}

void loop() {
    if (Serial.available() > 0) {
        handleSerialCommand();
    }

    // Check the liquid sensor and control the pump accordingly
    if (pumpActive) {
        int liquid_level = digitalRead(liquid_sensor);
        if (liquid_level == LOW) {  // Liquid detected
            Serial.println("Liquid detected! Pumping...");
            startPump();
        } else {  // No liquid detected
            Serial.println("No liquid detected. Stopping pump.");
            stopPump();  // Stop pump if no liquid is detected
        }
    }

    // Get current encoder distances
    float leftDist = leftEncoderCount * distance_per_count * scale_factor;
    float rightDist = rightEncoderCount * distance_per_count * scale_factor;
    
    // Calculate odometry
    float linearChange = 0;
    float angularChange = 0;
    
    if (isMoving) {
        // Calculate linear and angular changes
        linearChange = (leftDist + rightDist) / 2.0;
        angularChange = (rightDist - leftDist) / wheelbase;
        
        // Update total linear distance only when moving straight
        if (!isRotating) {
            totalLinearDistance += linearChange;
        }
        
        // Accumulate the total angular rotation
        totalAngularRotation += angularChange;
        
        // Reset encoder counts after processing
        leftEncoderCount = 0;
        rightEncoderCount = 0;
    }
    
    // Use total distance for linear movement, final cumulative angle for angular
    float linear = totalLinearDistance;
    float angular = totalAngularRotation;
    
    // Only send data if we're moving or values have changed
    unsigned long currentTime = millis();
    float timestamp = currentTime / 1000.0;
    
    // Don't send duplicate data when stopped
    bool shouldSendData = isMoving || 
                          (linear != lastLinear) || 
                          (angular != lastAngular) ||
                          ((currentTime - lastUpdateTime) > 50); // Force update every second
    
    if (shouldSendData) {
        // Format for ROS compatibility
        Serial.print("D:");
        Serial.print(linear, 4);
        Serial.print(" A:");
        Serial.print(angular, 4);
        Serial.print(" T:");
        Serial.println(timestamp);
        lastLinear = linear;
        lastAngular = angular;
        lastUpdateTime = currentTime;
    }

    delay(50); // Small delay to allow the pump logic to run smoothly
}

// Encoder handlers with improved quadrature detection
void handleLeftEncoder() {
    static int lastA = LOW;
    int a = digitalRead(encoderALeftPin);
    int b = digitalRead(encoderBLeftPin);
    
    if (a != lastA) {
        if (b != a) {
            leftEncoderCount++;  // Forward direction
        } else {
            leftEncoderCount--;  // Backward direction
        }
        lastA = a;
    }
}

void handleRightEncoder() {
    static int lastA = LOW;
    int a = digitalRead(encoderARightPin);
    int b = digitalRead(encoderBRightPin);
    
    if (a != lastA) {
        if (b == a) {  // Reverse phase relationship for right wheel
            rightEncoderCount++;  // Forward direction
        } else {
            rightEncoderCount--;  // Backward direction
        }
        lastA = a;
    }
}

void handleSerialCommand() {
    char command = Serial.read();
    switch (command) {
        case 'F': case 'f': 
            resetPid();
            isRotating = false;
            motorState = 1;
            moveForward(); 
            break;
        case 'B': case 'b': 
            resetPid();
            isRotating = false;
            motorState = 2;
            moveBackward(); 
            break;
        case 'L': case 'l': 
            resetPid();
            isRotating = true;
            motorState = 3;
            turnLeft(); 
            break;
        case 'R': case 'r': 
            resetPid();
            isRotating = true;
            motorState = 4;
            turnRight(); 
            break;
        case 'S': case 's': 
            motorState = 0;
            stopMotors(); 
            break;
        case 'P': case 'p': 
            pumpActive = true; // Start the pump when 'P' is pressed
            break;
        case 'O': case 'o': 
            pumpActive = false; // Stop the pump when 'O' is pressed
            stopPump(); 
            break;
    }
}

// Motor control functions
void moveForward() {
    isMoving = true;
    digitalWrite(motorAIn4, LOW);
    digitalWrite(motorAIn3, HIGH);
    digitalWrite(motorBIn2, LOW);
    digitalWrite(motorBIn1, HIGH);
    setMotorSpeeds(baseSpeed, baseSpeed);
}

void moveBackward() {
    isMoving = true;
    digitalWrite(motorAIn4, HIGH);
    digitalWrite(motorAIn3, LOW);
    digitalWrite(motorBIn2, HIGH);
    digitalWrite(motorBIn1, LOW);
    setMotorSpeeds(baseSpeed, baseSpeed);
}

void turnLeft() {
    isMoving = false;
    digitalWrite(motorAIn4, LOW);
    digitalWrite(motorAIn3, HIGH);
    digitalWrite(motorBIn2, HIGH);
    digitalWrite(motorBIn1, LOW);
    setMotorSpeeds(baseSpeed, baseSpeed);
}

void turnRight() {
    isMoving = false;
    digitalWrite(motorAIn4, HIGH);
    digitalWrite(motorAIn3, LOW);
    digitalWrite(motorBIn2, LOW);
    digitalWrite(motorBIn1, HIGH);
    setMotorSpeeds(baseSpeed, baseSpeed);
}

void stopMotors() {
    isMoving = false;
    setMotorSpeeds(0, 0);
}

void setMotorSpeeds(int leftSpeed, int rightSpeed) {
    analogWrite(motorAPWM, leftSpeed);
    analogWrite(motorBPWM, rightSpeed);
}

// Pump control functions
void startPump() {
    digitalWrite(pumpIn1, HIGH);
    digitalWrite(pumpIn2, LOW);
    analogWrite(pumpPWM, 255); // Full speed
}

void stopPump() {
    digitalWrite(pumpIn1, LOW);
    digitalWrite(pumpIn2, LOW);
    analogWrite(pumpPWM, 0); // Turn off pump
}

void resetPid() {
    errorPrev = 0;
    errorIntegral = 0;
    lastPidTime = millis();
}

