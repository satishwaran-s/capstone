// motor A pins
const int motorAIn4 = 47;
const int motorAIn3 = 49;
const int motorAPWM = 10;

// motor B pins
const int motorBIn2 = 43;
const int motorBIn1 = 45;
const int motorBPWM = 9;

// pump control pins
const int pumpIn1 = 5;
const int pumpIn2 = 4;
const int pumpPWM = 3;
const int liquid_sensor = 2;  // non contct liquid sensor pin

// encoder pins
const int encoderALeftPin = 21;  // yellow
const int encoderBLeftPin = 20;  // green
const int encoderARightPin = 18; // yellow
const int encoderBRightPin = 19; // green

// encoder counts (signed for direction)
volatile int leftEncoderCount = 0;
volatile int rightEncoderCount = 0;
float totalLinearDistance = 0.0; // store total linear distance

// motor and wheel specs
const float wheelDiameter = 0.065; // 65mm = 0.065m as per datasheet 
const int ppr = 11; // pulses per revolution as per datasheet 
const int gear_ratio = 150; // 150:1 gear reduction as per datasheet 
const float wheelbase = 0.33; // distance between wheels in meters based on physical setup
const float scale_factor = 2.0; // scaling factor for distance calculation

// state tracking
bool isRotating = false; // whether robot is turning
bool isMoving = false; // whether motors are active
int motorState = 0; // 0=stop, 1=forward, 2=backward, 3=left, 4=right

// calculated constants
const float wheelCircumference = PI * wheelDiameter;
const float counts_per_wheel_rev = ppr * 4 * gear_ratio;  // quadrature encoding
const float distance_per_count = wheelCircumference / counts_per_wheel_rev;

int baseSpeed = 180;  // base motor speed

bool pumpActive = false;  // track if pump should be active

// PID constants
float Kp = 2.5, Ki = 0.1, Kd = 0.05;
float errorPrev = 0, errorIntegral = 0;
unsigned long lastPidTime = 0;

// last sent values to avoid duplicate output when stopped
float lastLinear = 0;
float lastAngular = 0;
unsigned long lastUpdateTime = 0;

// variables for final angular rotation
float totalAngularRotation = 0.0;  // store final cumulative angular rotation

// function Prototypes
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
    
    // initialise motor control pins
    pinMode(motorAIn4, OUTPUT);
    pinMode(motorAIn3, OUTPUT);
    pinMode(motorAPWM, OUTPUT);
    pinMode(motorBIn2, OUTPUT);
    pinMode(motorBIn1, OUTPUT);
    pinMode(motorBPWM, OUTPUT);
    
    // initialise pump control pins
    pinMode(pumpIn1, OUTPUT);
    pinMode(pumpIn2, OUTPUT);
    pinMode(pumpPWM, OUTPUT);
    pinMode(liquid_sensor, INPUT);

    // initialise encoder pins
    pinMode(encoderALeftPin, INPUT_PULLUP);
    pinMode(encoderBLeftPin, INPUT_PULLUP);
    pinMode(encoderARightPin, INPUT_PULLUP);
    pinMode(encoderBRightPin, INPUT_PULLUP);
    
    // attach encoder interrupts with CHANGE for better quadrature detection
    attachInterrupt(digitalPinToInterrupt(encoderALeftPin), handleLeftEncoder, CHANGE);
    attachInterrupt(digitalPinToInterrupt(encoderARightPin), handleRightEncoder, CHANGE);

    // initilise motors to stopped state
    stopMotors();
}

void loop() {
    if (Serial.available() > 0) {
        handleSerialCommand();
    }

    // to call the PID controller only when moving straight
    if (isMoving && !isRotating) {
        updatePid();
    }

    // check the liquid sensor and control the pump accordingly
    if (pumpActive) {
        int liquid_level = digitalRead(liquid_sensor);
        if (liquid_level == LOW) {  // liquid detected
            Serial.println("Liquid detected! Pumping...");
            startPump();
        } else {  // no liquid detected
            Serial.println("No liquid detected. Stopping pump.");
            stopPump();  // stop pump if no liquid is detected
        }
    }

    // get current encoder distances
    float leftDist = leftEncoderCount * distance_per_count * scale_factor;
    float rightDist = rightEncoderCount * distance_per_count * scale_factor;
    
    // calculate odometry
    float linearChange = 0;
    float angularChange = 0;
    
    if (isMoving || isRotating) {
        // calculate linear and angular changes
        linearChange = (leftDist + rightDist) / 2.0;
        angularChange = (rightDist - leftDist) / wheelbase;
        
        // update total linear distance only when moving straight
        if (!isRotating) {
            totalLinearDistance += linearChange;
        }
        
        // accumulate the total angular rotation
        totalAngularRotation += angularChange;
        
        // reset encoder counts after processing
        leftEncoderCount = 0;
        rightEncoderCount = 0;
    }
    
    // use total distance for linear movement, final cumulative angle for angular
    float linear = totalLinearDistance;
    float angular = totalAngularRotation;
    
    // only send data if we're moving or values have changed
    unsigned long currentTime = millis();
    float timestamp = currentTime / 1000.0;
    
    // don't send duplicate data when stopped
    bool shouldSendData = isMoving || 
                          (linear != lastLinear) || 
                          (angular != lastAngular) ||
                          ((currentTime - lastUpdateTime) > 50); // force update every second
    
    if (shouldSendData) {
        // REGEX (regular expression) format for ROS compatibility
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

    delay(50); // short delay to allow the pump logic to run smoothly
}

// encoder handlers with improved quadrature detection
void handleLeftEncoder() {
    static int lastA = LOW;
    int a = digitalRead(encoderALeftPin);
    int b = digitalRead(encoderBLeftPin);
    
    if (a != lastA) {
        if (b != a) {
            leftEncoderCount++;  // forward direction
        } else {
            leftEncoderCount--;  // backward direction
        }
        lastA = a;
    }
}

void handleRightEncoder() {
    static int lastA = LOW;
    int a = digitalRead(encoderARightPin);
    int b = digitalRead(encoderBRightPin);
    
    if (a != lastA) {
        if (b == a) {  // reverse phase relationship for right wheel
            rightEncoderCount++;  // forward direction
        } else {
            rightEncoderCount--;  // backward direction
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
            pumpActive = true; // start the pump 
            break;
        case 'O': case 'o': 
            pumpActive = false; // stop the pump 
            stopPump(); 
            break;
    }
}

// motor control functions
void moveForward() {
    isMoving = true;
    digitalWrite(motorAIn4, LOW);
    digitalWrite(motorAIn3, HIGH);
    digitalWrite(motorBIn2, LOW);
    digitalWrite(motorBIn1, HIGH);
    setMotorSpeeds(baseSpeed, 200);
}

void moveBackward() {
    isMoving = true;
    digitalWrite(motorAIn4, HIGH);
    digitalWrite(motorAIn3, LOW);
    digitalWrite(motorBIn2, HIGH);
    digitalWrite(motorBIn1, LOW);
    setMotorSpeeds(baseSpeed, baseSpeed);
}

void turnRight() {
    isMoving = false;
    digitalWrite(motorAIn4, LOW);
    digitalWrite(motorAIn3, HIGH);
    digitalWrite(motorBIn2, HIGH);
    digitalWrite(motorBIn1, LOW);
    setMotorSpeeds(baseSpeed, baseSpeed);
    
}

void turnLeft() {
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
    analogWrite(pumpPWM, 255); // full speed
}

void stopPump() {
    digitalWrite(pumpIn1, LOW);
    digitalWrite(pumpIn2, LOW);
    analogWrite(pumpPWM, 0); // turn off pump
}

void resetPid() {
    errorPrev = 0;
    errorIntegral = 0;
    lastPidTime = millis();
}

void updatePid() {
    // calculate time since last update
    unsigned long currentTime = millis();
    float deltaTime = (currentTime - lastPidTime) / 1000.0; // convert to seconds
    lastPidTime = currentTime;
    
    // skip if time delta is too small
    if (deltaTime < 0.01) return;
    
    // calculate error (difference between left and right encoder counts)
    float leftDist = leftEncoderCount * distance_per_count;
    float rightDist = rightEncoderCount * distance_per_count;
    float error = leftDist - rightDist;
    
    // calculate PID terms
    float errorDerivative = (error - errorPrev) / deltaTime;
    errorIntegral += error * deltaTime;
    errorPrev = error;
    
    // anti-windup for integral term
    if (errorIntegral > 10) errorIntegral = 10;
    if (errorIntegral < -10) errorIntegral = -10;
    
    // calculate correction
    float correction = Kp * error + Ki * errorIntegral + Kd * errorDerivative;
    
    // apply correction to motor speeds
    int leftSpeed = baseSpeed;
    int rightSpeed = baseSpeed;
    
    // only apply correction when moving straight
    if (!isRotating && isMoving) {
        leftSpeed -= correction;
        rightSpeed += correction;
        
        // constrain speeds
        if (leftSpeed < 0) leftSpeed = 0;
        if (leftSpeed > 255) leftSpeed = 255;
        if (rightSpeed < 0) rightSpeed = 0;
        if (rightSpeed > 255) rightSpeed = 255;
        
        // apply the new speeds
        setMotorSpeeds(leftSpeed, rightSpeed);
    }
}
