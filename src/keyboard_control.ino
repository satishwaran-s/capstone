
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

// Encoder Pins (Phase A and Phase B for both motors)
const int encoderALeftPin = 18;  // Motor A Yellow (Phase A)
const int encoderBLeftPin = 19;  // Motor A Green (Phase B)
const int encoderARightPin = 20; // Motor B Yellow (Phase A)
const int encoderBRightPin = 21; // Motor B Green (Phase B)

bool pumpActive = false;

// Variables for encoder counts
volatile int leftEncoderCount = 0;
volatile int rightEncoderCount = 0;

// Encoder interrupt handlers
void encoderALeftISR() {
    if (digitalRead(encoderBLeftPin) == HIGH) {
        leftEncoderCount++;
    } else {
        leftEncoderCount--;
    }
}

void encoderARightISR() {
    if (digitalRead(encoderBRightPin) == HIGH) {
        rightEncoderCount++;
    } else {
        rightEncoderCount--;
    }
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

    // Set encoder pins as inputs
    pinMode(encoderALeftPin, INPUT);
    pinMode(encoderBLeftPin, INPUT);
    pinMode(encoderARightPin, INPUT);
    pinMode(encoderBRightPin, INPUT);

    // Attach interrupts for both encoders
    attachInterrupt(digitalPinToInterrupt(encoderALeftPin), encoderALeftISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(encoderARightPin), encoderARightISR, CHANGE);

    Serial.begin(115200);
    Serial.println("Robot Control Ready");
}

void loop() {
    // Print encoder counts
    Serial.print("Left Encoder Count: ");
    Serial.println(leftEncoderCount);
    Serial.print("Right Encoder Count: ");
    Serial.println(rightEncoderCount);

    // Process commands from Serial
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
            default:
              Serial.print("Unknown command: ");
              Serial.println(command);
              break;
        }
    }

    delay(100);  // Add delay for readability
}

void moveForward() {
    analogWrite(motorAPWM, 222);
    analogWrite(motorBPWM, 240);
    digitalWrite(motorAIn1, LOW);
    digitalWrite(motorAIn2, HIGH);
    digitalWrite(motorBIn3, LOW);
    digitalWrite(motorBIn4, HIGH);
    Serial.println("Moving Forward");
}

void moveBackward() {
    analogWrite(motorAPWM, 222);
    analogWrite(motorBPWM, 190);
    digitalWrite(motorAIn1, HIGH);
    digitalWrite(motorAIn2, LOW);
    digitalWrite(motorBIn3, HIGH);
    digitalWrite(motorBIn4, LOW);
    Serial.println("Moving Backward");
}

void turnRight() {
    analogWrite(motorAPWM, 200);
    analogWrite(motorBPWM, 200);
    digitalWrite(motorAIn1, LOW);
    digitalWrite(motorAIn2, HIGH);
    digitalWrite(motorBIn3, HIGH);
    digitalWrite(motorBIn4, LOW);
    Serial.println("Turning Right");
}

void turnLeft() {
    analogWrite(motorAPWM, 200);
    analogWrite(motorBPWM, 200);
    digitalWrite(motorAIn1, HIGH);
    digitalWrite(motorAIn2, LOW);
    digitalWrite(motorBIn3, LOW);
    digitalWrite(motorBIn4, HIGH);
    Serial.println("Turning Left");
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

void startPump() {
    pumpActive = true;
    digitalWrite(pumpIn1, HIGH);  // Ensure this is correct for your pump's direction
    digitalWrite(pumpIn2, LOW);   // Ensure this is correct for your pump's direction
    analogWrite(pumpPWM, 255);    // Try lowering this value if the pump is not spraying well
    Serial.println("Pump Activated");
}

void stopPump() {
    pumpActive = false;
    digitalWrite(pumpIn1, LOW);   // Ensure pump is stopped
    digitalWrite(pumpIn2, LOW);   // Ensure pump is stopped
    analogWrite(pumpPWM, 0);      // Stop PWM signal to pump
    Serial.println("Pump Stopped");
}
