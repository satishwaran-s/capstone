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

// Speed Variables
int motorSpeed = 150;
const int speedStep = 25;
bool pumpActive = false;

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
    
    Serial.begin(115200);
    Serial.println("Robot Control Ready");
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
            case 'P': startPump(); break;
            case 'O': stopPump(); break;
            default:
              Serial.print("Unknown command: ");
              Serial.println(command);
              break;
        }
    }
    
    // Check liquid sensor only when pump is active
    if (pumpActive) {
        int liquid_level = digitalRead(liquid_sensor);
        if (liquid_level == LOW) {  // Liquid detected
            Serial.println("Liquid detected! Pumping...");
            digitalWrite(pumpIn1, HIGH);  // Ensure correct direction
            digitalWrite(pumpIn2, LOW);   // Ensure correct direction
            analogWrite(pumpPWM, 255);    // Full PWM speed (adjust if needed)
        } else {
            Serial.println("No liquid detected. Stopping pump.");
            stopPump();  // Stop pump if no liquid is detected
        }
    }

    delay(100); 
}

void moveForward() {
    analogWrite(motorAPWM, 222);
    analogWrite(motorBPWM, 190);
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
