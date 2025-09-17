#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// Define servo motor driver
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

// Servo pulse range
#define SERVOMIN 150
#define SERVOMAX 600

// Servo channel mapping
#define BASE_SERVO 0
#define SHOULDER_SERVO 1
#define ELBOW_SERVO 2
#define GRIPPER_SERVO 3

// Define motor driver pins
#define ENA 3
#define IN1 4
#define IN2 5
#define ENB 6
#define IN3 7
#define IN4 8

// Define IR sensor pins
#define IR_LEFT 9
#define IR_RIGHT 10

// Define ultrasonic sensor pins
#define TRIG_PIN 11
#define ECHO_PIN 12

// T-junction tracking
bool atTJunction = false;
char lastBTSignal = 'G'; // Default to forward

// Function to move servo gradually
void moveServoSmoothly(int servo, int startAngle, int endAngle, int stepDelay) {
    if (startAngle < endAngle) {
        for (int angle = startAngle; angle <= endAngle; angle++) {
            pwm.setPWM(servo, 0, map(angle, 0, 180, SERVOMIN, SERVOMAX));
            delay(stepDelay);
        }
    } else {
        for (int angle = startAngle; angle >= endAngle; angle--) {
            pwm.setPWM(servo, 0, map(angle, 0, 180, SERVOMIN, SERVOMAX));
            delay(stepDelay);
        }
    }
}

// Ultrasonic distance measurement
long getDistance() {
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
    long duration = pulseIn(ECHO_PIN, HIGH, 30000);
    return (duration == 0) ? 9999 : (duration / 2) / 29.1;
}

// Movement functions
void moveForward() {
    if (getDistance() <= 6) {
        stopMotors();
        Serial.println("Obstacle detected! Stopping.");
        return;
    }
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENA, 60);
    analogWrite(ENB, 60);
}

void stopMotors() {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    analogWrite(ENA, 0);
    analogWrite(ENB, 0);
}

void moveLeft() {
    Serial.println("Turning Left...");
    stopMotors();
    delay(100);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, 160);
    delay(1761);
    stopMotors();
    delay(200);
    moveForward();
}

void moveRight() {
    Serial.println("Turning Right...");
    stopMotors();
    delay(100);
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, 160);
    delay(1761);
    stopMotors();
    delay(200);
    moveForward();
}

void adjustLine() {
    Serial.println("Adjusting alignment...");
    stopMotors();
    delay(200);
}

void pickupBox() {
    Serial.println("Picking up the box...");
    moveServoSmoothly(SHOULDER_SERVO, 40, 80, 30);
    moveServoSmoothly(ELBOW_SERVO, 140, 130, 30);
    moveServoSmoothly(GRIPPER_SERVO, 30, 6, 50);
    delay(300);
    moveServoSmoothly(ELBOW_SERVO, 130, 140, 30);
    moveServoSmoothly(SHOULDER_SERVO, 80, 30, 30);
    Serial.println("Box picked up!");
}

void dropBox() {
    Serial.println("Dropping the box...");
    moveServoSmoothly(SHOULDER_SERVO, 30, 80, 30);
    moveServoSmoothly(ELBOW_SERVO, 140, 130, 30);
    moveServoSmoothly(GRIPPER_SERVO, 6, 30, 50);
    Serial.println("Box dropped!");
}

void returnToStart() {
    Serial.println("Returning to start position...");
    while (true) {
        int leftSensor = digitalRead(IR_LEFT);
        int rightSensor = digitalRead(IR_RIGHT);
        if (leftSensor == LOW && rightSensor == LOW) {
            moveForward();
        } else if (leftSensor == HIGH && rightSensor == LOW) {
            moveRight();
        } else if (leftSensor == LOW && rightSensor == HIGH) {
            moveLeft();
        } else {
            stopMotors();
            delay(300);
            Serial.println(lastBTSignal);
            if (lastBTSignal == 'R') moveRight();
            else if (lastBTSignal == 'G') { moveForward(); delay(500); stopMotors(); adjustLine(); }
            else if (lastBTSignal == 'B') moveLeft();
        }
        if (leftSensor == HIGH && rightSensor == HIGH) {
            stopMotors();
            Serial.println("Start position reached!");
            break;
        }
    }
}

void executeTurn(char signal) {
    lastBTSignal = signal;
    if (signal == 'R') moveRight();
    else if (signal == 'L') moveLeft();
    else if (signal == 'G') { moveForward(); delay(500); stopMotors(); adjustLine(); }
    atTJunction = false;
}

void setup() {
    Serial.begin(9600);
    Wire.begin();
    pwm.begin();
    pwm.setPWMFreq(65);
    pinMode(ENA, OUTPUT);
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(ENB, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
    pinMode(IR_LEFT, INPUT);
    pinMode(IR_RIGHT, INPUT);
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
    Serial.println("System Initialized");
}

void loop() {
    pickupBox();

    while (true) {
        int leftSensor = digitalRead(IR_LEFT);
        int rightSensor = digitalRead(IR_RIGHT);

        if (!atTJunction) {
            if (leftSensor == LOW && rightSensor == LOW) moveForward();
            else if (leftSensor == HIGH && rightSensor == LOW) moveRight();
            else if (leftSensor == LOW && rightSensor == HIGH) moveLeft();
        }
        
        if (atTJunction && Serial.available()) executeTurn(Serial.read());
        if (Serial.available() && Serial.read() == 'D') { dropBox(); break; }
    }
    returnToStart();
}
