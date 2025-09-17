Smart Seed-Vault-Storing system

This project is an Arduino-based robot capable of picking up boxes, following lines, avoiding obstacles, and navigating T-junctions based on serial commands. It uses servo motors, an ultrasonic sensor, IR sensors, and a motor driver to perform automated tasks.

Features

Line Following: Uses left and right IR sensors to follow a black line.

Obstacle Avoidance: Ultrasonic sensor stops the robot when an obstacle is detected.

Box Handling: Servo-controlled robotic arm picks up and drops boxes.

T-Junction Navigation: Turns at junctions based on serial commands:

R – Turn Right

L – Turn Left

G – Go Forward

Return to Start: After completing the task, the robot follows the line back to the start position.

Hardware Components
Component	Quantity
Arduino Uno / Nano / Mega	1
Adafruit PWM Servo Driver (PCA9685)	1
SG90 / MG90S Servo Motors	4
L298N Motor Driver or H-Bridge	1
DC Motors	2
Ultrasonic Sensor (HC-SR04)	1
IR Line Tracking Sensors	2
Jumper Wires / Breadboard	As needed
Wiring Overview

Servos: Connected to PWM driver channels: Base (0), Shoulder (1), Elbow (2), Gripper (3)

DC Motors: Controlled via ENA/IN1/IN2 and ENB/IN3/IN4 pins

IR Sensors: Connected to digital pins 9 (Left) and 10 (Right)

Ultrasonic Sensor: TRIG – pin 11, ECHO – pin 12

Installation

Clone the repository:

git clone https://github.com/yourusername/smart-box-robot.git


Open the Arduino IDE.

Install the required library:

Adafruit PWM Servo Driver

Open SmartBoxRobot.ino and upload it to your Arduino board.

Usage

Place the robot on a line track.

Open Serial Monitor at 9600 baud rate.

The robot will automatically pick up the box.

Send serial commands to navigate T-junctions:

R – Turn Right

L – Turn Left

G – Go Forward

Send D to drop the box. The robot will return to the starting position.

How It Works

Line Following: Continuously reads IR sensors to detect the line.

Obstacle Avoidance: Stops if an obstacle is within 6 cm using the ultrasonic sensor.

Box Handling: Uses servo motors to pick up and release the box smoothly.

T-Junctions: Serial commands decide the direction at junctions.

Return Home: Follows the line back to the starting point after dropping the box.

Future Improvements

Non-blocking code using millis() for better responsiveness.

Bluetooth control for wireless T-junction navigation.

Integration with a camera or sensor for object detection.

PID line-following algorithm for smoother navigation.
