// Define motor pins for left front motor
const int leftFrontMotorPWM_enB   = 6;    // PWM pin for left front motor speed control
const int leftFrontMotorDir1_IN4  = 22;    // Direction pin 1 for left front motor
const int leftFrontMotorDir2_IN3  = 24;    // Direction pin 2 for left front motor

// Define motor pins for left back motor
const int leftBackMotorPWM_enA    = 9;    // PWM pin for left back motor speed control
const int leftBackMotorDir1_IN2   = 26;    // Direction pin 1 for left back motor
const int leftBackMotorDir2_IN1   = 28
;    // Direction pin 2 for left back motor

// Define motor pins for right front motor
const int rightFrontMotorPWM_enA  = 10;    // PWM pin for right front motor speed control
const int rightFrontMotorDir1_IN1 = 52;   // Direction pin 1 for right front motor
const int rightFrontMotorDir2_IN2 = 50;   // Direction pin 2 for right front motor

// Define motor pins for right back motor
const int rightBackMotorPWM_enB   = 11;   // PWM pin for right back motor speed control
const int rightBackMotorDir1_IN3  = 48;   // Direction pin 1 for right back motor
const int rightBackMotorDir2_IN4  = 46;   // Direction pin 2 for right back motor

// Define motor speeds
const int motorSpeed = 150;   // Adjust this value for desired speed (0-255)

void setup() {
  // Initialize motor pins as outputs
  pinMode(leftFrontMotorPWM_enB, OUTPUT);
  pinMode(leftFrontMotorDir1_IN4, OUTPUT);
  pinMode(leftFrontMotorDir2_IN3, OUTPUT);
  pinMode(leftBackMotorPWM_enA, OUTPUT);
  pinMode(leftBackMotorDir1_IN2, OUTPUT);
  pinMode(leftBackMotorDir2_IN1, OUTPUT);
  pinMode(rightFrontMotorPWM_enA, OUTPUT);
  pinMode(rightFrontMotorDir1_IN1, OUTPUT);
  pinMode(rightFrontMotorDir2_IN2, OUTPUT);
  pinMode(rightBackMotorPWM_enB, OUTPUT);
  pinMode(rightBackMotorDir1_IN3, OUTPUT);
  pinMode(rightBackMotorDir2_IN4, OUTPUT);
}

void loop() {
  // Move forward
  moveForward();
  delay(2000);  // Move forward for 2 seconds
  stopMotors(); // Stop the robot
  delay(1000);  // Pause for 1 second
}

void moveForward() {
  // Set motor directions for forward movement
  digitalWrite(leftFrontMotorDir1_IN4, HIGH);
  digitalWrite(leftFrontMotorDir2_IN3, LOW);
  digitalWrite(leftBackMotorDir1_IN2, HIGH);
  digitalWrite(leftBackMotorDir2_IN1, LOW);
  digitalWrite(rightFrontMotorDir1_IN1, HIGH);
  digitalWrite(rightFrontMotorDir2_IN2, LOW);
  digitalWrite(rightBackMotorDir1_IN3, HIGH);
  digitalWrite(rightBackMotorDir2_IN4, LOW);
  
  // Set motor speeds
  analogWrite(leftFrontMotorPWM_enB, motorSpeed);
  analogWrite(leftBackMotorPWM_enA, motorSpeed);
  analogWrite(rightFrontMotorPWM_enA, motorSpeed);
  analogWrite(rightBackMotorPWM_enB, motorSpeed);
}

void stopMotors() {
  // Turn off all motors
  digitalWrite(leftFrontMotorPWM_enB, LOW);
  digitalWrite(leftBackMotorPWM_enA, LOW);
  digitalWrite(rightFrontMotorPWM_enA, LOW);
  digitalWrite(rightBackMotorPWM_enB, LOW);
}
