#include <Servo.h>

// Define pins for ultrasonic sensors
#define TRIGPIN_FRONT  4
#define ECHOPIN_FRONT  5
#define TRIGPIN_LEFT   6
#define ECHOPIN_LEFT   7
#define TRIGPIN_RIGHT  8
#define ECHOPIN_RIGHT  9

// Define ultrasonic distance variables
int distanceFront, distanceLeft, distanceRight;

// Define movement pulses
const int stopPulse = 1500;         // Neutral position for stop
const int forwardPulse = 1700;      // Adjust for forward speed (values > 1500)
const int backwardPulse = 1300;     // Adjust for backward speed (values < 1500)

int leftWheelPulse;
int rightWheelPulse;

// Define PID variables
float error = 0;
float last_error = 0;
float integral = 0;
float derivative = 0;
float PID_value = 0;

// Recommended PID constants (adjust based on your specific robot)
const float Kp = 2.5;  // Proportional gain
const float Ki = 0.05; // Integral gain (small to prevent windup)
const float Kd = 1.8;  // Derivative gain

// Setpoint for left wall following (target distance from wall in cm)
const int left_setpoint = 5;

// Create Servo objects for motors
Servo leftMotor;
Servo rightMotor;

void setup() {
  Serial.begin(9600);

  // Initialize ultrasonic sensor pins
  pinMode(TRIGPIN_FRONT, OUTPUT);
  pinMode(ECHOPIN_FRONT, INPUT);
  pinMode(TRIGPIN_LEFT, OUTPUT);
  pinMode(ECHOPIN_LEFT, INPUT);
  pinMode(TRIGPIN_RIGHT, OUTPUT);
  pinMode(ECHOPIN_RIGHT, INPUT);

  // Attach motors
  leftMotor.attach(12);   // Assuming left motor is on pin 12
  rightMotor.attach(13);  // Assuming right motor is on pin 13
}

// Function to measure distance for a given ultrasonic sensor
int measureDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  return pulseIn(echoPin, HIGH) * 0.034 / 2;  // Convert time to distance in cm
}

// Function to calculate the PID value for left wall following
float calculatePID(float current_distance) {
    error = left_setpoint - current_distance;
    
    // Prevent integral wind-up by clamping
    integral += error;
    if (integral > 50) integral = 50;
    if (integral < -50) integral = -50;

    derivative = error - last_error;
    PID_value = Kp * error + Ki * integral + Kd * derivative;

    // Clamp PID_value to prevent out-of-range motor commands
    if (PID_value > 45) PID_value = 45;
    if (PID_value < -45) PID_value = -45;

    last_error = error;
    return PID_value;
}

void sharp_right() {
  Serial.println("Turning right");
  leftMotor.writeMicroseconds(forwardPulse);
  rightMotor.writeMicroseconds(forwardPulse);
  delay(500);  // Adjust turn duration as needed
}

void sharp_left() {
  Serial.println("Turning left");
  leftMotor.writeMicroseconds(backwardPulse);
  rightMotor.writeMicroseconds(backwardPulse);
  delay(500);  // Adjust turn duration as needed
}

void stop_robot() {
  leftMotor.writeMicroseconds(stopPulse);
  rightMotor.writeMicroseconds(stopPulse);
}

void setMotorSpeed(float pid_output) {
  // Base forward movement pulses
  int leftBasePulse = 1300;   // Backwards pulse for forward movement
  int rightBasePulse = 1700;  // Forward pulse for forward movement

  // Apply PID correction
  int leftCorrectedPulse = leftBasePulse + pid_output;
  int rightCorrectedPulse = rightBasePulse - pid_output;

  // Ensure pulses stay within valid range
  leftCorrectedPulse = constrain(leftCorrectedPulse, 1300, 1500);
  rightCorrectedPulse = constrain(rightCorrectedPulse, 1500, 1700);

  // Write corrected pulses to motors
  leftMotor.writeMicroseconds(leftCorrectedPulse);
  rightMotor.writeMicroseconds(rightCorrectedPulse);
}

void navigate() {
  // Measure distances using ultrasonic sensors
  distanceFront = measureDistance(TRIGPIN_FRONT, ECHOPIN_FRONT);
  distanceLeft = measureDistance(TRIGPIN_LEFT, ECHOPIN_LEFT);
  distanceRight = measureDistance(TRIGPIN_RIGHT, ECHOPIN_RIGHT);

  // Debugging outputS
  Serial.print("Front: "); Serial.print(distanceFront);
  Serial.print(" cm, Left: "); Serial.print(distanceLeft);
  Serial.print(" cm, Right: "); Serial.print(distanceRight);

  // Obstacle avoidance and navigation logic
  if (distanceFront < 10) {
    // Obstacle directly in front
    if (distanceRight > distanceLeft) {
        Serial.println("Turning rights");
        sharp_right();
    } else if (distanceLeft > distanceRight) {
        if (distanceLeft > 1100){
          sharp_right();
        }else{
          Serial.println("Turning lefts");
          sharp_left();
        }
    } else {
        Serial.println("Moving forward");
    }
    return;
  }

  if (distanceLeft < 7 && distanceFront < 7 && distanceRight > 10) {
    // Tight corner scenario
    sharp_right();
    return;
  }

  // Normal wall following using PID
  if (distanceLeft > 0 && distanceLeft < 50) {
    float pid_output = calculatePID(distanceLeft);
    Serial.print(" PID: "); Serial.println(pid_output);
    
    // Use PID to adjust direction while maintaining forward movement
    setMotorSpeed(pid_output);
  } else {
    // If no clear wall detected, move forward
    setMotorSpeed(0);
  }
}

void loop() {
  navigate();
}