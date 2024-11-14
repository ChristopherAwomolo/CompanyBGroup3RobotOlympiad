
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

// Define PID variables
float error = 0;
float last_error = 0;
float integral = 0;
float derivative = 0;
float PID_value = 0;

// PID constants
const float Kp = 1.2;
const float Ki = 0.005;
const float Kd = 1.3;

// Setpoint for left wall following (target distance from wall in cm)
const int left_setpoint = 20;

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
float calculatePID(int current_distance) {
  error = left_setpoint - current_distance;
  integral += error;
  derivative = error - last_error;
  PID_value = Kp * error + Ki * integral + Kd * derivative;
  last_error = error;
  return PID_value;
}

void leftwall_follow(){
  //sets the PID_value to be lowest to -45 and highest to 45
  if(PID_value < -correctionSpeed){
    PID_value = -correctionSpeed;
    //Serial.println(PID_value);
    //delay(1000);
  }
  if(PID_value> correctionSpeed){
    PID_value = correctionSpeed;
    //Serial.println(PID_value);
    // delay(1000);
  }
  if(PID_value < 0)
  {
    rightWheelSpeed = base_speedRight - PID_value   ;
    leftWheelSpeed = base_speedLeft - PID_value;
  }
  else
  {
    rightWheelSpeed = base_speedRight - PID_value;
    leftWheelSpeed = base_speedLeft - PID_value;
  }
  servoLeft.write(leftWheelSpeed);
  servoRight.write(rightWheelSpeed);
  //sharp right turn(90 degree turn) when theres a wall on the left and wall at the front as well
  if(SonarDistance <7 && SonarDistance !=0 && IrDistance > 100)
  {
    sharp_right();
  }
}


// Main loop function with left wall following using PID
void loop() {
  // Measure distances from each sensor
  distanceFront = measureDistance(TRIGPIN_FRONT, ECHOPIN_FRONT);
  distanceLeft = measureDistance(TRIGPIN_LEFT, ECHOPIN_LEFT);
  distanceRight = measureDistance(TRIGPIN_RIGHT, ECHOPIN_RIGHT);

  // Debugging information
  Serial.print("Front: "); Serial.print(distanceFront);
  Serial.print(" cm, Left: "); Serial.print(distanceLeft);
  Serial.print(" cm, Right: "); Serial.println(distanceRight);

  // PID calculation for left wall following
  float pid_output = calculatePID(distanceLeft);

  // Movement control based on PID output and obstacle detection
  if (distanceFront > 20) { // Move forward if no obstacle within 20 cm
    // Adjust motor speeds based on PID output for wall following
    leftMotor.writeMicroseconds(backwardPulse + pid_output);
    rightMotor.writeMicroseconds(forwardPulse - pid_output);
  } 
  else if (distanceFront <= 20) {  // Stop if obstacle is too close in front
    leftMotor.writeMicroseconds(stopPulse);
    rightMotor.writeMicroseconds(stopPulse-22);
  }

  delay(100);  // Short delay to stabilize sensor readings
}
