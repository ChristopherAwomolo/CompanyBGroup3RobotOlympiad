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

// Main loop function with movement logic based on sensor readings
void loop() {
  // Measure distances from each sensor
  distanceFront = measureDistance(TRIGPIN_FRONT, ECHOPIN_FRONT);
  distanceLeft = measureDistance(TRIGPIN_LEFT, ECHOPIN_LEFT);
  distanceRight = measureDistance(TRIGPIN_RIGHT, ECHOPIN_RIGHT);

  // Debugging information
  Serial.print("Front: "); Serial.print(distanceFront);
  Serial.print(" cm, Left: "); Serial.print(distanceLeft);
  Serial.print(" cm, Right: "); Serial.println(distanceRight);

  // Movement logic based on distances
  if (distanceFront > 20) {  // Move forward if no obstacle within 20 cm
    leftMotor.writeMicroseconds(backwardPulse);
    rightMotor.writeMicroseconds(forwardPulse);
  } 
  else if (distanceLeft > 20 && distanceRight <= 20) {  // Move left if right is blocked
    leftMotor.writeMicroseconds(stopPulse);
    rightMotor.writeMicroseconds(forwardPulse);
  } 
  else if (distanceRight > 20 && distanceLeft <= 20) {  // Move right if left is blocked
    leftMotor.writeMicroseconds(backwardPulse);
    rightMotor.writeMicroseconds(stopPulse);
  } 
  else {  // Stop if obstacles are close on all sides
    leftMotor.writeMicroseconds(stopPulse);
    rightMotor.writeMicroseconds(stopPulse);
  }

  delay(100);  // Short delay to avoid excessive looping
}
