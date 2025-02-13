#include <Wire.h>

// Declare sensor readings
float RateRoll, RatePitch, RateYaw;
float AccX, AccY, AccZ;
float AngleRoll, AnglePitch, AngleYaw;
float dt; // Time delta

void gyro_signals(void) {
  // Read accelerometer data
  Wire.beginTransmission(0x68);
  Wire.write(0x3B); // Start reading from the accelerometer data registers
  Wire.endTransmission();
  Wire.requestFrom(0x68, 6); // 6 bytes to read: AccX, AccY, AccZ
  int16_t AccXLSB = Wire.read() << 8 | Wire.read();
  int16_t AccYLSB = Wire.read() << 8 | Wire.read();
  int16_t AccZLSB = Wire.read() << 8 | Wire.read();

  // Read gyroscope data
  Wire.beginTransmission(0x68);
  Wire.write(0x43); // Start reading from the gyroscope data registers
  Wire.endTransmission();
  Wire.requestFrom(0x68, 6); // 6 bytes to read: GyroX, GyroY, GyroZ
  int16_t GyroX = Wire.read() << 8 | Wire.read();
  int16_t GyroY = Wire.read() << 8 | Wire.read();
  int16_t GyroZ = Wire.read() << 8 | Wire.read();

  // Convert accelerometer data to 'g'
  AccX = (float)AccXLSB / 4096.0;
  AccY = (float)AccYLSB / 4096.0;
  AccZ = (float)AccZLSB / 4096.0;

  // Convert gyroscope data to degrees per second (gyro sensitivity is 65.5 for ±250°/s)
  RateRoll = (float)GyroX / 65.5;
  RatePitch = (float)GyroY / 65.5;
  RateYaw = (float)GyroZ / 65.5;

  // Calculate angles from accelerometer data
  AngleRoll = atan(AccY / sqrt(AccX * AccX + AccZ * AccZ)) * (180 / 3.141592653589793);
  AnglePitch = -atan(AccX / sqrt(AccY * AccY + AccZ * AccZ)) * (180 / 3.141592653589793);

  // Use a fixed dt since we are using delay(50) -> 50ms = 0.05s
  dt = 50 / 1000.0;  // 50ms to seconds

  // Integrate the gyroscope data to calculate yaw
  AngleYaw += RateYaw * dt; // Integrating angular velocity to get angle

    if (AngleYaw > 90.0) {
    AngleYaw = 90.0;
  } 
  else if (AngleYaw < -90.0) {
    AngleYaw = -90.0;
  }
  
  // Handle the wrap-around for yaw, i.e., if it goes beyond 360° or below -360°
  if (AngleYaw > 180.0) {
    AngleYaw -= 360.0;  // Wrap around positive angle
  }
  else if (AngleYaw < -180.0) {
    AngleYaw += 360.0;  // Wrap around negative angle
  }
}

void setup() {
  // Initialize serial and I2C
  Serial.begin(57600);
  Wire.setClock(400000);  // Set I2C clock speed to 400kHz (optional)
  Wire.begin();

  // Wake up the MPU-6050 (set it to normal mode)
  Wire.beginTransmission(0x68); 
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0x00); // Wake up the MPU-6050
  Wire.endTransmission();
}

void loop() {
  // Read sensor data and compute angles
  gyro_signals();

  // Print the calculated angles
  Serial.print("Angle Roll = ");
  Serial.print(AngleRoll);
  Serial.print(" Angle Pitch = ");
  Serial.print(AnglePitch);
  Serial.print(" Angle Yaw = ");
  Serial.println(AngleYaw);

  // Wait for the next loop iteration
  delay(50); // Delay 50ms to limit the reading rate
}
