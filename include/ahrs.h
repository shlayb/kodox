#include <Wire.h>
#include <Arduino.h>

const int MPU = 0x68; // MPU6050 I2C address
int16_t tempAccX, tempAccY, tempAccZ;
float AccX, AccY, AccZ;
int16_t tempGyroX, tempGyroY, tempGyroZ;
float GyroX, GyroY, GyroZ;
float accAngleX, accAngleY, gyroAngleX, gyroAngleY;
float roll, pitch, yaw;
float AccErrorX, AccErrorY, GyroErrorX, GyroErrorY, GyroErrorZ;
unsigned long previousTime;
const int numCalibrationSamples = 2000;
const float alpha = 0.9; // Complementary filter constant
float alertThreshold = 40;

//timing variabels
unsigned long previousSensorTime = 0;
unsigned long previousSerialCheckTime = 0;
const unsigned long SENSOR_INTERVAL = 100;    // Read sensor every 100ms
const unsigned long SERIAL_CHECK_INTERVAL = 50;  //Check serial input every 50ms

//buffer for serial input
const int BUFFER_SIZE = 10;
char inputBuffer[BUFFER_SIZE];
int bufferIndex = 0;

// Kalman filter parameters
const float Q = 0.01; // process noise covariance
const float R = 0.03;  // measurement noise covariance
float P_est_X = 1, P_est_Y = 1, P_est_Z = 1;  // Separate estimates per axis
float GyroX_est, GyroY_est, GyroZ_est;

void readAcc() {
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)MPU, (uint8_t)6, (uint8_t)true);
  tempAccX = (Wire.read() << 8 | Wire.read());
  tempAccY = (Wire.read() << 8 | Wire.read());
  tempAccZ = (Wire.read() << 8 | Wire.read());

  AccX = ((float)tempAccX) / 16384.0;
  AccY = ((float)tempAccY) / 16384.0;
  AccZ = ((float)tempAccZ) / 16384.0;

  accAngleX = (atan2(AccY, sqrt(pow(AccX, 2) + pow(AccZ, 2))) * 180 / PI) - AccErrorX;
  accAngleY = (atan2(-AccX, sqrt(pow(AccY, 2) + pow(AccZ, 2))) * 180 / PI) + AccErrorY;
}

void readGyro() {
  Wire.beginTransmission(MPU);
  Wire.write(0x43);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)MPU, (uint8_t)6, (uint8_t)true);
  tempGyroX = (Wire.read() << 8 | Wire.read());
  tempGyroY = (Wire.read() << 8 | Wire.read());
  tempGyroZ = (Wire.read() << 8 | Wire.read());

  GyroX = ((float)tempGyroX) / 131.0 - GyroErrorX;
  GyroY = ((float)tempGyroY) / 131.0 - GyroErrorY;
  GyroZ = ((float)tempGyroZ) / 131.0 - GyroErrorZ;

  // Kalman filter per sumbu
  P_est_X += Q;
  float K_X = P_est_X / (P_est_X + R);
  GyroX_est = GyroX_est + K_X * (GyroX - GyroX_est);
  P_est_X = (1 - K_X) * P_est_X;

  P_est_Y += Q;
  float K_Y = P_est_Y / (P_est_Y + R);
  GyroY_est = GyroY_est + K_Y * (GyroY - GyroY_est);
  P_est_Y = (1 - K_Y) * P_est_Y;

  P_est_Z += Q;
  float K_Z = P_est_Z / (P_est_Z + R);
  GyroZ_est = GyroZ_est + K_Z * (GyroZ - GyroZ_est);
  P_est_Z = (1 - K_Z) * P_est_Z;

  GyroX = GyroX_est;
  GyroY = GyroY_est;
  GyroZ = GyroZ_est;
}

void ahrsSetup() {
  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission(true);

  for (int i = 0; i < numCalibrationSamples; i++) {
    Wire.beginTransmission(MPU);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)MPU, (uint8_t)6, (uint8_t)true);
    tempGyroX = (Wire.read() << 8 | Wire.read());
    tempGyroY = (Wire.read() << 8 | Wire.read());
    tempGyroZ = (Wire.read() << 8 | Wire.read());

    GyroErrorX += ((float)tempGyroX / 131.0);
    GyroErrorY += ((float)tempGyroY / 131.0);
    GyroErrorZ += ((float)tempGyroZ / 131.0);
  }
  GyroErrorX /= numCalibrationSamples;
  GyroErrorY /= numCalibrationSamples;
  GyroErrorZ /= numCalibrationSamples;

  // Initial values
  GyroX_est = GyroErrorX;
  GyroY_est = GyroErrorY;
  GyroZ_est = GyroErrorZ;

  roll = 0;
  pitch = 0;
  yaw = 0;
}


void updateGyro(){
    readAcc();
    readGyro();

    // Integrasi gyro untuk menghitung angle
    gyroAngleX += GyroX * elapsedTime;
    gyroAngleY += GyroY * elapsedTime;
    yaw += GyroZ * elapsedTime;

    // Complementary Filter untuk Roll dan Pitch
    roll = alpha * (roll + GyroX * elapsedTime) + (1 - alpha) * accAngleX;
    pitch = alpha * (pitch + GyroY * elapsedTime) + (1 - alpha) * accAngleY;

    // Penanganan overflow untuk yaw agar tetap dalam rentang [-180, 180]
    if (yaw > 180) yaw -= 360;
    else if (yaw < -180) yaw += 360;
}