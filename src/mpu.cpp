#include <Arduino.h>
#include <Wire.h>
#include <ESP32Servo.h>
#include "I2Cdev.h"
#include <VL53L0X.h>
#include "variables.h"
#include "mpu.h"
#include "piloto_mode.h"

#define SDA_MPU 21
#define SCL_MPU 22
#define SDA_TOF 4
#define SCL_TOF 5

void gyro_signals(void)
{
  Wire.beginTransmission(0x68);
  Wire.write(0x1A);
  Wire.write(0x03);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 6);
  int16_t AccXLSB = Wire.read() << 8 | Wire.read();
  int16_t AccYLSB = Wire.read() << 8 | Wire.read();
  int16_t AccZLSB = Wire.read() << 8 | Wire.read();
  Wire.beginTransmission(0x68);
  Wire.write(0x1B);
  Wire.write(0x8);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 6);
  int16_t GyroX = Wire.read() << 8 | Wire.read();
  int16_t GyroY = Wire.read() << 8 | Wire.read();
  int16_t GyroZ = Wire.read() << 8 | Wire.read();

  AccX = (float)AccXLSB / 4096;
  AccY = (float)AccYLSB / 4096;
  AccZ = (float)AccZLSB / 4096;

  AngleRoll_est = atan(AccY / sqrt(AccX * AccX + AccZ * AccZ)) * 57.29;
  AnglePitch_est = -atan(AccX / sqrt(AccY * AccY + AccZ * AccZ)) * 57.29;

  accAngleX = (atan(AccY / sqrt(pow(AccX, 2) + pow(AccZ, 2))) * 180 / PI) - 0.58;
  accAngleY = (atan(-1 * AccX / sqrt(pow(AccY, 2) + pow(AccZ, 2))) * 180 / PI) + 1.58;

  if (AccZ != 0)
  {
    accAngleRoll = (atan(AccY / sqrt(pow(AccX, 2) + pow(AccZ, 2))) * 180 / PI) - 0.58;       // Ángulo de roll (grados)
    accAnglePitch = (atan(-1 * AccX / sqrt(pow(AccY, 2) + pow(AccZ, 2))) * 180 / PI) + 1.58; // Ángulo de pitch (grados)
  }
  else
  {
    Serial.println("Error: AccZ es cero");
  }
  gyroRateRoll = GyroX / 65.5; // Tasa de giro en grados/segundo
  gyroRatePitch = GyroY / 65.5;
  RateYaw = GyroZ / 65.5;
}

void setupMPU()
{
  Serial.println("Iniciando sensores...");
  Wire.begin(SDA_MPU, SCL_MPU); // Ensure correct I2C pins are used
  Wire.setClock(400000);        // Set I2C clock speed to 400kHz
  accelgyro.initialize();
  delay(20); // Añade esto
  calibrateSensors();
  if (!accelgyro.testConnection())
  {
    Serial.println("Error: No se pudo conectar con el MPU6050.");
    while (true)
    {
      delay(1000); // Halt execution if the sensor is not connected
    }
  }
  Serial.println("MPU6050 conectado correctamente.");
}
