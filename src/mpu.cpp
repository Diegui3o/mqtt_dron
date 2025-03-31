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

// Declare and initialize Q and R
float Q[2][2] = {{0.001, 0}, {0, 0.003}}; // Process noise covariance matrix
float R = 0.0005;                         // Measurement noise covariance

// Función para el filtro de Kalman (roll)
template <size_t N>
void kalmanUpdate(volatile float &angle, volatile float &bias, float P[2][2],
                  float accAngle, float gyroRate,
                  float residual_history[], int &residual_index,
                  float &R_angle, float &lambda)
{

  // 0. Aplicar factor de olvido ANTES de la predicción
  for (int i = 0; i < 2; i++)
  {
    for (int j = 0; j < 2; j++)
    {
      P[i][j] *= lambda;
    }
  }

  // 1. Predicción del estado
  angle += dt * (gyroRate - bias);

  // 2. Predicción de covarianza (Forma matricial correcta)
  float F[2][2] = {{1, -dt}, {0, 1}};
  float P_pred[2][2] = {0};

  // F * P
  for (int i = 0; i < 2; i++)
  {
    for (int j = 0; j < 2; j++)
    {
      P_pred[i][j] = F[i][0] * P[0][j] + F[i][1] * P[1][j];
    }
  }

  // (F * P) * F^T + Q
  for (int i = 0; i < 2; i++)
  {
    for (int j = 0; j < 2; j++)
    {
      P[i][j] = 0;
      for (int k = 0; k < 2; k++)
      {
        P[i][j] += P_pred[i][k] * F[j][k];
      }
      P[i][j] += Q[i][j] * dt; // Q es por unidad de tiempo
    }
  }

  // 3. Residual y ventana
  float residual = accAngle - angle;
  residual_history[residual_index % window_size] = residual * residual;
  residual_index++;

  // 4. Calcular Ck (covarianza residual)
  float Ck = 0;
  for (int i = 0; i < window_size; i++)
  {
    Ck += residual_history[i];
  }
  Ck /= window_size;

  // 5. Adaptar R dinámicamente
  float sigmaR = Ck - P[0][0];
  if (sigmaR < 0)
    sigmaR = 0;

  if (fabs(sigmaR) > c_threshold)
  {
    R_angle = sigmaR + (R_angle - sigmaR) / sqrt(residual_index + 1.0);
  }

  // 6. Ganancia de Kalman
  float S = P[0][0] + R_angle;
  float K[2] = {P[0][0] / S, P[1][0] / S};

  // 7. Actualizar estado
  angle += K[0] * residual;
  bias += K[1] * residual;

  // 8. Actualizar covarianza
  float P00 = P[0][0];
  float P01 = P[0][1];

  P[0][0] -= K[0] * P00;
  P[0][1] -= K[0] * P01;
  P[1][0] -= K[1] * P00;
  P[1][1] -= K[1] * P01;

  // 9. Calcular factor de olvido para próxima iteración
  int l1 = window_size;
  int l2 = window_size / 2;
  float recent = 0, past = 0;

  for (int i = residual_index - l1; i < residual_index; i++)
  {
    if (i >= 0)
      recent += residual_history[i % window_size];
  }
  for (int i = residual_index - l1; i < residual_index - l2; i++)
  {
    if (i >= 0)
      past += residual_history[i % window_size];
  }

  lambda = (past != 0) ? recent / past : 1.0;
}

// Wrappers para roll y pitch
void kalmanUpdateRoll(float accAngleRoll, float gyroRateRoll)
{
  kalmanUpdate<window_size>(AngleRoll, x_roll[1], P_roll,
                            accAngleRoll, gyroRateRoll, // These should be the actual measurements
                            residual_history_roll, residual_index_roll,
                            R_angle_roll, lambda_roll);
}

void kalmanUpdatePitch(float accAnglePitch, float gyroRatePitch)
{
  kalmanUpdate<window_size>(AnglePitch, x_pitch[1], P_pitch,
                            accAnglePitch, gyroRatePitch, // These should be the actual measurements
                            residual_history_pitch, residual_index_pitch,
                            R_angle_pitch, lambda_pitch);
}

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

  // Aplicar el filtro de Kalman
  kalmanUpdateRoll(accAngleRoll, gyroRateRoll);
  kalmanUpdatePitch(accAnglePitch, gyroRatePitch);
}

void setupMPU()
{
  Serial.println("Iniciando sensores...");
  Wire.begin(SDA_MPU, SCL_MPU); // Ensure correct I2C pins are used
  Wire.setClock(400000);        // Set I2C clock speed to 400kHz
  accelgyro.initialize();
  delay(100); // Añade esto
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
