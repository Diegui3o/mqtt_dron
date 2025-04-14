#include <Arduino.h>
#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include <ESP32Servo.h>
#include <piloto_mode.h>
#include "variables.h"
#include <esp_task_wdt.h>
#include "mpu.h"
#include "motores.h"

// Define T as a global variable
float T = 0.0;

// Variable to track MPU calibration status
bool mpu_ready = false;

// === Matrices LQR ===
const float Ki_at[3][3] = {
    {19.1623, 0, 0},
    {0, 19.1623, 0},
    {0, 0, 3.87}};

const float Kc_at[3][6] = {
    {6.3882, 0, 0, 4.2, 0, 0},
    {0, 6.4022, 0, 0, 4.2, 0},
    {0, 0, 2.97864, 0, 0, 1.3182}};

// === Matrices LQR para altitud ===
const float Ki_alt = 31.6228;
const float Kc_alt[2] = {28.8910, 10.5624};

// === SETUP INICIAL ===
void setup_pilote_mode()
{
    pinMode(pinLed, OUTPUT);
    digitalWrite(pinLed, HIGH);
    delay(50);
    Serial.begin(115200);
    Serial.println("Iniciando modo pilote...");
    setupMotores();
    Serial.println("Setup completado.");
    digitalWrite(pinLed, LOW);
}

// === LOOP CON CONTROL LQR ===
void loop_pilote_mode()
{
    // Estado del sistema
    float x_c[6] = {AngleRoll, AnglePitch, AngleYaw, gyroRateRoll, gyroRatePitch, RateYaw};
    float x_i[3] = {integral_phi, integral_theta, integral_psi};

    tau_x = Ki_at[0][0] * x_i[0] + Kc_at[0][0] * error_phi - Kc_at[0][3] * x_c[3];
    tau_y = Ki_at[1][1] * x_i[1] + Kc_at[1][1] * error_theta - Kc_at[1][4] * x_c[4];
    tau_z = Ki_at[2][2] * x_i[2] + Kc_at[2][2] * error_psi - Kc_at[2][5] * x_c[5];

    error_phi = phi_ref - x_c[0];
    error_theta = theta_ref - x_c[1];
    error_psi = psi_ref - 0;

    tau_x -= Ki_at[0][0] * x_i[0];
    tau_y -= Ki_at[1][1] * x_i[1];
    tau_z -= Ki_at[2][2] * x_i[2];

    InputThrottle = T; // Empuje total calculado por el controlador de altitud
    applyControl(tau_x, tau_y, tau_z);
}

// === Loop de control de altitud ===
void controlAltitud()
{
    float x_alt[2] = {z, vel_z};
    float integral_z;

    // Cálculo del empuje total T (salida del LQR+I)
    T = Ki_alt * integral_z + Kc_alt[0] * error_z - Kc_alt[1] * x_alt[1];

    // Error
    error_z = z_ref - x_alt[0];

    // Saturación del empuje si es necesario
    if (T > 1700)
        T = 1700;
    if (T < 1000)
        T = 1000;
}
