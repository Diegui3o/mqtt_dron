#include "motores.h"
#include <Arduino.h>
#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include <ESP32Servo.h>
#include <piloto_mode.h>
#include "variables.h"
#include <esp_task_wdt.h>
#include "mpu.h"

// Variable to track MPU calibration status
bool mpu_ready = false;

// === Matrices LQR ===
const float Ki_at[3][3] = {
    {0.90, 0, 0},
    {0, 0.90, 0},
    {0, 0, 0.162}};

const float Kc_at[3][6] = {
    {5.98, 0, 0, 3.57, 0, 0},
    {0, 5.99, 0, 0, 3.58, 0},
    {0, 0, 3.6, 0, 0, 1.60}};

// === Matrices LQR para altitud ===
const float Ki_alt = 31.6228;
const float Kc_alt[2] = {28.8910, 10.5624};

// === SETUP INICIAL ===
void setup_pilote_mode()
{
    pinMode(pinLed, OUTPUT);
    Serial.begin(115200);
    Serial.println("Iniciando modo pilote...");
    delay(100);
    Serial.println("Setup completado.");
}

void applyIKZControl(float tau_x, float tau_y, float tau_z, float dt);

void loop_pilote_mode(float dt)
{
    // 1. Actualizar el estado del dron antes de calcular el control
    updateDroneState();

    // 2. Obtener estados del sistema IKZ (reemplaza tus variables actuales)
    float x_c[6] = {
        AngleRoll,     // AngleRoll (φ)
        AnglePitch,    // AnglePitch (θ)
        AngleYaw,      // AngleYaw (ψ)
        gyroRateRoll,  // ω_x (del giroscopio)
        gyroRatePitch, // ω_y
        RateYaw        // ω_z
    };

    // 3. Calcular errores con referencias (ajusta según tu sistema)
    error_phi = phi_ref - x_c[0];
    error_theta = theta_ref - x_c[1];
    error_psi = psi_ref - x_c[2];

    // 4. Control LQR con compensación IKZ (usando τ_x, τ_y, τ_z)
    tau_x = Ki_at[0][0] * integral_phi + Kc_at[0][0] * error_phi - Kc_at[0][3] * x_c[3];
    tau_y = Ki_at[1][1] * integral_theta + Kc_at[1][1] * error_theta - Kc_at[1][4] * x_c[4];
    tau_z = Ki_at[2][2] * integral_psi + Kc_at[2][2] * error_psi - Kc_at[2][5] * x_c[5];

    // 5. Actualizar integrales (anti-windup opcional)
    integral_phi += error_phi * dt;
    integral_theta += error_theta * dt;
    integral_psi += error_psi * dt;

    InputThrottle = 1500; // Elimina esta línea

    // 6. Aplicar control solo si hay throttle suficiente
    if (InputThrottle > 1020)
    {
        applyIKZControl(tau_x, tau_y, tau_z, dt);
    }
    else
    {
        apagarMotores();
    }
}