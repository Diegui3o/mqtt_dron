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

    InputThrottle = 1000; // Valor de referencia para el acelerador

    // Aplicar a los motores
    applyControl(tau_x, tau_y, tau_z);
}

// === CALIBRACIÓN DEL MPU6050 ===
void calibrateSensors()
{
    Serial.println("\nCalibrando sensores...");
    digitalWrite(pinLed, HIGH);

    accelgyro.setXAccelOffset(0);
    accelgyro.setYAccelOffset(0);
    accelgyro.setZAccelOffset(0);
    accelgyro.setXGyroOffset(0);
    accelgyro.setYGyroOffset(0);
    accelgyro.setZGyroOffset(0);

    meansensors();
    Serial.println("\nCalculando offsets...");
    calibration();

    accelgyro.setXAccelOffset(ax_offset);
    accelgyro.setYAccelOffset(ay_offset);
    accelgyro.setZAccelOffset(az_offset);
    accelgyro.setXGyroOffset(gx_offset);
    accelgyro.setYGyroOffset(gy_offset);
    accelgyro.setZGyroOffset(gz_offset);

    Serial.println("Calibración completada.");
    digitalWrite(pinLed, LOW);
}

void meansensors()
{
    long i = 0, buff_ax = 0, buff_ay = 0, buff_az = 0, buff_gx = 0, buff_gy = 0, buff_gz = 0;
    while (i < (buffersize + 101))
    {
        accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
        if (i > 100 && i <= (buffersize + 100))
        {
            buff_ax += ax;
            buff_ay += ay;
            buff_az += az;
            buff_gx += gx;
            buff_gy += gy;
            buff_gz += gz;
        }
        i++;
        delay(2);
    }

    mean_ax = buff_ax / buffersize;
    mean_ay = buff_ay / buffersize;
    mean_az = buff_az / buffersize;
    mean_gx = buff_gx / buffersize;
    mean_gy = buff_gy / buffersize;
    mean_gz = buff_gz / buffersize;
}

void calibration()
{
    ax_offset = -mean_ax / 8;
    ay_offset = -mean_ay / 8;
    az_offset = (16384 - mean_az) / 8;

    gx_offset = -mean_gx / 4;
    gy_offset = -mean_gy / 4;
    gz_offset = -mean_gz / 4;

    while (1)
    {
        int ready = 0;
        accelgyro.setXAccelOffset(ax_offset);
        accelgyro.setYAccelOffset(ay_offset);
        accelgyro.setZAccelOffset(az_offset);
        accelgyro.setXGyroOffset(gx_offset);
        accelgyro.setYGyroOffset(gy_offset);
        accelgyro.setZGyroOffset(gz_offset);

        meansensors();

        if (abs(mean_ax) <= acel_deadzone)
            ready++;
        else
            ax_offset -= mean_ax / acel_deadzone;

        if (abs(mean_ay) <= acel_deadzone)
            ready++;
        else
            ay_offset -= mean_ay / acel_deadzone;

        if (abs(16384 - mean_az) <= acel_deadzone)
            ready++;
        else
            az_offset += (16384 - mean_az) / acel_deadzone;

        if (abs(mean_gx) <= giro_deadzone)
            ready++;
        else
            gx_offset -= mean_gx / (giro_deadzone + 1);

        if (abs(mean_gy) <= giro_deadzone)
            ready++;
        else
            gy_offset -= mean_gy / (giro_deadzone + 1);

        if (abs(mean_gz) <= giro_deadzone)
            ready++;
        else
            gz_offset -= mean_gz / (giro_deadzone + 1);

        if (ready == 6)
            break;
    }
}