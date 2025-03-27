#include <Arduino.h>
#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include <ESP32Servo.h>
#include <piloto_mode.h>
#include "variables.h"
#include "mpu.h"
#include "motores.h"

// === Matrices LQR ===
const float Ki_at[3][3] = {
    {17.3205, 0, 0},
    {0, 17.3205, 0},
    {0, 0, 3.873}};

const float Kc_at[3][6] = {
    {4.8651, 0, 0, 0.6804, 0, 0},
    {0, 4.8651, 0, 0, 0.6804, 0},
    {0, 0, 3.1383, 0, 0, 1.2069}};

// === SETUP INICIAL ===
void setup_pilote_mode()
{
    Serial.begin(115200);
    Serial.println("Iniciando modo pilote...");
    Wire.begin();
    pinMode(pinLed, OUTPUT);
    accelgyro.initialize();
    calibrateSensors();
    setupMotores();
    Serial.println("Setup completado.");
}

// === LOOP CON CONTROL LQR ===
void loop_pilote_mode()
{
    // === Calcular errores actuales ===
    error_phi = phi_ref - AngleRoll;
    error_theta = theta_ref - AnglePitch;
    error_psi = psi_ref - AngleYaw;

    // Actualizar términos integrales con anti-windup
    integral_phi = constrain(integral_phi + error_phi * DT, -50, 50);
    integral_theta = constrain(integral_theta + error_theta * DT, -50, 50);
    integral_psi = constrain(integral_psi + error_psi * DT, -50, 50);

    // Estado del sistema
    float x_c[6] = {AngleRoll, AnglePitch, AngleYaw, gyroRateRoll, gyroRatePitch, RateYaw};
    float x_i[3] = {integral_phi, integral_theta, integral_psi};

    // Calcular señales de control u = -Kc*x_c - Ki*x_i
    tau_x = 0, tau_y = 0, tau_z = 0;

    for (int j = 0; j < 6; j++)
    {
        tau_x = Ki_at[0][0] * integral_phi + Kc_at[0][0] * error_phi + Kc_at[0][3] * RateRoll;
        tau_y = Ki_at[1][1] * integral_theta + Kc_at[1][1] * error_theta + Kc_at[1][4] * RatePitch;
        tau_z = Ki_at[2][2] * integral_psi + Kc_at[2][2] * error_psi + Kc_at[2][5] * RateYaw;
    }

    tau_x -= Ki_at[0][0] * x_i[0];
    tau_y -= Ki_at[1][1] * x_i[1];
    tau_z -= Ki_at[2][2] * x_i[2];

    // Aplicar a los motores
    applyControl(tau_x, tau_y, tau_z);
}

// === CONTROL A LOS MOTORES ===
void applyControl(float tau_x, float tau_y, float tau_z)
{
    float pwm1 = 1500 - tau_x - tau_y - tau_z;
    float pwm2 = 1500 - tau_x + tau_y + tau_z;
    float pwm3 = 1500 + tau_x + tau_y - tau_z;
    float pwm4 = 1500 + tau_x - tau_y + tau_z;

    // Limitar valores PWM
    MotorInput1 = constrain(pwm1, 1000, 2000);
    MotorInput2 = constrain(pwm2, 1000, 2000);
    MotorInput3 = constrain(pwm3, 1000, 2000);
    MotorInput4 = constrain(pwm4, 1000, 2000);

    // Enviar señales
    mot1.writeMicroseconds(round(MotorInput1));
    mot2.writeMicroseconds(round(MotorInput2));
    mot3.writeMicroseconds(round(MotorInput3));
    mot4.writeMicroseconds(round(MotorInput4));
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
