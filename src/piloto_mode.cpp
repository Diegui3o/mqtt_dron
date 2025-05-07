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
#include "QuadcopterLQI.h"

// Declaración de la variable global
unsigned long lastMicros = 0; // Inicializar con 0

QuadcopterLQI *lqiControllerPilot = nullptr;

void setup_pilote_mode()
{
    pinMode(pinLed, OUTPUT);
    digitalWrite(pinLed, HIGH);
    delay(50);
    Serial.begin(115200);
    Serial.println("Iniciando modo piloto...");
    setupMotores();

    if (lqiControllerPilot == nullptr)
    {
        lqiControllerPilot = new QuadcopterLQI(ControlMode::MIMO);
        lqiControllerPilot->init();
    }

    Serial.println("Setup completado.");
    digitalWrite(pinLed, LOW);

    // Inicializar tiempo para dt
    lastMicros = micros();
}

void loop_pilote_mode()
{
    double now = micros();
    double dt = (now - lastMicros) / 1e6;
    lastMicros = now;
    if (dt <= 0.0 || dt > 0.2)
    {
        dt = 0.01; // valor por defecto de 10 ms
    }

    // Calcular errores
    error_phi = phi_ref - AngleRoll;
    error_theta = theta_ref - AnglePitch;
    error_psi = psi_ref - AngleYaw;

    float state[9] = {
        error_phi, error_theta, error_psi,
        gyroRateRoll, gyroRatePitch, RateYaw,
        integral_phi, integral_theta, integral_psi};

    lqiControllerPilot->update(state, dt);

    float controlOutputs[3];
    lqiControllerPilot->getControlOutputs(controlOutputs);

    tau_x = controlOutputs[0];
    tau_y = controlOutputs[1];
    tau_z = controlOutputs[2];

    integral_phi = state[6];
    integral_theta = state[7];
    integral_psi = state[8];

    InputThrottle = 1500;
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