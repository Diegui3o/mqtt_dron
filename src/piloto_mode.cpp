#include <Arduino.h>
#include <Wire.h>
<<<<<<< HEAD
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
    float error_phi = phi_ref - AngleRoll;
    float error_theta = theta_ref - AnglePitch;
    float error_psi = psi_ref - AngleYaw;

    // Actualizar términos integrales con anti-windup
    integral_phi = constrain(integral_phi + error_phi * DT, -50, 50);
    integral_theta = constrain(integral_theta + error_theta * DT, -50, 50);
    integral_psi = constrain(integral_psi + error_psi * DT, -50, 50);

    // Estado del sistema
    float x_c[6] = {AngleRoll, AnglePitch, AngleYaw, gyroRateRoll, gyroRatePitch, RateYaw};
    float x_i[3] = {integral_phi, integral_theta, integral_psi};

    // Calcular señales de control u = -Kc*x_c - Ki*x_i
    float tau_x = 0, tau_y = 0, tau_z = 0;

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

=======
#include <motores.h>
#include <ESP32Servo.h>
#include <VL53L0X.h>
#include "variables.h"
#include "mpu.h"
#include "piloto_mode.h"

int led_time = 100;

// Matrices de ganancia
float Ki_at[3][3] = {
    {2.5, 0, 0}, // Reducir las ganancias
    {0, 1.0, 0},
    {0, 0, 1.0}};

float Kc_at[3][6] = {
    {0.5, 0, 0, 0.05, 0, 0}, // Reducir las ganancias
    {0, 0.5, 0, 0, 0.05, 0},
    {0, 0, 0.5, 0, 0, 0.05}};

// Variables de estado
extern float phi_ref, theta_ref, psi_ref; // Ángulos de referencia

// Variables de control
extern float integral_phi, integral_theta, integral_psi;
float prev_error_phi = 0.0; // Error previo para el término derivativo

void setup_pilote_mode()
{
    Serial.begin(115200);
    Serial.println("Iniciando mode pilote...");
    Wire.begin();
    pinMode(pinLed, OUTPUT);
    accelgyro.initialize(); // Initialize the accelgyro
    calibrateSensors();     // Calibrar sensores
    setupMotores();         // Inicializar los motores
    Serial.println("Setup completado.");
}

void loop_pilote_mode()
{
    Serial.println("Inicio de loop_pilote_mode");
    // Calcular errores
    float error_phi = phi_ref - AngleRoll;

    // Limitar acumulación del término integral (evita integral windup)
    integral_phi = constrain(integral_phi + error_phi * 0.01, -50, 50);

    // Calcular el término derivativo
    float derivative_phi = (error_phi - prev_error_phi) / 0.01;
    prev_error_phi = error_phi;

    // Ley de control más rápida
    float tau_x = Ki_at[0][0] * integral_phi + Kc_at[0][0] * error_phi + Kc_at[0][3] * RateRoll + 0.1 * derivative_phi;

    // Actuación (conversión a PWM)
    applyControl(tau_x);
}

void calibrateSensors()
{
    // Calibrar sensores
    Serial.println("\nCalibrando sensores...");
    Serial.println("Coloca el MPU6050 en una superficie plana y no lo muevas durante la calibración.");
    digitalWrite(pinLed, HIGH);
    // Reiniciar offsets
>>>>>>> b1f65c3d56428ed493e4ce8dedd9ed69bd1c07f5
    accelgyro.setXAccelOffset(0);
    accelgyro.setYAccelOffset(0);
    accelgyro.setZAccelOffset(0);
    accelgyro.setXGyroOffset(0);
    accelgyro.setYGyroOffset(0);
    accelgyro.setZGyroOffset(0);

<<<<<<< HEAD
=======
    // Promediar lecturas del sensor
>>>>>>> b1f65c3d56428ed493e4ce8dedd9ed69bd1c07f5
    meansensors();
    Serial.println("\nCalculando offsets...");
    calibration();

<<<<<<< HEAD
=======
    // Aplicar offsets al sensor
>>>>>>> b1f65c3d56428ed493e4ce8dedd9ed69bd1c07f5
    accelgyro.setXAccelOffset(ax_offset);
    accelgyro.setYAccelOffset(ay_offset);
    accelgyro.setZAccelOffset(az_offset);
    accelgyro.setXGyroOffset(gx_offset);
    accelgyro.setYGyroOffset(gy_offset);
    accelgyro.setZGyroOffset(gz_offset);

<<<<<<< HEAD
    Serial.println("Calibración completada.");
=======
    Serial.println("\nCalibración completada.");
    Serial.print("Offsets calculados:\t");
    Serial.print(ax_offset);
    Serial.print("\t");
    Serial.print(ay_offset);
    Serial.print("\t");
    Serial.print(az_offset);
    Serial.print("\t");
    Serial.print(gx_offset);
    Serial.print("\t");
    Serial.print(gy_offset);
    Serial.print("\t");
    Serial.println(gz_offset);
>>>>>>> b1f65c3d56428ed493e4ce8dedd9ed69bd1c07f5
    digitalWrite(pinLed, LOW);
}

void meansensors()
{
    long i = 0, buff_ax = 0, buff_ay = 0, buff_az = 0, buff_gx = 0, buff_gy = 0, buff_gz = 0;
<<<<<<< HEAD
    while (i < (buffersize + 101))
    {
        accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
        if (i > 100 && i <= (buffersize + 100))
        {
=======

    while (i < (buffersize + 101))
    {
        // Leer datos del sensor
        accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

        if (i > 100 && i <= (buffersize + 100))
        { // Descartar las primeras 100 lecturas
>>>>>>> b1f65c3d56428ed493e4ce8dedd9ed69bd1c07f5
            buff_ax += ax;
            buff_ay += ay;
            buff_az += az;
            buff_gx += gx;
            buff_gy += gy;
            buff_gz += gz;
        }
        i++;
<<<<<<< HEAD
        delay(2);
    }

=======
        delay(2); // Esperar para evitar lecturas repetidas
    }

    // Calcular promedios
>>>>>>> b1f65c3d56428ed493e4ce8dedd9ed69bd1c07f5
    mean_ax = buff_ax / buffersize;
    mean_ay = buff_ay / buffersize;
    mean_az = buff_az / buffersize;
    mean_gx = buff_gx / buffersize;
    mean_gy = buff_gy / buffersize;
    mean_gz = buff_gz / buffersize;
}

void calibration()
{
<<<<<<< HEAD
=======
    // Calcular offsets
>>>>>>> b1f65c3d56428ed493e4ce8dedd9ed69bd1c07f5
    ax_offset = -mean_ax / 8;
    ay_offset = -mean_ay / 8;
    az_offset = (16384 - mean_az) / 8;

    gx_offset = -mean_gx / 4;
    gy_offset = -mean_gy / 4;
    gz_offset = -mean_gz / 4;

<<<<<<< HEAD
=======
    // Ajustar offsets hasta que estén dentro de la zona muerta
>>>>>>> b1f65c3d56428ed493e4ce8dedd9ed69bd1c07f5
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
<<<<<<< HEAD
            break;
    }
=======
            break; // Todos los offsets están dentro de la zona muerta
    }
}

void applyControl(float tau_x)
{
    int pwm1 = 1500 - tau_x; // Motor 1 (derecha)
    int pwm2 = 1500 - tau_x; // Motor 2 (derecha)
    int pwm3 = 1500 + tau_x; // Motor 3 (izquierda)
    int pwm4 = 1500 + tau_x; // Motor 4 (izquierda)

    // Ampliar el rango PWM
    pwm1 = constrain(pwm1, 1200, 1800);
    pwm2 = constrain(pwm2, 1200, 1800);
    pwm3 = constrain(pwm3, 1200, 1800);
    pwm4 = constrain(pwm4, 1200, 1800);

    mot1.writeMicroseconds(pwm1);
    mot2.writeMicroseconds(pwm2);
    mot3.writeMicroseconds(pwm3);
    mot4.writeMicroseconds(pwm4);
>>>>>>> b1f65c3d56428ed493e4ce8dedd9ed69bd1c07f5
}
