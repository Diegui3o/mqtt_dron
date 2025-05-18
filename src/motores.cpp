#include <Arduino.h>
#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include <ESP32Servo.h>
#include <piloto_mode.h>
#include "variables.h"
#include "mpu.h"

// Ajustar según ruido de tus sensores
#define ZUPT_THRESHOLD 0.15f // m/s
#define ZARUT_THRESHOLD 0.1f // rad/s

struct DroneState
{
    float angles[3];    // roll, pitch, yaw
    float gyroRates[3]; // ω_x, ω_y, ω_z
    float velocity[3];  // v_x, v_y, v_z
    float position[3];  // x, y, z
};

DroneState currentState;

void setupMotores()
{
    Wire.setClock(400000);
    Wire.begin();
    delay(250);
    Wire.beginTransmission(0x68);
    Wire.write(0x6B);
    Wire.write(0x00);
    Wire.endTransmission();

    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    ESP32PWM::allocateTimer(2);
    ESP32PWM::allocateTimer(3);

    mot1.attach(mot1_pin, 1000, 2000);
    mot2.attach(mot2_pin, 1000, 2000);
    mot3.attach(mot3_pin, 1000, 2000);
    mot4.attach(mot4_pin, 1000, 2000);
    delay(1000);

    mot1.writeMicroseconds(IDLE_PWM);
    mot2.writeMicroseconds(IDLE_PWM);
    mot3.writeMicroseconds(IDLE_PWM);
    mot4.writeMicroseconds(IDLE_PWM);
    delay(2000);

    mot1.attach(mot1_pin, 1000, 2000);
    delay(1000);
    mot1.setPeriodHertz(ESCfreq);
    delay(100);
    mot2.attach(mot2_pin, 1000, 2000);
    delay(1000);
    mot2.setPeriodHertz(ESCfreq);
    delay(100);
    mot3.attach(mot3_pin, 1000, 2000);
    delay(1000);
    mot3.setPeriodHertz(ESCfreq);
    delay(100);
    mot4.attach(mot4_pin, 1000, 2000);
    delay(1000);
    mot4.setPeriodHertz(ESCfreq);
    delay(100);

    mot1.writeMicroseconds(IDLE_PWM);
    mot2.writeMicroseconds(IDLE_PWM);
    mot3.writeMicroseconds(IDLE_PWM);
    mot4.writeMicroseconds(IDLE_PWM);
    delay(2000);
    Serial.println("Motores inicializados");
}

void apagarMotores()
{
    mot1.writeMicroseconds(1000); // Apagar los motores
    mot2.writeMicroseconds(1000);
    mot3.writeMicroseconds(1000);
    mot4.writeMicroseconds(1000);
}

void encenderMotores(int speed)
{
    mot1.writeMicroseconds(speed);
    mot2.writeMicroseconds(speed);
    mot3.writeMicroseconds(speed);
    mot4.writeMicroseconds(speed);
}

// === CONTROL A LOS MOTORES ===
void applyIKZControl(float tau_x, float tau_y, float tau_z, float dt)
{
    // 1. Aplicar ZUPT/ZARUT para corrección de errores
    if (sqrt(currentState.velocity[0] * currentState.velocity[0] +
             currentState.velocity[1] * currentState.velocity[1] +
             currentState.velocity[2] * currentState.velocity[2]) < ZUPT_THRESHOLD)
    {
        currentState.velocity[0] = 0;
        currentState.velocity[1] = 0;
        currentState.velocity[2] = 0;
    }

    if (sqrt(currentState.gyroRates[0] * currentState.gyroRates[0] +
             currentState.gyroRates[1] * currentState.gyroRates[1] +
             currentState.gyroRates[2] * currentState.gyroRates[2]) < ZARUT_THRESHOLD)
    {
        tau_x = 0;
        tau_y = 0;
        tau_z = 0;
    }

    // 2. Calcular PWM con mezcla X (adaptado a tu configuración)
    float pwm1 = InputThrottle - tau_x - tau_y + tau_z; // Motor delantero izquierdo
    float pwm2 = InputThrottle - tau_x + tau_y - tau_z; // Motor delantero derecho
    float pwm3 = InputThrottle + tau_x + tau_y + tau_z; // Motor trasero izquierdo
    float pwm4 = InputThrottle + tau_x - tau_y - tau_z; // Motor trasero derecho

    // 3. Limitar y enviar PWM
    MotorInput1 = constrain(pwm1, 1000, 2000);
    MotorInput2 = constrain(pwm2, 1000, 2000);
    MotorInput3 = constrain(pwm3, 1000, 2000);
    MotorInput4 = constrain(pwm4, 1000, 2000);

    mot1.writeMicroseconds(round(MotorInput1));
    mot2.writeMicroseconds(round(MotorInput2));
    mot3.writeMicroseconds(round(MotorInput3));
    mot4.writeMicroseconds(round(MotorInput4));
}

void applyControl(float tau_x, float tau_y, float tau_z)
{
    // Reutiliza la lógica de applyIKZControl pero sin ZUPT/ZARUT ni dt
    float pwm1 = InputThrottle - tau_x - tau_y + tau_z;
    float pwm2 = InputThrottle - tau_x + tau_y - tau_z;
    float pwm3 = InputThrottle + tau_x + tau_y + tau_z;
    float pwm4 = InputThrottle + tau_x - tau_y - tau_z;

    MotorInput1 = constrain(pwm1, 1000, 2000);
    MotorInput2 = constrain(pwm2, 1000, 2000);
    MotorInput3 = constrain(pwm3, 1000, 2000);
    MotorInput4 = constrain(pwm4, 1000, 2000);

    mot1.writeMicroseconds(round(MotorInput1));
    mot2.writeMicroseconds(round(MotorInput2));
    mot3.writeMicroseconds(round(MotorInput3));
    mot4.writeMicroseconds(round(MotorInput4));
}

void updateDroneState()
{
    // Ángulos (en grados o radianes según tu sistema)
    currentState.angles[0] = AngleRoll;
    currentState.angles[1] = AnglePitch;
    currentState.angles[2] = AngleYaw;

    // Velocidades angulares (en deg/s)
    currentState.gyroRates[0] = gyroRateRoll;
    currentState.gyroRates[1] = gyroRatePitch;
    currentState.gyroRates[2] = RateYaw;

    // Velocidades lineales (deberías calcularlas integrando la aceleración)
    // Ejemplo simple (debes mejorar con filtro y compensación de gravedad):
    static float lastAccX = 0, lastAccY = 0, lastAccZ = 0;
    float accX = AccX; // Aceleración en m/s^2
    float accY = AccY;
    float accZ = AccZ;
    float dt_local = dt; // Usa tu variable de tiempo real

    currentState.velocity[0] += 0.5f * (accX + lastAccX) * dt_local;
    currentState.velocity[1] += 0.5f * (accY + lastAccY) * dt_local;
    currentState.velocity[2] += 0.5f * (accZ + lastAccZ) * dt_local;

    lastAccX = accX;
    lastAccY = accY;
    lastAccZ = accZ;

    // Posición (opcional, si quieres estimar)
    currentState.position[0] += currentState.velocity[0] * dt_local;
    currentState.position[1] += currentState.velocity[1] * dt_local;
    currentState.position[2] += currentState.velocity[2] * dt_local;
}