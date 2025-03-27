#include <Arduino.h>
#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include <ESP32Servo.h>
#include <piloto_mode.h>
#include "variables.h"
#include "mpu.h"

void setupMotores()
{
    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    ESP32PWM::allocateTimer(2);
    ESP32PWM::allocateTimer(3);

    mot1.setPeriodHertz(400); // Frecuencia PWM para ESCs
    mot2.setPeriodHertz(400);
    mot3.setPeriodHertz(400);
    mot4.setPeriodHertz(400);

    mot1.attach(mot1_pin, 1000, 2000);
    mot2.attach(mot2_pin, 1000, 2000);
    mot3.attach(mot3_pin, 1000, 2000);
    mot4.attach(mot4_pin, 1000, 2000);

    // Inicializar ESCs
    mot1.writeMicroseconds(IDLE_PWM);
    mot2.writeMicroseconds(IDLE_PWM);
    mot3.writeMicroseconds(IDLE_PWM);
    mot4.writeMicroseconds(IDLE_PWM);
    delay(2000);
    Serial.println("Motores inicializados");
}

void encenderMotores(int speed)
{
    mot1.writeMicroseconds(speed);
    mot2.writeMicroseconds(speed);
    mot3.writeMicroseconds(speed);
    mot4.writeMicroseconds(speed);
}

void apagarMotores()
{
    mot1.writeMicroseconds(1000); // Apagar los motores
    mot2.writeMicroseconds(1000);
    mot3.writeMicroseconds(1000);
    mot4.writeMicroseconds(1000);
}

void loopMotores()
{
    // Código de loop para motores (si es necesario)
}