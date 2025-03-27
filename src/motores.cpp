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
    mot1.attach(mot1_pin);
    mot2.attach(mot2_pin);
    mot3.attach(mot3_pin);
    mot4.attach(mot4_pin);
    Serial.println("Iniciando ESCs...");
    delay(200);
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