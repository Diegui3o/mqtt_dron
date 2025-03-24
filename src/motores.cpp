#include <Arduino.h>
#include <ESP32Servo.h>
#include "motores.h"
#include "variables.h"

void setupMotores()
{
    // Asignar pines a los motores
    mot1.attach(mot1_pin);
    mot2.attach(mot2_pin);
    mot3.attach(mot3_pin);
    mot4.attach(mot4_pin);

    // Inicializar ESCs
    Serial.println("Iniciando ESCs...");
    mot1.writeMicroseconds(1000); // Valor mínimo para inicializar
    mot2.writeMicroseconds(1000);
    mot3.writeMicroseconds(1000);
    mot4.writeMicroseconds(1000);
    delay(2000); // Esperar a que los ESCs se inicialicen
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