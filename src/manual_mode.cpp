#include <Arduino.h>
#include <Wire.h>
#include <ESP32Servo.h>
#include "I2Cdev.h"
#include <VL53L0X.h>
#include "variables.h"
#include "mpu.h"
#include "manual_mode.h"
#include "motores.h"
// Modificaciones a manual_mode.cpp

#include "QuadcopterLQI.h"

// Añadir al principio del archivo
QuadcopterLQI* lqiController = nullptr;

void channelInterrupHandler()
{
  current_time = micros();
  if (digitalRead(channel_1_pin))
  {
    if (last_channel_1 == 0)
    {
      last_channel_1 = 1;
      timer_1 = current_time;
    }
  }
  else if (last_channel_1 == 1)
  {
    last_channel_1 = 0;
    ReceiverValue[0] = current_time - timer_1;
  }
  if (digitalRead(channel_2_pin))
  {
    if (last_channel_2 == 0)
    {
      last_channel_2 = 1;
      timer_2 = current_time;
    }
  }
  else if (last_channel_2 == 1)
  {
    last_channel_2 = 0;
    ReceiverValue[1] = current_time - timer_2;
  }
  if (digitalRead(channel_3_pin))
  {
    if (last_channel_3 == 0)
    {
      last_channel_3 = 1;
      timer_3 = current_time;
    }
  }
  else if (last_channel_3 == 1)
  {
    last_channel_3 = 0;
    ReceiverValue[2] = current_time - timer_3;
  }
  if (digitalRead(channel_4_pin))
  {
    if (last_channel_4 == 0)
    {
      last_channel_4 = 1;
      timer_4 = current_time;
    }
  }
  else if (last_channel_4 == 1)
  {
    last_channel_4 = 0;
    ReceiverValue[3] = current_time - timer_4;
  }
  if (digitalRead(channel_5_pin))
  {
    if (last_channel_5 == 0)
    {
      last_channel_5 = 1;
      timer_5 = current_time;
    }
  }
  else if (last_channel_5 == 1)
  {
    last_channel_5 = 0;
    ReceiverValue[4] = current_time - timer_5;
  }
  if (digitalRead(channel_6_pin))
  {
    if (last_channel_6 == 0)
    {
      last_channel_6 = 1;
      timer_6 = current_time;
    }
  }
  else if (last_channel_6 == 1)
  {
    last_channel_6 = 0;
    ReceiverValue[5] = current_time - timer_6;
  }
}

// Modificar setup_manual_mode()
void setup_manual_mode() {
    pinMode(pinLed, OUTPUT);
    digitalWrite(pinLed, HIGH);
    delay(50);
    Serial.begin(115200);
    Serial.println("Iniciando modo manual...");
    setupMotores();
    
    // Inicializar el controlador LQI
    if (lqiController == nullptr) {
        lqiController = new QuadcopterLQI(ControlMode::DECOUPLED);
        lqiController->init();
    }

    pinMode(channel_1_pin, INPUT_PULLUP);
    pinMode(channel_2_pin, INPUT_PULLUP);
    pinMode(channel_3_pin, INPUT_PULLUP);
    pinMode(channel_4_pin, INPUT_PULLUP);
    pinMode(channel_5_pin, INPUT_PULLUP);
    pinMode(channel_6_pin, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(channel_1_pin), channelInterrupHandler, CHANGE);
    attachInterrupt(digitalPinToInterrupt(channel_2_pin), channelInterrupHandler, CHANGE);
    attachInterrupt(digitalPinToInterrupt(channel_3_pin), channelInterrupHandler, CHANGE);
    attachInterrupt(digitalPinToInterrupt(channel_4_pin), channelInterrupHandler, CHANGE);
    attachInterrupt(digitalPinToInterrupt(channel_5_pin), channelInterrupHandler, CHANGE);
    attachInterrupt(digitalPinToInterrupt(channel_6_pin), channelInterrupHandler, CHANGE);

    Serial.println("Setup completado.");
    digitalWrite(pinLed, LOW);
}

// Modificar loop_manual_mode()
void loop_manual_mode() {
    // Leer entradas del receptor
    DesiredAngleRoll = 0.1 * (ReceiverValue[0] - 1500);
    DesiredAnglePitch = 0.1 * (ReceiverValue[1] - 1500);
    InputThrottle = ReceiverValue[2];
    DesiredRateYaw = 0.15 * (ReceiverValue[3] - 1500);
    
    // Establecer ángulos deseados en el controlador
    lqiController->setDesiredAttitude(DesiredAngleRoll, DesiredAnglePitch, DesiredRateYaw);
    
    // Calcular errores
    error_phi = DesiredAngleRoll - AngleRoll;
    error_theta = DesiredAnglePitch - AnglePitch;
    error_psi = DesiredRateYaw - RateYaw;
    
    // Construir vector de estado para LQI
    // El orden debe ser: [error_roll, error_pitch, error_yaw, rate_roll, rate_pitch, rate_yaw, integral_roll, integral_pitch, integral_yaw]
    float state[9] = {
        error_phi, error_theta, error_psi,  // Errores de ángulo
        gyroRateRoll, gyroRatePitch, RateYaw,  // Velocidades angulares
        integral_phi, integral_theta, integral_psi  // Estados integrales (se actualizarán dentro del controlador)
    };
    
    // Actualizar controlador LQI
    lqiController->update(state, dt);
    
    // Obtener señales de control
    float controlOutputs[3];
    lqiController->getControlOutputs(controlOutputs);
    
    // Extraer señales de control
    tau_x = controlOutputs[0];
    tau_y = controlOutputs[1];
    tau_z = controlOutputs[2];
    
    // Actualizar variables de integrador para uso en otras partes del código
    integral_phi = state[6];
    integral_theta = state[7];
    integral_psi = state[8];
    
    // Aplicar control a los motores
    applyControl(tau_x, tau_y, tau_z);
}