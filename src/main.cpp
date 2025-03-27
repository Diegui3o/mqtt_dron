#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <Preferences.h>
#include "motores.h"
#include "variables.h"
#include "mpu.h"
#include "manual_mode.h"
#include <data.h>
#include <Wire.h>
#include "piloto_mode.h"

// Variables compartidas entre núcleos (volatile para sincronización)
volatile bool ledState = false;
volatile bool motorState = false;
volatile int modoActual = 1;
volatile bool modoCambiado = false;

// Objetos globales
WiFiClient espClient;
PubSubClient client(espClient);
Preferences preferences;
TaskHandle_t Task1;

// Prototipos de funciones
void setup_wifi();
void reconnect();
void callback(char *topic, byte *payload, unsigned int length);
void handleControlMessage(const String &message);
void handleModeMessage(const String &message);
void Task1code(void *pvParameters); // Función para el núcleo 1

// Configuración WiFi (igual que antes)
void setup_wifi()
{
    Serial.print("Conectando a ");
    Serial.println(ssid);
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\n✅ WiFi conectado. IP: " + WiFi.localIP().toString());
}

// Reconexión MQTT (igual que antes)
void reconnect()
{
    while (!client.connected())
    {
        Serial.print("🔄 Intentando conexión MQTT...");
        if (client.connect("ESP32Client"))
        {
            Serial.println("✅ Conectado a MQTT");
            client.subscribe(mqtt_control);
            client.subscribe(mqtt_topic_mode);
            Serial.println("📡 Suscrito a:");
            Serial.println(mqtt_control);
            Serial.println(mqtt_topic_mode);
        }
        else
        {
            Serial.print("❌ Falló, rc=");
            Serial.println(client.state());
            delay(5000);
        }
    }
}

// Callback MQTT (modificado para solo manejar mensajes)
void callback(char *topic, byte *payload, unsigned int length)
{
    Serial.print("📩 Mensaje recibido en ");
    Serial.print(topic);
    Serial.print(" -> ");

    String message;
    for (unsigned int i = 0; i < length; i++)
    {
        message += (char)payload[i];
    }
    Serial.println(message);

    if (strcmp(topic, mqtt_control) == 0)
    {
        handleControlMessage(message);
    }
    else if (strcmp(topic, mqtt_topic_mode) == 0)
    {
        handleModeMessage(message);
    }
}

// Manejo de mensajes de control (modificado para solo actualizar variables)
void handleControlMessage(const String &message)
{
    if (message == "ON_LED")
    {
        ledState = true;
    }
    else if (message == "OFF_LED")
    {
        ledState = false;
    }
    else if (message == "ON_MOTORS")
    {
        motorState = true;
    }
    else if (message == "OFF_MOTORS")
    {
        motorState = false;
    }
    preferences.putBool("ledState", ledState);
}

// Manejo de mensajes de modo (modificado para solo actualizar variables)
void handleModeMessage(const String &message)
{
    int nuevoModo;

    if (message.charAt(0) == '{')
    {
        DynamicJsonDocument doc(256);
        deserializeJson(doc, message);
        nuevoModo = doc["modo"];
    }
    else
    {
        nuevoModo = message.toInt();
    }

    if (nuevoModo != modoActual)
    {
        modoActual = nuevoModo;
        modoCambiado = true;
        Serial.printf("✅ Modo cambiado a: %d\n", modoActual);
    }
    else
    {
        Serial.println("🔄 Modo recibido es el mismo que el actual.");
    }
}

// Tarea que corre en el núcleo 1 (manejo de modos y actuadores)
void Task1code(void *pvParameters)
{
    Serial.print("Task1 running on core ");
    Serial.println(xPortGetCoreID());

    for (;;)
    {
        // Control del LED
        digitalWrite(pinLed, ledState ? HIGH : LOW);

        // Control de motores
        if (motorState)
        {
            encenderMotores(1500);
        }
        else
        {
            apagarMotores();
        }

        // Manejo de modos
        switch (modoActual)
        {
        case 0:
            if (modoCambiado)
            {
                Serial.println("🔴 Modo 0 - Piloto");
                setup_pilote_mode();
                modoCambiado = false;
            }
            loop_pilote_mode();
            break;
        case 1:
            if (modoCambiado)
            {
                Serial.println("🟡 Modo 1 - Espera");
                modoCambiado = false;
            }
            // Código para estado de espera
            break;
        case 2:
            if (modoCambiado)
            {
                Serial.println("🟢 Modo 2 - Apagado");
                modoCambiado = false;
            }
            // Código para apagar motores
            break;
        default:
            if (modoCambiado)
            {
                Serial.println("⚠️ Modo desconocido.");
                modoCambiado = false;
            }
            break;
        }

        delay(10); // Pequeña pausa para evitar saturación
    }
}

void setup()
{
    Serial.begin(115200);
    pinMode(pinLed, OUTPUT);

    preferences.begin("dronData", false);
    modoActual = preferences.getInt("modo", 1);
    ledState = preferences.getBool("ledState", false);
    digitalWrite(pinLed, ledState ? HIGH : LOW);

    Serial.print("🔄 Estado restaurado: Modo = ");
    Serial.print(modoActual);
    Serial.print(", LED = ");
    Serial.println(ledState ? "ON" : "OFF");

    setupMotores();
    setupMPU();
    setup_wifi();

    client.setServer(mqttServer, mqtt_port);
    client.setCallback(callback);

    // Crear tarea para el núcleo 1
    xTaskCreatePinnedToCore(
        Task1code, // Función de la tarea
        "Task1",   // Nombre de la tarea
        10000,     // Tamaño del stack
        NULL,      // Parámetros
        1,         // Prioridad
        &Task1,    // Handle de la tarea
        1          // Núcleo (1)
    );
}

void loop()
{
    gyro_signals();

    if (!client.connected())
    {
        reconnect();
    }
    client.loop();

    // Publicar datos del MPU
    StaticJsonDocument<200> anglesDoc;
    anglesDoc["AngleRoll"] = AngleRoll_est;
    anglesDoc["AnglePitch"] = AnglePitch_est;
    anglesDoc["AngleYaw"] = AngleYaw;
    char anglesBuffer[200];
    size_t anglesLen = serializeJson(anglesDoc, anglesBuffer);
    client.publish(mqtt_topic_angles, anglesBuffer, anglesLen);

    // Publicar tasas
    StaticJsonDocument<200> ratesDoc;
    ratesDoc["RateRoll"] = RateRoll;
    ratesDoc["RatePitch"] = RatePitch;
    ratesDoc["RateYaw"] = RateYaw;
    char ratesBuffer[200];
    size_t ratesLen = serializeJson(ratesDoc, ratesBuffer);
    client.publish(mqtt_topic_rates, ratesBuffer, ratesLen);

    // Publicar aceleraciones
    StaticJsonDocument<200> accDoc;
    accDoc["AccX"] = AccX;
    accDoc["AccY"] = AccY;
    accDoc["AccZ"] = AccZ;
    char accBuffer[200];
    size_t accLen = serializeJson(accDoc, accBuffer);
    client.publish(mqtt_topic_acc, accBuffer, accLen);

    // Publicar giroscopio
    StaticJsonDocument<200> gyroDoc;
    gyroDoc["GyroXdps"] = GyroXdps;
    gyroDoc["GyroYdps"] = GyroYdps;
    gyroDoc["GyroZdps"] = GyroZdps;
    char gyroBuffer[200];
    size_t gyroLen = serializeJson(gyroDoc, gyroBuffer);
    client.publish(mqtt_topic_gyro, gyroBuffer, gyroLen);

    // Publicar ángulos de Kalman
    StaticJsonDocument<800> kalmanDoc;
    kalmanDoc["KalmanAngleRoll"] = AngleRoll;
    kalmanDoc["KalmanAnglePitch"] = AnglePitch;
    kalmanDoc["complementaryAngleRoll"] = complementaryAngleRoll;
    kalmanDoc["complementaryAnglePitch"] = complementaryAnglePitch;
    kalmanDoc["InputThrottle"] = tau_x;
    kalmanDoc["InputRoll"] = tau_y;
    kalmanDoc["InputPitch"] = tau_z;
    kalmanDoc["InputYaw"] = error_phi;
    char kalmanBuffer[800];
    size_t kalmanLen = serializeJson(kalmanDoc, kalmanBuffer);
    client.publish(mqtt_topic_kalman, kalmanBuffer, kalmanLen);

    // Publicar entradas de motores
    StaticJsonDocument<200> motorsDoc;
    motorsDoc["MotorInput1"] = MotorInput1;
    motorsDoc["MotorInput2"] = MotorInput2;
    motorsDoc["MotorInput3"] = MotorInput3;
    motorsDoc["MotorInput4"] = MotorInput4;
    char motorsBuffer[200];
    size_t motorsLen = serializeJson(motorsDoc, motorsBuffer);
    client.publish(mqtt_topic_motors, motorsBuffer, motorsLen);

    delay(30); // Publicar cada 30ms
}