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
#include <queue>
#include <esp_task_wdt.h>

// Variables compartidas entre núcleos
volatile bool ledState = false;
volatile bool motorState = false;
volatile int modoActual = 1;
volatile bool modoCambiado = false;

// Objetos globales
WiFiClient espClient;
PubSubClient client(espClient);
Preferences preferences;
TaskHandle_t Task1;

// Cola de mensajes MQTT
std::queue<String> mqttQueue;
unsigned long lastPublishTime = 0;
const int publishInterval = 30; // ms

// Prototipos de funciones
void setup_wifi();
void reconnect();
void callback(char *topic, byte *payload, unsigned int length);
void handleControlMessage(const String &message);
void handleModeMessage(const String &message);
void Task1code(void *pvParameters);
void prepareAndQueueMessages();
void processMQTTQueue();
void checkStack();

void setup_wifi()
{
    Serial.print("Conectando a ");
    Serial.println(ssid);
    WiFi.begin(ssid, password);
    WiFi.setSleep(true);
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\n✅ WiFi conectado. IP: " + WiFi.localIP().toString());
}

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
        }
        else
        {
            Serial.print("❌ Falló, rc=");
            Serial.println(client.state());
            delay(5000);
        }
    }
}

void callback(char *topic, byte *payload, unsigned int length)
{
    String message;
    for (unsigned int i = 0; i < length; i++)
    {
        message += (char)payload[i];
    }

    if (strcmp(topic, mqtt_control) == 0)
    {
        handleControlMessage(message);
    }
    else if (strcmp(topic, mqtt_topic_mode) == 0)
    {
        handleModeMessage(message);
    }
}

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

void handleModeMessage(const String &message)
{
    int nuevoModo;

    if (message.charAt(0) == '{')
    {
        JsonDocument doc;
        DeserializationError error = deserializeJson(doc, message);
        if (error)
        {
            Serial.println("⚠️ Error al analizar JSON");
            return;
        }
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
        preferences.putInt("modo", modoActual);
    }
}

void Task1code(void *pvParameters)
{
    for (;;)
    {
        digitalWrite(pinLed, ledState ? HIGH : LOW);
        motorState ? encenderMotores(1500) : apagarMotores();

        switch (modoActual)
        {
        case 0:
            if (modoCambiado)
            {
                setup_pilote_mode();
                modoCambiado = false;
            }
            loop_pilote_mode();
            break;
        case 1:
            if (modoCambiado)
                modoCambiado = false;
            break;
        case 2:
            if (modoCambiado)
                modoCambiado = false;
            break;
        default:
            if (modoCambiado)
                modoCambiado = false;
            break;
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

void prepareAndQueueMessages()
{
    // Publicar datos del MPU
    JsonDocument anglesDoc;
    anglesDoc["AngleRoll"] = AngleRoll_est;
    anglesDoc["AnglePitch"] = AnglePitch_est;
    anglesDoc["AngleYaw"] = AngleYaw;
    char anglesBuffer[200];
    size_t anglesLen = serializeJson(anglesDoc, anglesBuffer);
    client.publish(mqtt_topic_angles, anglesBuffer, anglesLen);

    // Publicar tasas
    JsonDocument ratesDoc;
    ratesDoc["RateRoll"] = gyroRateRoll;
    ratesDoc["RatePitch"] = gyroRatePitch;
    ratesDoc["RateYaw"] = RateYaw;
    char ratesBuffer[200];
    size_t ratesLen = serializeJson(ratesDoc, ratesBuffer);
    client.publish(mqtt_topic_rates, ratesBuffer, ratesLen);

    // Publicar aceleraciones
    JsonDocument accDoc;
    accDoc["AccX"] = AccX;
    accDoc["AccY"] = AccY;
    accDoc["AccZ"] = AccZ;
    char accBuffer[200];
    size_t accLen = serializeJson(accDoc, accBuffer);
    client.publish(mqtt_topic_acc, accBuffer, accLen);

    // Publicar giroscopio
    JsonDocument gyroDoc;
    gyroDoc["GyroXdps"] = GyroXdps;
    gyroDoc["GyroYdps"] = GyroYdps;
    gyroDoc["GyroZdps"] = GyroZdps;
    char gyroBuffer[200];
    size_t gyroLen = serializeJson(gyroDoc, gyroBuffer);
    client.publish(mqtt_topic_gyro, gyroBuffer, gyroLen);

    // Publicar ángulos de Kalman
    JsonDocument kalmanDoc;
    kalmanDoc["KalmanAngleRoll"] = AngleRoll;
    kalmanDoc["KalmanAnglePitch"] = AnglePitch;
    kalmanDoc["complementaryAngleRoll"] = complementaryAngleRoll;
    kalmanDoc["complementaryAnglePitch"] = complementaryAnglePitch;
    kalmanDoc["InputThrottle"] = error_phi;
    kalmanDoc["InputRoll"] = error_theta;
    kalmanDoc["InputPitch"] = tau_x;
    kalmanDoc["InputYaw"] = tau_y;
    char kalmanBuffer[800];
    size_t kalmanLen = serializeJson(kalmanDoc, kalmanBuffer);
    client.publish(mqtt_topic_kalman, kalmanBuffer, kalmanLen);

    // Publicar entradas de motores
    JsonDocument motorsDoc;
    motorsDoc["MotorInput1"] = MotorInput1;
    motorsDoc["MotorInput2"] = MotorInput2;
    motorsDoc["MotorInput3"] = MotorInput3;
    motorsDoc["MotorInput4"] = MotorInput4;
    char motorsBuffer[200];
    size_t motorsLen = serializeJson(motorsDoc, motorsBuffer);
    client.publish(mqtt_topic_motors, motorsBuffer, motorsLen);
}

void processMQTTQueue()
{
    if (!mqttQueue.empty() && client.connected())
    {
        String message = mqttQueue.front();
        int separatorPos = message.indexOf('|');
        String topic = message.substring(0, separatorPos);
        String payload = message.substring(separatorPos + 1);

        client.publish(topic.c_str(), payload.c_str(), false); // QoS 0
        mqttQueue.pop();
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
    setCpuFrequencyMhz(160);
    setupMPU();
    setup_wifi();
    btStop();
    client.setServer(mqttServer, mqtt_port);
    client.setCallback(callback);

    // Configurar watchdog
    esp_task_wdt_init(5, true);

    // Tarea en núcleo 1 con mayor prioridad
    xTaskCreatePinnedToCore(
        Task1code,
        "Task1",
        10000,
        NULL,
        2, // Prioridad aumentada
        &Task1,
        1);
}

void loop()
{
    unsigned long currentTime = millis();

    // 1. Gestionar conexión MQTT
    if (!client.connected())
    {
        reconnect();
    }
    client.loop();

    // 2. Procesar sensores
    gyro_signals();
    loopMPU();

    // 3. Publicar datos en intervalos regulares
    if (currentTime - lastPublishTime >= publishInterval)
    {
        lastPublishTime = currentTime;
        prepareAndQueueMessages();
    }

    // 4. Procesar cola MQTT
    processMQTTQueue();
}

void checkStack()
{
    Serial.printf("Stack libre (Core 0): %d\n", uxTaskGetStackHighWaterMark(NULL));
    Serial.printf("Stack libre (Core 1): %d\n", uxTaskGetStackHighWaterMark(Task1));
}
