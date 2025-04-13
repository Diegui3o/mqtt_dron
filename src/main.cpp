#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <Preferences.h>
#include "motores.h"
#include "variables.h"
#include "mpu.h"
#include <data.h>
#include <Wire.h>
#include "piloto_mode.h"
#include <esp_task_wdt.h>
#include "manual_mode.h"

// ================= VARIABLES =================
volatile bool ledState = false;
volatile bool motorState = false;
volatile int modoActual = 1;
volatile bool modoCambiado = false;

WiFiClient espClient;
PubSubClient client(espClient);
Preferences preferences;
TaskHandle_t TaskControl;

unsigned long lastPublishTime = 0;
const int publishInterval = 30; // ms

// ================= FUNCIONES =================
void setup_wifi();
void reconnect();
void callback(char *topic, byte *payload, unsigned int length);
void handleControlMessage(const String &message);
void handleModeMessage(const String &message);
void TaskControlCode(void *pvParameters);
void prepareAndPublishMessages();

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
    int nuevoModo = -1;

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

    if (nuevoModo != modoActual && nuevoModo >= 0)
    {
        modoActual = nuevoModo;
        modoCambiado = true;
        preferences.putInt("modo", modoActual);
    }
}

void TaskControlCode(void *pvParameters)
{
    esp_task_wdt_add(NULL);
    for (;;)
    {
        esp_task_wdt_reset();

        digitalWrite(pinLed, ledState ? HIGH : LOW);
        motorState ? encenderMotores(1500) : apagarMotores();

        if (modoCambiado)
        {
            esp_task_wdt_reset();
            modoCambiado = false; // Resetear la bandera aquí
            switch (modoActual)
            {
            case 0:
                Serial.println("Modo piloto");
                setup_pilote_mode();
                break;
            case 1:
                Serial.println("Modo seguro");
                // Lógica del modo seguro
                break;
            case 2:
                Serial.println("Modo manual");
                setup_manual_mode();
                break;
            default:
                break;
            }
        }

        if (modoActual == 0)
        {
            loop_pilote_mode();
        }
        if (modoActual == 2)
        {
            loop_manual_mode();
        }

        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

void prepareAndPublishMessages()
{
    // Angles
    JsonDocument anglesDoc;
    anglesDoc["AngleRoll"] = AngleRoll_est;
    anglesDoc["AnglePitch"] = AnglePitch_est;
    anglesDoc["AngleYaw"] = AngleYaw;
    char anglesBuffer[200];
    size_t anglesLen = serializeJson(anglesDoc, anglesBuffer);
    client.publish(mqtt_topic_angles, anglesBuffer, anglesLen);

    // Rates
    JsonDocument ratesDoc;
    ratesDoc["RateRoll"] = gyroRateRoll;
    ratesDoc["RatePitch"] = gyroRatePitch;
    ratesDoc["RateYaw"] = RateYaw;
    char ratesBuffer[200];
    size_t ratesLen = serializeJson(ratesDoc, ratesBuffer);
    client.publish(mqtt_topic_rates, ratesBuffer, ratesLen);

    // Aceleración
    JsonDocument accDoc;
    accDoc["AccX"] = AccX;
    accDoc["AccY"] = AccY;
    accDoc["AccZ"] = AccZ;
    char accBuffer[200];
    size_t accLen = serializeJson(accDoc, accBuffer);
    client.publish(mqtt_topic_acc, accBuffer, accLen);

    // Giroscopio
    JsonDocument gyroDoc;
    gyroDoc["GyroXdps"] = GyroXdps;
    gyroDoc["GyroYdps"] = GyroYdps;
    gyroDoc["GyroZdps"] = GyroZdps;
    char gyroBuffer[200];
    size_t gyroLen = serializeJson(gyroDoc, gyroBuffer);
    client.publish(mqtt_topic_gyro, gyroBuffer, gyroLen);

    // Kalman
    JsonDocument kalmanDoc;
    kalmanDoc["KalmanAngleRoll"] = AngleRoll;
    kalmanDoc["KalmanAnglePitch"] = AnglePitch;
    kalmanDoc["complementaryAngleRoll"] = integral_phi;
    kalmanDoc["complementaryAnglePitch"] = integral_theta;
    kalmanDoc["InputThrottle"] = error_phi;
    kalmanDoc["InputRoll"] = error_theta;
    kalmanDoc["InputPitch"] = tau_x;
    kalmanDoc["InputYaw"] = tau_y;
    char kalmanBuffer[800];
    size_t kalmanLen = serializeJson(kalmanDoc, kalmanBuffer);
    client.publish(mqtt_topic_kalman, kalmanBuffer, kalmanLen);

    // Motores
    JsonDocument motorsDoc;
    motorsDoc["MotorInput1"] = MotorInput1;
    motorsDoc["MotorInput2"] = MotorInput2;
    motorsDoc["MotorInput3"] = MotorInput3;
    motorsDoc["MotorInput4"] = MotorInput4;
    char motorsBuffer[200];
    size_t motorsLen = serializeJson(motorsDoc, motorsBuffer);
    client.publish(mqtt_topic_motors, motorsBuffer, motorsLen);
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

    esp_task_wdt_init(10, true);
    esp_task_wdt_add(NULL);

    xTaskCreatePinnedToCore(
        TaskControlCode,
        "TaskControl",
        10000,
        NULL,
        1,
        &TaskControl,
        1); // Núcleo 1
}

void loop()
{
    esp_task_wdt_reset();

    if (!client.connected())
    {
        reconnect();
    }
    client.loop();

    gyro_signals();

    unsigned long currentTime = millis();
    if (currentTime - lastPublishTime >= publishInterval)
    {
        lastPublishTime = currentTime;
        prepareAndPublishMessages();
    }
}
