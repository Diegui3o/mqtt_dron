#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <Preferences.h>
#include "motores.h"
#include "variables.h"
#include "mpu.h"
#include "manual_mode.h"
#include <ArduinoJson.h>
#include <data.h>

WiFiClient espClient;
PubSubClient client(espClient);
Preferences preferences;

int modo;
bool ledState;
int modoActual;

void setup_wifi();
void reconnect();
void callback(char *topic, byte *payload, unsigned int length);
void handleControlMessage(const String &message);
void handleModeMessage(const String &message);

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

void callback(char *topic, byte *payload, unsigned int length)
{
    Serial.print("📩 Mensaje recibido en ");
    Serial.print(topic);
    Serial.print(" -> ");

    String message;
    for (unsigned int i = 0; i < length; i++)
    {
        message += (char)payload[i]; // Construir el string correctamente
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

void handleControlMessage(const String &message)
{
    if (message == "ON_LED")
    {
        digitalWrite(pinLed, HIGH);
        ledState = true;
    }
    else if (message == "OFF_LED")
    {
        digitalWrite(pinLed, LOW);
        ledState = false;
    }
    preferences.putBool("ledState", ledState);
}

void handleModeMessage(const String &message)
{
    int nuevoModo;

    // Verificar si el mensaje es un JSON o solo un número
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

    if (nuevoModo == modoActual)
    {
        Serial.println("🔄 Modo recibido es el mismo que el actual.");
        return;
    }

    // Cambiar el modo y ejecutar la acción correspondiente
    modoActual = nuevoModo;
    Serial.printf("✅ Modo cambiado a: %d\n", modoActual);

    switch (modoActual)
    {
    case 0:
        Serial.println("🔴 Modo 0: Motores activados.");
        // Código para activar motores
        break;
    case 1:
        Serial.println("🟡 Modo 1: En espera.");
        // Código para estado de espera
        break;
    case 2:
        Serial.println("🟢 Modo 2: Motores apagados.");
        // Código para apagar motores
        break;
    default:
        Serial.println("⚠️ Modo desconocido.");
        break;
    }
}

void setup()
{
    Serial.begin(115200);
    pinMode(pinLed, OUTPUT);

    preferences.begin("dronData", false);
    modo = preferences.getInt("modo", 1); // Asegúrate de que el modo se restaura correctamente
    ledState = preferences.getBool("ledState", false);
    digitalWrite(pinLed, ledState ? HIGH : LOW);

    Serial.print("🔄 Estado restaurado: Modo = ");
    Serial.print(modo);
    Serial.print(", LED = ");
    Serial.println(ledState ? "ON" : "OFF");

    setupMPU();
    setup_wifi();
    client.setServer(mqttServer, mqtt_port);
    client.setCallback(callback);
}

void loop()
{
    mpu_signals(); // Obtener datos del sensor

    if (!client.connected())
    {
        reconnect();
    }
    client.loop();

    // Publicar datos del MPU
    StaticJsonDocument<200> anglesDoc;
    anglesDoc["AngleRoll"] = AngleRoll;
    anglesDoc["AnglePitch"] = AnglePitch;
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
    StaticJsonDocument<200> kalmanDoc;
    kalmanDoc["KalmanAngleRoll"] = KalmanAngleRoll;
    kalmanDoc["KalmanAnglePitch"] = KalmanAnglePitch;
    char kalmanBuffer[200];
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