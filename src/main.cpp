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

// ================= DEFINICIÓN DE MODOS =================
#define MODO_PILOTO_AUTOMATICO 0
#define MODO_ESPERA 1
#define MODO_MANUAL 2

// ================= VARIABLES GLOBALES =================
volatile int modoActual = MODO_ESPERA;
volatile bool modoCambiado = false;
volatile bool ledState = false;
volatile bool motorState = false;

WiFiClient espClient;
PubSubClient client(espClient);
Preferences preferences;
TaskHandle_t TaskControl;

unsigned long lastPublishTime = 0;
const int publishInterval = 30;
bool motoresHabilitados = false;
bool wifiConnected = false;
bool mqttConnected = false;
unsigned long lastReconnectAttempt = 0;
const unsigned long reconnectInterval = 5000;

// ================= FUNCIONES =================
void setup_wifi();
bool reconnectMQTT();
void callback(char *topic, byte *payload, unsigned int length);
void handleControlMessage(const String &message);
void handleModeMessage(const String &message);
void TaskControlCode(void *pvParameters);
void prepareAndPublishMessages();
void TaskGyroAndData(void *pvParameters);
void safeDelay(unsigned long ms);

void setup_wifi()
{
    Serial.print("Conectando a ");
    Serial.println(ssid);

    WiFi.disconnect(true);
    delay(100);
    WiFi.begin(ssid, password);
    WiFi.setSleep(false); // Deshabilitar sleep para mejor estabilidad

    unsigned long start = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - start < 10000)
    {
        delay(250);
        Serial.print(".");
    }

    if (WiFi.status() == WL_CONNECTED)
    {
        wifiConnected = true;
        Serial.println("\n✅ WiFi conectado. IP: " + WiFi.localIP().toString());
    }
    else
    {
        wifiConnected = false;
        Serial.println("\n❌ Fallo conexión WiFi");
    }
}

bool reconnectMQTT()
{
    if (!wifiConnected || WiFi.status() != WL_CONNECTED)
    {
        return false;
    }

    if (millis() - lastReconnectAttempt < reconnectInterval)
    {
        return false;
    }

    lastReconnectAttempt = millis();

    Serial.print("🔄 Intentando conexión MQTT...");
    if (client.connect("ESP32Client", NULL, NULL, "dron/status", 1, true, "offline"))
    {
        mqttConnected = true;
        Serial.println("✅ Conectado a MQTT");

        // Suscripciones con QoS 1 para mayor confiabilidad
        client.subscribe(mqtt_control, 1);
        client.subscribe(mqtt_topic_mode, 1);
        client.publish("dron/status", "online", true);
        return true;
    }
    else
    {
        mqttConnected = false;
        Serial.print("❌ Falló, rc=");
        Serial.println(client.state());
        return false;
    }
}

void callback(char *topic, byte *payload, unsigned int length)
{
    if (!mqttConnected)
        return;

    char *message = (char *)malloc(length + 1);
    if (message == NULL)
        return;

    memcpy(message, payload, length);
    message[length] = '\0';

    Serial.printf("📩 Mensaje recibido en el topic '%s': %s\n", topic, message);

    if (strcmp(topic, mqtt_control) == 0)
    {
        handleControlMessage(String(message));
    }
    else if (strcmp(topic, mqtt_topic_mode) == 0)
    {
        handleModeMessage(String(message));
    }
    else
    {
        Serial.println("⚠️ Mensaje recibido en un topic no manejado");
    }

    free(message);
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
    Serial.println("📩 Mensaje recibido en handleModeMessage: " + message);

    int nuevoModo = -1;

    if (message.charAt(0) == '{')
    {
        StaticJsonDocument<200> doc;
        DeserializationError error = deserializeJson(doc, message);
        if (!error)
        {
            nuevoModo = doc["modo"];
            Serial.printf("🔧 Modo extraído del JSON: %d\n", nuevoModo);
        }
        else
        {
            Serial.println("❌ Error al deserializar JSON");
        }
    }
    else
    {
        nuevoModo = message.toInt();
        Serial.printf("🔧 Modo extraído del mensaje: %d\n", nuevoModo);
    }

    if (nuevoModo >= 0 && nuevoModo <= 2 && nuevoModo != modoActual)
    {
        Serial.printf("🔄 Cambiando modo de %d a %d\n", modoActual, nuevoModo);
        modoActual = nuevoModo;
        modoCambiado = true;
        preferences.putInt("modo", modoActual);

        // Notificar cambio de modo
        String notificacion = "{\"modo\":" + String(modoActual) + ",\"status\":\"cambiado\"}";
        client.publish("dron/notificacion", notificacion.c_str());
    }
    else
    {
        Serial.println("⚠️ Modo no válido o sin cambios");
    }
}

void TaskControlCode(void *pvParameters)
{
    esp_task_wdt_add(NULL);
    int modoAnterior = -1;
    bool configuracionInicial = true;

    for (;;)
    {
        esp_task_wdt_reset();

        // Manejo de cambio de modo o primera configuración
        if (configuracionInicial || modoActual != modoAnterior)
        {
            Serial.printf("🔄 Detectado cambio de modo: %d -> %d\n", modoAnterior, modoActual);

            modoAnterior = modoActual;
            configuracionInicial = false;

            switch (modoActual)
            {
            case MODO_PILOTO_AUTOMATICO:
                Serial.println("⚙️ Configurando modo piloto automático...");
                setup_pilote_mode();
                Serial.println("✅ Modo piloto automático configurado");
                break;

            case MODO_ESPERA:
                Serial.println("⚙️ Configurando modo espera...");
                apagarMotores();
                Serial.println("🛑 Modo espera - Sistema en standby");
                break;

            case MODO_MANUAL:
                Serial.println("⚙️ Configurando modo manual...");
                setup_manual_mode();
                Serial.println("🎮 Modo manual configurado");
                break;

            default:
                Serial.println("⚠️ Modo desconocido");
                break;
            }
        }

        // Ejecución del modo actual
        static uint32_t last_time = micros();
        float dt = (micros() - last_time) / 1e6;

        if (dt >= 0.002)
        { // Control a 500Hz
            switch (modoActual)
            {
            case MODO_PILOTO_AUTOMATICO:
                loop_pilote_mode(dt);
                break;

            case MODO_MANUAL:
                loop_manual_mode(dt);
                break;

            case MODO_ESPERA:
                // No se requiere acción en modo espera
                break;
            }
            last_time = micros();
        }

        vTaskDelay(pdMS_TO_TICKS(1));
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
    gyroDoc["tau_x"] = tau_x;
    gyroDoc["tau_y"] = tau_y;
    gyroDoc["tau_z"] = tau_z;
    char gyroBuffer[200];
    size_t gyroLen = serializeJson(gyroDoc, gyroBuffer);
    client.publish(mqtt_topic_gyro, gyroBuffer, gyroLen);

    // Kalman
    JsonDocument kalmanDoc;
    kalmanDoc["KalmanAngleRoll"] = AngleRoll;
    kalmanDoc["KalmanAnglePitch"] = AnglePitch;
    kalmanDoc["error_phi"] = error_phi;
    kalmanDoc["error_theta"] = error_theta;
    kalmanDoc["InputThrottle"] = InputThrottle;
    kalmanDoc["InputRoll"] = DesiredAngleRoll;
    kalmanDoc["InputPitch"] = DesiredAnglePitch;
    kalmanDoc["InputYaw"] = DesiredRateYaw;
    char kalmanBuffer[800];
    size_t kalmanLen = serializeJson(kalmanDoc, kalmanBuffer);
    client.publish(mqtt_topic_kalman, kalmanBuffer, kalmanLen);

    // Motores
    JsonDocument motorsDoc;
    motorsDoc["MotorInput1"] = MotorInput1;
    motorsDoc["MotorInput2"] = MotorInput2;
    motorsDoc["MotorInput3"] = MotorInput3;
    motorsDoc["MotorInput4"] = MotorInput4;
    motorsDoc["Altura"] = T;
    char motorsBuffer[200];
    size_t motorsLen = serializeJson(motorsDoc, motorsBuffer);
    client.publish(mqtt_topic_motors, motorsBuffer, motorsLen);
}

void setup()
{
    Serial.begin(115200);
    while (!Serial)
        delay(10);

    pinMode(pinLed, OUTPUT);
    digitalWrite(pinLed, LOW);

    // Configuración inicial
    preferences.begin("dronData", false);
    modoActual = preferences.getInt("modo", MODO_ESPERA);
    ledState = preferences.getBool("ledState", false);
    digitalWrite(pinLed, ledState ? HIGH : LOW);

    // Optimización de frecuencia
    setCpuFrequencyMhz(240);

    // Inicialización segura de hardware
    setupMPU();
    setupMotores();

    // Configuración de red mejorada
    setup_wifi();
    btStop();
    client.setServer(mqttServer, mqtt_port);
    client.setCallback(callback);
    client.setBufferSize(2048);
    client.setSocketTimeout(15); // Timeout más corto

    // Configuración robusta del Watchdog
    esp_task_wdt_init(30, true);
    esp_task_wdt_add(NULL);

    // Creación de tareas con protección
    BaseType_t xReturned;
    xReturned = xTaskCreatePinnedToCore(
        TaskControlCode, "TaskControl",
        16384,
        NULL,
        2,
        &TaskControl,
        1);

    if (xReturned != pdPASS)
    {
        Serial.println("❌ Error creando TaskControl");
        while (1)
            delay(1000);
    }

    xReturned = xTaskCreatePinnedToCore(
        TaskGyroAndData, "TaskGyroAndData",
        12288,
        NULL,
        1,
        NULL,
        0);

    if (xReturned != pdPASS)
    {
        Serial.println("❌ Error creando TaskGyroAndData");
        while (1)
            delay(1000);
    }

    Serial.println("✅ Sistema inicializado correctamente");
}

void loop()
{
    esp_task_wdt_reset();

    // Manejo mejorado de reconexión
    if (!mqttConnected)
    {
        reconnectMQTT();
    }

    // Procesamiento MQTT con timeout
    unsigned long start = millis();
    while (mqttConnected && millis() - start < 10)
    {
        client.loop();
        delay(1);
    }

    safeDelay(5);
}

void safeDelay(unsigned long ms)
{
    unsigned long start = millis();
    while (millis() - start < ms)
    {
        delay(1);
        esp_task_wdt_reset();
    }
}

void TaskGyroAndData(void *pvParameters)
{
    esp_task_wdt_add(NULL); // Añadir esta tarea al WDT

    for (;;)
    {
        esp_task_wdt_reset(); // Resetear WDT también aquí

        gyro_signals(); // Leer sensores

        // Publicar datos con intervalo controlado
        static uint32_t lastPubTime = 0;
        if (millis() - lastPubTime >= publishInterval)
        {
            prepareAndPublishMessages();
            lastPubTime = millis();
        }

        vTaskDelay(pdMS_TO_TICKS(1));
    }
}