/*
  Water Flow Monitor (ESP32 + MQTT)
  Measures water usage via YF-S201 sensor and sends data to MQTT broker.
  LED indicates when total consumption exceeds a defined limit.
*/

#include <ESP8266WiFi.h>
#include <PubSubClient.h>

// Wi-Fi
const char* WIFI_SSID     = "SSID";
const char* WIFI_PASSWORD = "PASSWORD";

// MQTT
const char* MQTT_BROKER   = "mqtt.eclipseprojects.io";
const int   MQTT_PORT     = 1883;
const char* MQTT_CLIENT_ID = "waterFlowMonitor";

// MQTT Topics
#define TOPIC_SUB_LIMIT      "waterFlowMonitor/limit"
#define TOPIC_PUB_FLOW_RATE  "waterFlowMonitor/flowRate"
#define TOPIC_PUB_ML         "waterFlowMonitor/flowML"
#define TOPIC_PUB_TOTAL_ML   "waterFlowMonitor/totalML"

// Pins
#define SENSOR_PIN 27
#define LED_PIN    17

WiFiClient wifiClient;
PubSubClient MQTT(wifiClient);

// Flow data
volatile unsigned int pulseCount = 0;
float flowRate = 0.0;
unsigned int flowML = 0;
unsigned long totalML = 0;
unsigned long flowLimit = 0;
unsigned long lastTime = 0;

// Count sensor pulses
void IRAM_ATTR countPulse() {
    pulseCount++;
}

void connectWiFi() {
    if (WiFi.status() == WL_CONNECTED) {
        return;
    }
    Serial.print("Connecting to Wi-Fi: ");
    Serial.println(WIFI_SSID);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    while (WiFi.status() != WL_CONNECTED) {
        delay(100);
        Serial.print(".");
    }
    Serial.println("\nWiFi connected: " + WiFi.localIP().toString());
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
    String message = "";
    for (unsigned int i = 0; i < length; i++) {
        message += (char)payload[i];
    }
    if (String(topic) == TOPIC_SUB_LIMIT) {
        flowLimit = message.toInt();
        Serial.println("New limit: " + String(flowLimit) + " mL");
    }
}

void connectMQTT() {
    while (!MQTT.connected()) {
        Serial.print("Connecting to MQTT broker...");
        if (MQTT.connect(MQTT_CLIENT_ID)) {
            Serial.println(" connected.");
            MQTT.subscribe(TOPIC_SUB_LIMIT);
        } else {
            Serial.println(" failed. Retrying in 2s...");
            delay(2000);
        }
    }
}

void setup() {
    Serial.begin(115200);

    pinMode(SENSOR_PIN, INPUT_PULLUP);
    pinMode(LED_PIN, OUTPUT);

    attachInterrupt(digitalPinToInterrupt(SENSOR_PIN), countPulse, FALLING);

    connectWiFi();
    MQTT.setServer(MQTT_BROKER, MQTT_PORT);
    MQTT.setCallback(mqttCallback);
}

void loop() {
    if (!MQTT.connected()) {
        connectMQTT();
    }
    connectWiFi();

    unsigned long now = millis();

    if (now - lastTime > 1000) {

        flowRate = (1000.0 / (now - lastTime)) * pulseCount;

        pulseCount = 0;
        lastTime = now;

        flowML = (flowRate / 60) * 1000;

        totalML += flowML;

        digitalWrite(LED_PIN, totalML > flowLimit ? HIGH : LOW);

        Serial.printf("Flow: %.2f L/min, %d mL/s, Total: %lu mL\n", flowRate, flowML, totalML);

        MQTT.publish(TOPIC_PUB_FLOW_RATE, String(flowRate, 2).c_str());
        MQTT.publish(TOPIC_PUB_ML, String(flowML).c_str());
        MQTT.publish(TOPIC_PUB_TOTAL_ML, String(totalML).c_str());
    }

    MQTT.loop();
}
