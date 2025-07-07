#include "DHT.h"
#include <WiFi.h>
#include <WiFiClientSecure.h>
#define MQTT_VERSION MQTT_VERSION_3_1_1 // needed in order to have successful connection with HiveMQ via port 8883
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <config.h>

#define DHTPIN SENSOR_PIN
#define DHTTYPE DHT22

DHT dht(DHTPIN, DHTTYPE);
WiFiClientSecure espClient;
PubSubClient client(espClient);

const int baud_rate = BAUD_RATE;

// Device Info
const char* device_name = DEVICE_NAME;
const char* device_id = DEVICE_UUID;

// MQTT
const char* mqtt_server = MQTT_SERVER;
const int mqtt_port = MQTT_PORT;
const char* mqtt_user = MQTT_USERNAME;
const char* mqtt_password = MQTT_PASSWORD;
const char* mqtt_topic = MQTT_TOPIC;
const int mqtt_keep_alive = 20;

// WiFi
const char* ssid = WIFI_SSID;
const char* password = WIFI_PASSWORD;

struct HeartbeatUpdate {
  bool online;
  uint32_t free_memory;
  int signal_strength;
  unsigned long uptime;
};

struct ClimateUpdate {
  float temperature;
  float humidity;
};

// === Setup WiFi ===
void setup_wifi() {
  Serial.println(ssid);
  Serial.println(password);

  WiFi.begin(ssid, password);
  Serial.println("Connecting to WiFi...");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected to WiFi!");
}

// === Connect MQTT ===
void connect_mqtt() {
  while (!client.connected()) {
    Serial.println("Connecting to MQTT...");

    char will_message[512];
    HeartbeatUpdate last_will = {
      false,
      ESP.getFreeHeap(),
      WiFi.RSSI(),
      millis() / 1000
    };

    String ts = generate_timestamp();
    JsonDocument doc;
    doc["event_type"] = "device-disconnected";
    doc["device_id"] = device_id;
    doc["device_name"] = device_name;
    doc["timestamp"] = ts;

    JsonObject payload = doc["payload"].to<JsonObject>();
    payload["online"] = last_will.online;
    payload["free_memory"] = last_will.free_memory;
    payload["signal_strength"] = last_will.signal_strength;
    payload["uptime"] = last_will.uptime;

    serializeJson(doc, will_message);

    if (client.connect(device_name, mqtt_user, mqtt_password, mqtt_topic, 0, true, will_message)) {
      Serial.println("Connected to MQTT broker");
      send_heartbeat_update();
    } else {
      Serial.print("Failed, rc=");
      Serial.println(client.state());
      delay(5000);
    }
  }
}

// === Setup MQTT ===
void setup_mqtt() {
  espClient.setInsecure();
  client.setSocketTimeout(15);
  client.setBufferSize(512);   
  client.setServer(mqtt_server, mqtt_port);
  client.setKeepAlive(mqtt_keep_alive);
  connect_mqtt();
}

// === NTP Sync ===
void synchronize_time() {
  configTime(0, 0, "pool.ntp.org", "time.nist.gov");
  Serial.println("Synchronizing time...");
  struct tm timeinfo;
  while (!getLocalTime(&timeinfo)) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nTime synchronized");
}

// === Timestamp Generator ===
String generate_timestamp() {
  struct tm timeinfo;
  if (getLocalTime(&timeinfo)) {
    char timestamp[30];
    strftime(timestamp, sizeof(timestamp), "%Y-%m-%dT%H:%M:%S", &timeinfo);
    return String(timestamp);
  }
  return "0000-00-00T00:00:00";
}

// === Send Heartbeat ===
void send_heartbeat_update() {
  char jsonBuffer[512];
  HeartbeatUpdate hb = {
    true,
    ESP.getFreeHeap(),
    WiFi.RSSI(),
    millis() / 1000
  };

  String ts = generate_timestamp();
  JsonDocument doc;
  doc["event_type"] = "device-heartbeat";
  doc["device_id"] = device_id;
  doc["device_name"] = device_name;
  doc["timestamp"] = ts;

  JsonObject payload = doc["payload"].to<JsonObject>();
  payload["online"] = hb.online;
  payload["free_memory"] = hb.free_memory;
  payload["signal_strength"] = hb.signal_strength;
  payload["uptime"] = hb.uptime;

  serializeJson(doc, jsonBuffer);

  if (client.publish(mqtt_topic, jsonBuffer)) {
    Serial.println("Heartbeat update sent:");
    Serial.println(jsonBuffer);
  } else {
    Serial.println("Failed to send heartbeat update");
  }
}

// === Send Climate Reading ===
void send_climate_update() {
  char jsonBuffer[512];
  float temp = dht.readTemperature(true);
  float hum = dht.readHumidity();

  if (isnan(temp) || isnan(hum)) {
    Serial.println("Failed to read from DHT sensor");
    return;
  }

  String ts = generate_timestamp();
  JsonDocument doc;
  doc["event_type"] = "device-sensor-readings";
  doc["device_id"] = device_id;
  doc["device_name"] = device_name;
  doc["timestamp"] = ts;

  JsonObject payload = doc["payload"].to<JsonObject>();
  payload["temperature"] = temp;
  payload["humidity"] = hum;

  serializeJson(doc, jsonBuffer);

  if (client.publish(mqtt_topic, jsonBuffer)) {
    Serial.println("Climate update sent:");
    Serial.println(jsonBuffer);
  } else {
    Serial.println("Failed to send climate update");
  }
}

// === Main Setup ===
void setup() {
  Serial.begin(baud_rate);
  dht.begin();
  setup_wifi();
  synchronize_time();
  setup_mqtt();
}

// === Main Loop ===
void loop() {
  if (client.connected()) {
    send_climate_update();
    delay(10000);
  } else {
    connect_mqtt();
  }
}
