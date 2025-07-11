#include "DHT.h"
#include <WiFi.h>
#include <WiFiClientSecure.h>
#define MQTT_VERSION MQTT_VERSION_3_1_1
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <config.h>

#define DHTPIN SENSOR_PIN
#define DHTTYPE DHT22

DHT dht(DHTPIN, DHTTYPE);
WiFiClientSecure espClient;
PubSubClient client(espClient);

// --- Forward Declarations ---
void mqtt_callback(char* topic, byte* payload, unsigned int length);
char* generate_auth_payload();
char* generate_heartbeat_update(bool disconnected = false);
char* generate_climate_update();

// Device Info
const char* device_name = DEVICE_NAME;
const char* device_id = DEVICE_UUID;
const int baud_rate = BAUD_RATE;
bool has_permission = false;

// MQTT
const char* mqtt_server = MQTT_SERVER;
const int mqtt_port = MQTT_PORT;
const char* mqtt_user = MQTT_USERNAME;
const char* mqtt_password = MQTT_PASSWORD;
const char* mqtt_events_topic = MQTT_EVENTS_TOPIC;
String mqtt_device_topic = String("guppy-") + device_id;
const int mqtt_keep_alive = 20;

// WiFi
const char* ssid = WIFI_SSID;
const char* password = WIFI_PASSWORD;

// Event Types
const char* device_sensor_readings = "device-sensor-readings";
const char* device_heartbeat = "device-heartbeat";
const char* device_auth = "device-auth";

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

struct AuthPayload {
  const char* device_callback;
};

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

void setup_mqtt() {
  espClient.setInsecure();
  client.setSocketTimeout(15);
  client.setBufferSize(512);
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(mqtt_callback);
  client.setKeepAlive(mqtt_keep_alive);
  connect_mqtt();
}

void connect_mqtt() {
  while (!client.connected()) {
    Serial.println("Connecting to MQTT...");
    if (client.connect(device_name, mqtt_user, mqtt_password, mqtt_events_topic, 0, true, generate_heartbeat_update(true))) {
      Serial.println("✅ Connected to MQTT broker");
      client.publish(mqtt_events_topic, generate_auth_payload());
      client.subscribe(mqtt_device_topic.c_str());
    } else {
      Serial.print("❌ Failed, rc=");
      Serial.println(client.state());
      delay(5000);
    }
  }

  Serial.println("⏳ Waiting for authentication...");

  unsigned long start = millis();
  const unsigned long timeout = 15000;

  while (!has_permission && millis() - start < timeout) {
    client.loop();
    delay(100);
  }

  if (has_permission) {
    Serial.println("🔓 Authentication confirmed.");
  } else {
    Serial.println("⛔ Authentication timed out. Disconnecting...");
    client.disconnect();
  }
}


void mqtt_callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("\n📩 MQTT Message Received\n");
  Serial.print("Topic: ");
  Serial.println(topic);
  Serial.print("Payload: ");

  // Convert payload to a null-terminated string
  char message[length + 1];
  memcpy(message, payload, length);
  message[length] = '\0';
  Serial.println(message);

  // Check payload content
  if (message[0] == '1') {
    Serial.println("✅ Device Authenticated!");
    has_permission = true;

    Serial.println("📤 Sending heartbeat...");
    client.publish(mqtt_events_topic, generate_heartbeat_update());
  } else {
    Serial.println("⚠️ Unknown payload. Authentication not granted.");
  }
}


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

String generate_timestamp() {
  struct tm timeinfo;
  if (getLocalTime(&timeinfo)) {
    char timestamp[30];
    strftime(timestamp, sizeof(timestamp), "%Y-%m-%dT%H:%M:%S", &timeinfo);
    return String(timestamp);
  }
  return "0000-00-00T00:00:00";
}

char* generate_auth_payload() {
  static char jsonBuffer[512];
  AuthPayload auth = {
    mqtt_device_topic.c_str()
  };

  String ts = generate_timestamp();
  JsonDocument doc;
  doc["event_type"] = device_auth;
  doc["device_id"] = device_id;
  doc["device_name"] = device_name;
  doc["timestamp"] = ts;

  JsonObject payload = doc["payload"].to<JsonObject>();
  payload["device_callback"] = auth.device_callback;

  serializeJson(doc, jsonBuffer);
  return jsonBuffer;
}

char* generate_heartbeat_update(bool disconnected) {
  static char jsonBuffer[512];
  HeartbeatUpdate heartbeat = {
    true,
    ESP.getFreeHeap(),
    WiFi.RSSI(),
    millis() / 1000
  };

  String ts = generate_timestamp();
  JsonDocument doc;
  doc["event_type"] = device_heartbeat;
  doc["device_id"] = device_id;
  doc["device_name"] = device_name;
  doc["timestamp"] = ts;

  JsonObject payload = doc["payload"].to<JsonObject>();
  payload["online"] = heartbeat.online;
  payload["free_memory"] = heartbeat.free_memory;
  payload["signal_strength"] = heartbeat.signal_strength;
  payload["uptime"] = heartbeat.uptime;
  payload["disconnected"] = disconnected;

  serializeJson(doc, jsonBuffer);
  return jsonBuffer;
}

char* generate_climate_update() {
  static char jsonBuffer[512];
  ClimateUpdate climate = {
    dht.readTemperature(true),
    dht.readHumidity()
  };

  if (isnan(climate.temperature) || isnan(climate.humidity)) {
    Serial.println("Failed to read from DHT sensor");
    return nullptr;
  }

  String ts = generate_timestamp();
  JsonDocument doc;
  doc["event_type"] = device_sensor_readings;
  doc["device_id"] = device_id;
  doc["device_name"] = device_name;
  doc["timestamp"] = ts;

  JsonObject payload = doc["payload"].to<JsonObject>();
  payload["temperature"] = climate.temperature;
  payload["humidity"] = climate.humidity;

  serializeJson(doc, jsonBuffer);
  return jsonBuffer;
}

void setup() {
  Serial.begin(baud_rate);
  dht.begin();
  setup_wifi();
  synchronize_time();
  setup_mqtt();
}

void loop() {
  if (!client.connected()) {
    connect_mqtt();
  }

  client.loop();

  if (has_permission) {
    static unsigned long lastSent = 0;
    unsigned long now = millis();

    if (now - lastSent >= 10000) {
      char* climate_payload = generate_climate_update();
      if (climate_payload != nullptr) {
        client.publish(mqtt_events_topic, climate_payload);
        Serial.println("📤 Published climate data");
      }
      lastSent = now;
    }
  }
}

