#include "DHT.h"
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <config.h>

#define DHTPIN 2     // Pin connected to DATA pin of the DHT22
#define DHTTYPE DHT22   // DHT 22 (AM2302)

DHT dht(DHTPIN, DHTTYPE);
WiFiClient espClient;
PubSubClient client(espClient);

//Device Info
const char* device_name = DEVICE_NAME;
const char* device_uuid = DEVICE_UUID;

// MQTT Broker details
const char* mqtt_server = MQTT_SERVER;
const int mqtt_port = MQTT_PORT;
const char* mqtt_topic_heartbeat = "devices/heartbeat";
const char* mqtt_topic_climate = "devices/climate";
const int mqtt_keep_alive = 20; // 20 secs

// WiFi credentials
const char* ssid = WIFI_SSID;
const char* password = WIFI_PASSWORD;

struct HeartbeatUpdate {
  const char* device_name;
  const char* device_uuid;
  bool online;
  int free_memory;          // Free memory in bytes
  int signal_strength;      // Wi-Fi signal strength in dBm
  unsigned long uptime;     // Uptime in seconds
  String timestamp;         
};

struct ClimateUpdate {
  const char* device_name;
  const char* device_uuid;
  float temperature;
  float humidity;
  String timestamp;
};

void setup_wifi() {
  WiFi.begin(ssid, password);
  Serial.println("Connecting to WiFi");

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);

    Serial.print(".");
  }

  Serial.println("Connected to WiFi!");
}

void setup_mqtt() {
  client.setServer(mqtt_server, mqtt_port);
  client.setKeepAlive(mqtt_keep_alive);

  connect_mqtt();
}

void connect_mqtt() {
  while (!client.connected()) {
    Serial.println("Connecting to MQTT...\n");

    char last_will_message[512];
    HeartbeatUpdate last_will_update = {
      device_name,
      device_uuid,
      false,
      ESP.getFreeHeap(),
      WiFi.RSSI(),
      millis() / 1000,
      generate_timestamp()
    };

    // Serialize the struct into JSON
    StaticJsonDocument<200> jsonDoc;
    jsonDoc["device_name"] = last_will_update.device_name;
    jsonDoc["device_uuid"] = last_will_update.device_uuid;
    jsonDoc["online"] = last_will_update.online;
    jsonDoc["free_memory"] = last_will_update.free_memory;
    jsonDoc["signal_strength"] = last_will_update.signal_strength;
    jsonDoc["uptime"] = last_will_update.uptime;
    jsonDoc["timestamp"] = last_will_update.timestamp;

    serializeJson(jsonDoc, last_will_message);

    if (client.connect(device_name, mqtt_topic_heartbeat, 0, true, last_will_message)) {
      Serial.println("Connected to MQTT broker!");

      send_heartbeat_update();
    } 
    else {
      Serial.println("Failed to connect, rc=");
      Serial.println(client.state());

      delay(5000);  // Retry every 5 seconds
    }
  }
}

void synchronize_time() {
  // Synchronize time with NTP server
  configTime(0, 0, "pool.ntp.org", "time.nist.gov");
  Serial.println("Synchronizing time...");

  // Wait for the time to be set
  struct tm timeinfo;

  while (!getLocalTime(&timeinfo)) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\nTime synchronized");
}

void send_heartbeat_update() {
  char jsonBuffer[512];
  HeartbeatUpdate update = {
    device_name,
    device_uuid,
    true,
    ESP.getFreeHeap(),
    WiFi.RSSI(),
    millis() / 1000,
    generate_timestamp()
  };

  // Serialize the struct into JSON
  StaticJsonDocument<200> jsonDoc;
  jsonDoc["device_name"] = update.device_name;
  jsonDoc["device_uuid"] = update.device_uuid;
  jsonDoc["online"] = update.online;
  jsonDoc["free_memory"] = update.free_memory;
  jsonDoc["signal_strength"] = update.signal_strength;
  jsonDoc["uptime"] = update.uptime;
  jsonDoc["timestamp"] = update.timestamp;

  serializeJson(jsonDoc, jsonBuffer);

  // Publish to MQTT (assumes client setup already exists)
  if (client.publish(mqtt_topic_heartbeat, jsonBuffer)) {
    Serial.println("Heartbeat update sent:");
    Serial.println(jsonBuffer);
  } 
  else {
    Serial.println("Failed to send heartbeat update");
  }
}

void send_climate_update() {
  char jsonBuffer[512];
  ClimateUpdate update = {
    device_name,
    device_uuid,
    dht.readTemperature(true),
    dht.readHumidity(),
    generate_timestamp()
  };

  // Serialize the struct into JSON
  StaticJsonDocument<200> jsonDoc;
  jsonDoc["device_name"] = update.device_name;
  jsonDoc["device_uuid"] = update.device_uuid;
  jsonDoc["temperature"] = update.temperature;
  jsonDoc["humidity"] = update.humidity;
  jsonDoc["timestamp"] = update.timestamp;

  serializeJson(jsonDoc, jsonBuffer);

  // Publish to MQTT (assumes client setup already exists)
  if (client.publish(mqtt_topic_climate, jsonBuffer)) {
    Serial.println("Climate update sent:");
    Serial.println(jsonBuffer);
  } 
  else {
    Serial.println("Failed to send climate update");
  }
}

String generate_timestamp() {
  struct tm timeinfo;
  if (getLocalTime(&timeinfo)) {
    char timestamp[30];
    strftime(timestamp, sizeof(timestamp), "%Y-%m-%dT%H:%M:%S", &timeinfo);

    return String(timestamp);
  } 
  else {
    return "Failed to get time";
  }
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  dht.begin();
  setup_wifi();
  synchronize_time();
  setup_mqtt();
}

void loop() {
  // put your main code here, to run repeatedly:
  if(client.connected()) {
    send_climate_update();
    delay(10000);
  } else {
    connect_mqtt();
  }
}
