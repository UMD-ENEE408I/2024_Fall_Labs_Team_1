#include <Arduino.h>
#include <ArduinoJson.h>
#include <ArduinoWebsockets.h>
#include <WiFi.h>

#define WIFI_NETWORK "iPhone (10)"

using namespace websockets;
WebsocketsClient client;

void request_image();
void request_audio();
void send_to_jetson();

StaticJsonDocument<100> doc;

void setup() {

  Serial.begin(115200);
  
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_NETWORK, "iluvabi123");

  while (WiFi.status() != WL_CONNECTED) {
    Serial.println("No Wifi!");
    delay(500);
  }

  client.onMessage([&](WebsocketsMessage message){
        Serial.print("Got Message: ");
        Serial.println(message.data());
    });

  client.connect("ws://172.20.10.5:7000");

  if (!client.available()) {
      Serial.println("Failed to connect (websocket)...");
      Serial.flush();
      delay(1000);
      ESP.restart();
  }
  Serial.println("Connected to websocket");

  delay(1000);
}

void loop() {
  request_image();
  delay(2000);
  request_audio();
  while(1);
}

void request_image() {
  doc.clear();
  doc["op"] = "image";
  doc["model"] = "model_name";

  send_to_jetson();
}

void request_audio() {
  doc.clear();
  doc["op"] = "audio";
  doc["duration"] = 5;

  send_to_jetson();
}

void send_to_jetson() {
  char buff[100];
  doc["name"] = "KW";
  serializeJson(doc, buff);
  client.send(buff);
}