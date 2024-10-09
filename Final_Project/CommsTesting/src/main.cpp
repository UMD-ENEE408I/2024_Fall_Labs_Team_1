#include <Arduino.h>
#include <ArduinoJson.h>
#include <ArduinoWebsockets.h>
#include <WiFi.h>

#define WIFI_NETWORK "VisionSystem1116-2.4"

using namespace websockets;
WebsocketsClient client;

void request_image();
void request_audio();
void send_to_jetson();

StaticJsonDocument<100> doc;

void setup() {

  Serial.begin(115200);
  
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_NETWORK, NULL);

  if(WiFi.status() != WL_CONNECTED) {
    Serial.println("No Wifi!");
    return;
  }

  client.onMessage([&](WebsocketsMessage message){
        Serial.print("Got Message: ");
        Serial.println(message.data());
    });

  client.connect("ws://192.168.100.100:7755");

  if (!client.available()) {
        Serial.println("Failed to connect (websocket)...");
        Serial.flush();
        delay(1000);
        ESP.restart();
    }
    Serial.println("Connected to websocket");
}

void loop() {
  if(client.available()) 
  {
    client.poll();
  }
  delay(50);
}

void request_image() {
  doc.clear();
  doc["op"] = "image";

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
  serializeJson(doc, buff);
  client.send(buff);
}