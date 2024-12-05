#include <ArduinoJson.h>
#include <ArduinoWebsockets.h>
#include <WiFi.h>

#define WIFI_NETWORK "enee408i"

using namespace websockets;
WebsocketsClient client;

void onMessageCallback(const WebsocketsMessage &message) {
  Serial.print("Got Message: ");
  Serial.println(message.data());

  JsonDocument doc_msg_callback;
  deserializeJson(doc_msg_callback, message.data());

}

uint8_t send_buff[500];

void audio_request()
{
  JsonDocument doc_begin;
  doc_begin["op"] = "begin";
  doc_begin["name"] = "BotKW";

  serializeJson(doc_begin, send_buff);
  client.send(reinterpret_cast<const char *>(send_buff));
}

void comms_loop()
{
  client.poll();
  delay(1);
}

void comms_begin()
{
  WiFi.begin(WIFI_NETWORK, WIFI_NETWORK);

  while (WiFi.status() != WL_CONNECTED) {
    Serial.println("No Wifi!");
    delay(500);
  }

  client.onMessage(onMessageCallback);

  client.connect("ws://192.168.1.251:7000");

  if (!client.available()) {
      Serial.println("Failed to connect (websocket)...");
      Serial.flush();
      delay(1000);
      ESP.restart();
  }
  Serial.println("Connected to websocket");

  delay(500);

  JsonDocument doc_begin;
  doc_begin["op"] = "begin";
  doc_begin["name"] = "BotKW";

  serializeJson(doc_begin, send_buff);
  client.send(reinterpret_cast<const char *>(send_buff));

  delay(500);
}