#include <Arduino.h>
#include <ArduinoJson.h>
#include <ArduinoWebsockets.h>
#include <WiFi.h>
#include "mbedtls/aes.h"

#define WIFI_NETWORK "enee408i"

using namespace websockets;
WebsocketsClient client;

void buzz(int t);

const unsigned int BUZZ = 26; // Check schematic and see pin connection to buzzer
const unsigned int BUZZ_CHANNEL = 0; //Selecting PWM channel 0
const unsigned int octave = 5;

char *encrypted;

void setup() {
  pinMode(14, OUTPUT);
  digitalWrite(14, LOW);

  Serial.begin(115200);
  
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_NETWORK, "enee408i");

  while (WiFi.status() != WL_CONNECTED) {
    Serial.println("No Wifi!");
    delay(500);
  }

  encrypted = (char *) malloc(24);

  client.onMessage([&](WebsocketsMessage message){
        Serial.print("Got Message: ");
        Serial.println(message.data());

        JsonDocument doc;
        deserializeJson(doc, message.data());

        if (strcmp(doc["op"], "encrypted_final") == 0) {

          memcpy(encrypted, doc["encrypted"], 8);

          mbedtls_aes_context aes;
          mbedtls_aes_init(&aes);
          mbedtls_aes_setkey_dec(&aes, (const unsigned char *) doc['key'], 128);

          char output[128];
          mbedtls_aes_crypt_ecb(&aes, MBEDTLS_AES_DECRYPT, (const unsigned char *) encrypted, (unsigned char *) output);

          for (int i = 0; i < output[5]; i ++)
          {
            buzz(500);
            delay(500);
          }
          
          mbedtls_aes_free( &aes );
          
        } else if (strcmp(doc["op"], "init_encrypt") == 0) {
          memcpy(encrypted + 8, doc["encrypted"], 16);
        }
    });

  client.connect("ws://172.20.10.5:7000");

  if (!client.available()) {
      Serial.println("Failed to connect (websocket)...");
      Serial.flush();
      free(encrypted);
      delay(1000);
      ESP.restart();
  }
  Serial.println("Connected to websocket");
}


void loop() {
  if(client.available()) {
        client.poll();
    }
    delay(500);
}

void buzz(int t)
{
  ledcAttachPin(BUZZ, BUZZ_CHANNEL);
  ledcWriteNote(BUZZ_CHANNEL, NOTE_C, octave);
  delay(t);
  noTone(BUZZ_CHANNEL);
}