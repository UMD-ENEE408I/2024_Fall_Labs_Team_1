#include <Arduino.h>
#include <ArduinoJson.h>
#include <ArduinoWebsockets.h>
#include <WiFi.h>
#include "mbedtls/aes.h"
#include "mbedtls/base64.h"

#define WIFI_NETWORK "VisionSystem1120-2.4"

using namespace websockets;
WebsocketsClient client;

void buzz(int t);

const unsigned int BUZZ = 26; // Check schematic and see pin connection to buzzer
const unsigned int BUZZ_CHANNEL = 0; //Selecting PWM channel 0
const unsigned int octave = 5;

uint8_t encrypted[16*4];
uint8_t send_buff[500];

size_t convert_hex(uint8_t *dest, size_t count, const char *src) {
    size_t i;
    int value;
    for (i = 0; i < count && sscanf(src + i * 2, "%2x", &value) == 1; i++) {
        dest[i] = value;
    }
    return i;
}

String hexToASCII(String hex)
{
    // initialize the ASCII code string as empty.
    String ascii = "";
    for (size_t i = 0; i < hex.length(); i += 2)
    {
        String part = hex.substring(i, i+2);
        char ch = strtoul(part.c_str(), nullptr, 16); 
        ascii += ch;
    }
    return ascii;
}

void onMessageCallback(const WebsocketsMessage &message) {
  Serial.print("Got Message: ");
  Serial.println(message.data());

  JsonDocument doc_msg_callback;
  deserializeJson(doc_msg_callback, message.data());

  String opcode = doc_msg_callback["op"];
  String encrypted_in = doc_msg_callback["encrypted"];

  if (strcmp(opcode.c_str(), "encrypted_final") == 0) {

    convert_hex(encrypted, 16, encrypted_in.c_str());
    
    Serial.println("Final Encrypted Message: ");
    char to_print_1[16*4];
    for (int i = 0; i < 16*4; i ++)
    {
      sprintf(to_print_1+2*i, "%02X", encrypted[i]);
    }
    Serial.println(to_print_1);

    uint8_t key[16];
    String key_in = doc_msg_callback["key"];
    const char *key_c_str = key_in.c_str();
    for (int i = 0; i < 16; i ++)
    {
      key[i] = key_c_str[i];
    }

    Serial.println("Key rx: ");
    char to_print_2[16];
    for (int i = 0; i < 16; i ++)
    {
      sprintf(to_print_2+2*i, "%02X", key[i]);
    }
    Serial.println(to_print_2);

    mbedtls_aes_context aes;
    mbedtls_aes_init(&aes);
    Serial.println("\nInit MBEDTLS DONE");

    mbedtls_aes_setkey_dec(&aes, key, 128);
    Serial.println("Set Key MBEDTLS DONE");

    uint8_t output[16*4];
    for (int i = 0; i < 4; i ++)
    {
      mbedtls_aes_crypt_ecb(&aes, MBEDTLS_AES_DECRYPT, encrypted + 16*i, output + 16*i);
    }
    Serial.println("Decrypt MBEDTLS DONE");

    Serial.println("Decrypted Hex: ");

    char to_print[16*8];
    for (int i = 0; i < 16*4; i ++)
    {
      sprintf(to_print+2*i, "%02X", output[i]);
    }
    Serial.println(to_print);

    String final = hexToASCII(String(to_print));

    Serial.println("Decrypted ASCII: " + final);


    // Serial.println("Output[5] = ");
    // Serial.println(output[5]);

    // for (int i = 0; i < output[5]; i ++)
    // {
    //   buzz(500);
    //   delay(500);
    // }
    
    mbedtls_aes_free( &aes );
    
  } else if (strcmp(opcode.c_str(), "init_encrypt") == 0) {
    convert_hex(encrypted + 16, 16*3, encrypted_in.c_str());
  }
}

void setup() {
  pinMode(14, OUTPUT);
  digitalWrite(14, LOW);

  Serial.begin(115200);
  
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_NETWORK);//, "enee408i");

  while (WiFi.status() != WL_CONNECTED) {
    Serial.println("No Wifi!");
    delay(500);
  }

  client.onMessage(onMessageCallback);

  client.connect("ws://192.168.1.142:7000");

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
  doc_begin["name"] = "BotD";

  serializeJson(doc_begin, send_buff);
  client.send(reinterpret_cast<const char *>(send_buff));

  Serial.println("Sent message!");

}


void loop() {
  client.poll();
  delay(1);
}

void buzz(int t)
{
  ledcAttachPin(BUZZ, BUZZ_CHANNEL);
  ledcWriteNote(BUZZ_CHANNEL, NOTE_C, octave);
  delay(t);
  noTone(BUZZ_CHANNEL);
}