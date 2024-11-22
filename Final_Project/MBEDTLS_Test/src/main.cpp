#include <Arduino.h>
#include "mbedtls/aes.h"
#include "mbedtls/base64.h"

uint8_t encrypted[16*4];

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

void setup() {
  Serial.begin(115200);
}

void loop() {

    String encrypted_in = String("AA858E01C63EB9EB73FF61E9562CB9A05D8F1EB19497FAA5578DC630000110A6FDF188F17072529F892B02DC8DABE21EAF1F36226CBF2A58D33EA0F1A56B758F");

    convert_hex(encrypted, 16*4, encrypted_in.c_str());
    
    Serial.println("Final Encrypted Message: ");
    char to_print_1[16*4];
    for (int i = 0; i < 16*4; i ++)
    {
      sprintf(to_print_1+2*i, "%02X", encrypted[i]);
    }
    Serial.println(to_print_1);

    uint8_t key[16];
    String key_in = String("enee408ikeynumb1");
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

    while(1);
}

