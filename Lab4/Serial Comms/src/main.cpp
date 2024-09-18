#include <Arduino.h>

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("init");
}

void loop() {
  if (Serial.available() > 0) {
    String rx = Serial.readStringUntil('\n');
    Serial.print("Rx: ");
    Serial.println(rx);
  }

  delay(100);
}

