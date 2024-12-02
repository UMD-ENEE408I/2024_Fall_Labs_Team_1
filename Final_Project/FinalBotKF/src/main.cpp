#include <Arduino.h>
#include <ArduinoJson.h>
#include <ArduinoWebsockets.h>
#include <WiFi.h>
#include "LineFollower.h"

#define WIFI_NETWORK "enee408i"

using namespace websockets;
WebsocketsClient client;

uint8_t send_buff[500];

LineFollower lf;

enum STATE {
  START,
  LISTEN,
  NAV,
  DOTTED_NAV,
  COLOR_NAV,
};
STATE curr;

void setup() {
  Serial.begin(115200);
  curr = START;

  lf.begin();
  lf.setPID(2.0, 0.1, 50.0);  // Setting initial PID values
  lf.stopRobot();             // Stop the robot initially
  delay(1000);
}

void loop() {
  switch (curr)
  {
  case START:

    break;
  
  default:
    break;
  }

  lf.update();  // Update robot for line following
  delay(10);
}

