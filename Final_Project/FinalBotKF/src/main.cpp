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
  START2,
  START3,
  START4,
  START5,
  UNKNOWN,
  NO_MANS_LAND,
  COLOR_SQUARE,
  LISTEN,
  NAV,
  DOTTED_NAV,
  COLOR_NAV,
};

STATE curr;
STATE prev;

int audio_result;

void setup() {
  Serial.begin(115200);
  curr = START;

  lf.begin();
  lf.setPID(2.0, 0.1, 50.0);  // Setting initial PID values
  lf.stopRobot();             // Stop the robot initially
  delay(1000);
}

void loop() {

  float count;
  float pos;

  int dir;
  int side;

  switch (curr)
  {
  case COLOR_SQUARE:
    // TOOD find color
    
    delay(1000);
    lf.turn_motors2(1);
    delay(2000);
    lf.stopRobot();
    delay(1000);

    curr = (enum STATE)(curr + 1);
  break;

  case START:
  case START2:
  case START3:
  case START4:
  case START5:

    pos = lf.update(&count, &side);  // Update robot for line following

    if (pos <= 0)
    {
      prev = curr;
      curr = NO_MANS_LAND;
    }
    Serial.println();

    if (curr == START4)
    {
      if (side == (audio_result == 1) ? 2 : 1)
      {
        lf.stopRobot();
        delay(250);
        lf.turn_motors2((audio_result == 1));
        delay(2000);
        lf.stopRobot();
        delay(250);

        curr = START5;
      }
    }

    if (count > 9)
    {
      lf.stopRobot();
      prev = curr;
      if (curr == START || curr == START2 || curr == START3 || curr == START5)
      {
        curr = COLOR_SQUARE;
      }
      else if (curr == START3)
      {
        curr = LISTEN;
      }
    }

    break;

  case NO_MANS_LAND:
    //cc = (pos <=6);
    Serial.println("entered NO MANS LAND!");
    // Serial.print("With a");
    // Serial.print(cc);
    // Serial.println(" Hand Turn");
    
    //pos = lf.getPosition(&count, &side);
    //delay(10);
    while (count < 2)
    {
      Serial.print(" | Side (1 = left, 2 = right): ");
      Serial.println(side);
      dir = (side == 2);
      lf.turn_motors(dir);
      pos = lf.getPosition(&count, NULL);
      Serial.println(count);
      delay(10);
    }
    lf.stopRobot();
    curr = prev;

    break;
  
  case LISTEN:

    // todo listen req

    audio_result = 0; // 0 left 1 right

    delay(1000);
    lf.turn_motors2((audio_result == 1));
    delay(2000);
    lf.stopRobot();
    delay(1000);
    
    curr = START4;

    break;

  default:
    lf.stopRobot();
    while(1);
    break;
  }

  delay(10);
}

