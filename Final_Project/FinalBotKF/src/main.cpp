#include <Arduino.h>
#include "LineFollower.h"
#include "comms.h"

uint8_t send_buff[500];

LineFollower lf;

int delta_checkpoint = 5001;


#define  START 1
#define  START2 2 
#define  START3 3
#define  START4 4
#define  START5 5
#define  UNKNOWN 6
#define  NO_MANS_LAND 7
#define  COLOR_SQUARE 8
#define  LISTEN 9
#define  NAV 10
#define  DOTTED_NAV 11
#define  COLOR_NAV 12

int curr;
int prev;

int audio_result;

void setup() {
  Serial.begin(115200);
  curr = START;

  lf.begin();
  lf.setPID(2.0, 0.1, 50.0);  // Setting initial PID values
  lf.stopRobot();             // Stop the robot initially

  delay(1000);

  comms_begin();

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
    delay(750);
    lf.stopRobot();
    delay(1000);

    curr = prev+1;
    Serial.print("WE ARE NOW CURR ");
    Serial.println(curr);
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
        lf.turn_motors3((audio_result == 1));
        delay(1000);
        lf.stopRobot();
        delay(250);

        prev = curr;
        curr = START5;
      }
    }

    if (count > 9 && delta_checkpoint > 200)
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

      delta_checkpoint = 0;
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
      dir = (side == 1);
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
    lf.turn_motors3((audio_result == 1));
    delay(1000);
    lf.stopRobot();
    delay(1000);
    
    curr = START4;

    break;

  default:
    lf.stopRobot();
    while(1);
    break;
  }

  comms_loop();
  delta_checkpoint++;
  //Serial.println(delta_checkpoint);
  delay(10);
}

