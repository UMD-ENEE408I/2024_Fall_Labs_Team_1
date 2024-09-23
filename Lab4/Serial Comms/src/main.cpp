#include <Arduino.h>

const unsigned int M1_IN_1 = 13;
const unsigned int M1_IN_2 = 12;
const unsigned int M2_IN_1 = 25;
const unsigned int M2_IN_2 = 14;

const unsigned int M1_IN_1_CHANNEL = 8;
const unsigned int M1_IN_2_CHANNEL = 9;
const unsigned int M2_IN_1_CHANNEL = 10;
const unsigned int M2_IN_2_CHANNEL = 11;

const unsigned int M1_I_SENSE = 35;
const unsigned int M2_I_SENSE = 34;

const int freq = 5000;
const int resolution = 8;

const int speed_pwm = 110;

void moveForward();
void moveBackward();
void turnLeft();
void turnRight();
void stop();

void setup() {
  Serial.begin(115200);
  
  ledcSetup(M1_IN_1_CHANNEL, freq, resolution);
  ledcSetup(M1_IN_2_CHANNEL, freq, resolution);
  ledcSetup(M2_IN_1_CHANNEL, freq, resolution);
  ledcSetup(M2_IN_2_CHANNEL, freq, resolution);

  ledcAttachPin(M1_IN_1, M1_IN_1_CHANNEL);
  ledcAttachPin(M1_IN_2, M1_IN_2_CHANNEL);
  ledcAttachPin(M2_IN_1, M2_IN_1_CHANNEL);
  ledcAttachPin(M2_IN_2, M2_IN_2_CHANNEL);

  pinMode(M1_I_SENSE, INPUT);
  pinMode(M2_I_SENSE, INPUT);
  delay(1000);

}

void loop() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    
    switch (command[0])
    {
      case 'F':
        moveForward();
        break;
      case 'B':
        moveBackward();
        break;
      case 'L':
        turnLeft();
        break;
      case 'R':
        turnRight();
        break;
      case 'S':
      default:
        stop();
        break;
    }
  }
}

void moveForward() {
  ledcWrite(M1_IN_1_CHANNEL, 0);
  ledcWrite(M1_IN_2_CHANNEL, speed_pwm);
  ledcWrite(M2_IN_1_CHANNEL, 0);
  ledcWrite(M2_IN_2_CHANNEL, speed_pwm);
}

void moveBackward() {
  ledcWrite(M1_IN_1_CHANNEL, speed_pwm);
  ledcWrite(M1_IN_2_CHANNEL, 0);
  ledcWrite(M2_IN_1_CHANNEL, speed_pwm);
  ledcWrite(M2_IN_2_CHANNEL, 0);
}

void turnLeft() {
  ledcWrite(M1_IN_1_CHANNEL, speed_pwm);
  ledcWrite(M1_IN_2_CHANNEL, 0);
  ledcWrite(M2_IN_1_CHANNEL, 0);
  ledcWrite(M2_IN_2_CHANNEL, speed_pwm);
}

void turnRight() {
  ledcWrite(M1_IN_1_CHANNEL, 0);
  ledcWrite(M1_IN_2_CHANNEL, speed_pwm);
  ledcWrite(M2_IN_1_CHANNEL, speed_pwm);
  ledcWrite(M2_IN_2_CHANNEL, 0);
}

void stop() {
  ledcWrite(M1_IN_1_CHANNEL, 0);
  ledcWrite(M1_IN_2_CHANNEL, 0);
  ledcWrite(M2_IN_1_CHANNEL, 0);
  ledcWrite(M2_IN_2_CHANNEL, 0);
}
