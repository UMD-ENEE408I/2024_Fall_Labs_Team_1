
#include "LineFollower.h"

LineFollower robot;

// const unsigned int M1_ENC_A = 39;
// const unsigned int M1_ENC_B = 38;
// const unsigned int M2_ENC_A = 37;
// const unsigned int M2_ENC_B = 36;

// Encoder enc1(M1_ENC_A, M1_ENC_B);
// Encoder enc2(M2_ENC_A, M2_ENC_B);

void setup() {
    Serial.begin(9600);
    robot.begin();
    delay(1000);
    robot.stopRobot();
    delay(1000);
}

void loop() {
    robot.update();
    delay(10);
}
