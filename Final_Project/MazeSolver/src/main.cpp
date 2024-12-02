#include "LineFollower.h"

LineFollower robot;

void setup() {
    Serial.begin(115200);
    robot.begin();
    robot.setPID(2.0, 0.1, 50.0);  // Setting initial PID values
    robot.stopRobot();             // Stop the robot initially
    delay(1000);
}

void loop() {
    robot.update();  // Update robot for line following

    
    delay(10);
}
