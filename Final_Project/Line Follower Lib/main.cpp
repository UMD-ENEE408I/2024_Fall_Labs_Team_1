#ifndef LINEFOLLOWER_H
#define LINEFOLLOWER_H

#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_MCP3008.h>
#include <Encoder.h>

class LineFollower {
public:
    LineFollower(int adc1CS, int adc2CS, int m1EncA, int m1EncB, int m2EncA, int m2EncB);
    void begin();
    void update();
    void setPID(float kp, float ki, float kd);
    void turnCorner();

private:
    void readADC();
    void digitalConvert();
    float getPosition();
    void M1_forward(int pwm_value);
    void M2_forward(int pwm_value);
    void M1_backward(int pwm_value);
    void M2_backward(int pwm_value);
    void M1_stop();
    void M2_stop();

    Adafruit_MCP3008 adc1;
    Adafruit_MCP3008 adc2;
    Encoder enc1;
    Encoder enc2;

    int adc1_buf[8];
    int adc2_buf[8];
    uint8_t lineArray[13];

    // Motor pins and settings
    const int M1_IN_1_CHANNEL = 8;
    const int M1_IN_2_CHANNEL = 9;
    const int M2_IN_1_CHANNEL = 10;
    const int M2_IN_2_CHANNEL = 11;
    const int PWM_MAX = 255;

    // PID variables
    float Kp, Ki, Kd;
    float e, d_e, total_e;
    const int base_pid = 80;
};

#endif

//create the source file

#include "LineFollower.h"

LineFollower::LineFollower(int adc1CS, int adc2CS, int m1EncA, int m1EncB, int m2EncA, int m2EncB)
    : adc1(), adc2(), enc1(m1EncA, m1EncB), enc2(m2EncA, m2EncB) {
    adc1.begin(adc1CS);
    adc2.begin(adc2CS);
}

void LineFollower::begin() {
    pinMode(M1_IN_1_CHANNEL, OUTPUT);
    pinMode(M1_IN_2_CHANNEL, OUTPUT);
    pinMode(M2_IN_1_CHANNEL, OUTPUT);
    pinMode(M2_IN_2_CHANNEL, OUTPUT);
    M1_stop();
    M2_stop();
}

void LineFollower::readADC() {
    for (int i = 0; i < 8; i++) {
        adc1_buf[i] = adc1.readADC(i);
        adc2_buf[i] = adc2.readADC(i);
    }
}

void LineFollower::digitalConvert() {
    int threshold = 700;
    for (int i = 0; i < 7; i++) {
        lineArray[2*i] = (adc1_buf[i] > threshold) ? 1 : 0;
        if (i < 6) {
            lineArray[2*i+1] = (adc2_buf[i] > threshold) ? 1 : 0;
        }
    }
}

float LineFollower::getPosition() {
    // Implement your position logic
    return 6; // Placeholder for actual logic
}

void LineFollower::setPID(float kp, float ki, float kd) {
    Kp = kp;
    Ki = ki;
    Kd = kd;
}

void LineFollower::update() {
    readADC();
    digitalConvert();
    float pos = getPosition();
    
    // Define PID control logic
    // e = ...; d_e = ...; total_e = ...;
    // Implement control and motor commands
    M1_forward(0); // Replace with actual PWM value
    M2_forward(0); // Replace with actual PWM value
}

void LineFollower::M1_forward(int pwm_value) {
    ledcWrite(M1_IN_1_CHANNEL, 0);
    ledcWrite(M1_IN_2_CHANNEL, pwm_value);
}

void LineFollower::M2_forward(int pwm_value) {
    ledcWrite(M2_IN_1_CHANNEL, 0);
    ledcWrite(M2_IN_2_CHANNEL, pwm_value);
}

void LineFollower::M1_stop() {
    ledcWrite(M1_IN_1_CHANNEL, PWM_MAX);
    ledcWrite(M1_IN_2_CHANNEL, PWM_MAX);
}

void LineFollower::M2_stop() {
    ledcWrite(M2_IN_1_CHANNEL, PWM_MAX);
    ledcWrite(M2_IN_2_CHANNEL, PWM_MAX);
}

void LineFollower::turnCorner() {
    // Implement turn logic based on encoder readings
}

//use the library in 

#include <LineFollower.h>

LineFollower robot(2, 17, 39, 38, 37, 36); // Pass your pin numbers

void setup() {
    Serial.begin(115200);
    robot.begin();
}

void loop() {
    robot.update();
}
