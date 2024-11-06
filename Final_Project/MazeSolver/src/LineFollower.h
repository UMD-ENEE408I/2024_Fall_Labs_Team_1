#ifndef LINEFOLLOWER_H
#define LINEFOLLOWER_H

#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_MCP3008.h>
#include <Encoder.h>

class LineFollower {
public:
    LineFollower();
    void begin();
    void update();
    void setPID(float kp, float ki, float kd);
    void stopRobot();
    void turnCorner(bool cc);

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
    void turn_to(float target, bool cc);

    // Pin configuration and constants
    const unsigned int ADC_1_CS = 2;
    const unsigned int ADC_2_CS = 17;
    const unsigned int M1_IN_1 = 13;
    const unsigned int M1_IN_2 = 12;
    const unsigned int M2_IN_1 = 25;
    const unsigned int M2_IN_2 = 14;

    const unsigned int M1_I_SENSE = 35;
    const unsigned int M2_I_SENSE = 34;

    const unsigned int M1_IN_1_CHANNEL = 8;
    const unsigned int M1_IN_2_CHANNEL = 9;
    const unsigned int M2_IN_1_CHANNEL = 10;
    const unsigned int M2_IN_2_CHANNEL = 11;

    const unsigned int M1_ENC_A = 39;
    const unsigned int M1_ENC_B = 38;
    const unsigned int M2_ENC_A = 37;
    const unsigned int M2_ENC_B = 36;

    const int encoderCountsPerRevolution = 360;
    const float wheelCircumference = 0.2;
    const float turnRadius = 0.1;
    const float turnAngleDegrees = 90;
    float distanceToTravel = (PI * turnRadius * (turnAngleDegrees / 180));

    // 90 deg turn encoder count needed:
    int encoderCountsForTurn = (distanceToTravel / wheelCircumference) * encoderCountsPerRevolution;

    const unsigned int PWM_MAX = 110;
    const unsigned int TURN_PWM = 90;
    const int freq = 5000;
    const int resolution = 8;

    // PID settings
    float Kp = 2;
    float Ki = 0;
    float Kd = 50;
    const int base_pid = 90;
    const float mid = 6;

    // Turning angle
    const int TURN_ANGLE_DEGREES = 90;
    const float TURN_ANGLE_RADIANS = TURN_ANGLE_DEGREES * (PI / 180.0);

    uint8_t lineArray[13];
    int adc1_buf[8];
    int adc2_buf[8];

    Adafruit_MCP3008 adc1;
    Adafruit_MCP3008 adc2;
    Adafruit_MPU6050 mpu;

    // PID variables
    float prev_error = 0;
    float integral = 0;
    float prev_pos = 6;
    unsigned long prev_time = 0;
};

#endif // LINEFOLLOWER_H
