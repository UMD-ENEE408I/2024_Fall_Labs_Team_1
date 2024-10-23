#ifndef LINEFOLLOWER_H
#define LINEFOLLOWER_H

#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_MCP3008.h>
#include <Encoder.h>

class LineFollower {
    public:
        void begin();
        void update();
        void setPID(float kp, float ki, float kd);
        void turnCorner();
        void stopRobot();

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

        int adc1_buf[8];
        int adc2_buf[8];
        uint8_t lineArray[13];

        const unsigned int ADC_1_CS = 2;
        const unsigned int ADC_2_CS = 17;

        Adafruit_MCP3008 adc1;
        Adafruit_MCP3008 adc2;

        // Motors
        const unsigned int M1_IN_1 = 13;
        const unsigned int M1_IN_2 = 12;
        const unsigned int M2_IN_1 = 25;
        const unsigned int M2_IN_2 = 14;

        const unsigned int M1_I_SENSE = 35;
        const unsigned int M2_I_SENSE = 34;

        // Motor pins and settings
        const int M1_IN_1_CHANNEL = 8;
        const int M1_IN_2_CHANNEL = 9;
        const int M2_IN_1_CHANNEL = 10;
        const int M2_IN_2_CHANNEL = 11;

        const unsigned int PWM_MAX = 110;
        const unsigned int TURN_PWM = 90;
        const int freq = 5000;
        const int resolution = 8; // 8-bit resolution -> PWM values go from 0-255

        // PID variables
        float Kp, Ki, Kd;
        float e, d_e, total_e;
        const int base_pid = 80;
        const float mid = 6;
        unsigned long prev_time = 0;
        float prev_error = 0;
        float integral = 0;
        float prev_pos = 6;
};

#endif