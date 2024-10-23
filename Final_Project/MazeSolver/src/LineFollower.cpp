#include "LineFollower.h"

void LineFollower::begin() {
    ledcSetup(M1_IN_1_CHANNEL, freq, resolution);
    ledcSetup(M1_IN_2_CHANNEL, freq, resolution);
    ledcSetup(M2_IN_1_CHANNEL, freq, resolution);
    ledcSetup(M2_IN_2_CHANNEL, freq, resolution);

    ledcAttachPin(M1_IN_1, M1_IN_1_CHANNEL);
    ledcAttachPin(M1_IN_2, M1_IN_2_CHANNEL);
    ledcAttachPin(M2_IN_1, M2_IN_1_CHANNEL);
    ledcAttachPin(M2_IN_2, M2_IN_2_CHANNEL);

    adc1.begin(ADC_1_CS);
    adc2.begin(ADC_2_CS);

    pinMode(M1_I_SENSE, INPUT);
    pinMode(M2_I_SENSE, INPUT);

    M1_stop();
    M2_stop();

    delay(100);
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
    float sum = 0.0;
    float c = 0.0;
    for (int i = 0; i < 13; i++)
    {
        if (lineArray[i] == 1)
        {
        sum += (float)i;
        c++;
        }
    }

    return (c != 0 ? sum / c : -1);
}

void LineFollower::setPID(float kp, float ki, float kd) {
    Kp = kp;
    Ki = ki;
    Kd = kd;
}

void LineFollower::update() {

    float u;
    int rightWheelPWM;
    int leftWheelPWM;
    float pos;

    readADC();
    digitalConvert();
    pos = getPosition();
    
    pos = (pos < 0) ? prev_pos : pos;

    Serial.println("Pos: " + String(pos));

    // Calculate PID error
    float error = mid - pos;
    unsigned long current_time = millis();
    float dt = ((float)(current_time - prev_time)) / 1000.0; // Convert to seconds

    // Update integral and derivative
    integral += error * dt;
    float derivative = (error - prev_error) / dt;

    // PID output
    u = Kp * error + Ki * integral + Kd * derivative;

    Serial.println("err: + " + String(error) + "; int: " + String(integral) + "; der: " + String(derivative));

    // Calculate PWM values for motors
    rightWheelPWM = ((base_pid + u) < 0) ? 0 : (((base_pid + u) > PWM_MAX) ? PWM_MAX : (base_pid + u));
    leftWheelPWM = ((base_pid - u) < 0) ? 0 : (((base_pid - u) > PWM_MAX) ? PWM_MAX : (base_pid - u));

    Serial.println("u: + " + String(u) + "; rpwm: " + String(rightWheelPWM) + "; lpwm: " + String(leftWheelPWM));

    M1_forward(rightWheelPWM);
    M2_forward(leftWheelPWM);

    // Update previous values
    prev_error = error;
    prev_time = current_time;
    prev_pos = pos;
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

void LineFollower::stopRobot() {
    M1_stop();
    M2_stop();
}