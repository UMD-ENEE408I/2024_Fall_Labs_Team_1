#include "LineFollower.h"

LineFollower::LineFollower() : adc1(), adc2(), mpu() {}

void LineFollower::begin() {
    Serial.begin(115200);
    Serial.println("Initializing Line Follower...");
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

    if (!mpu.begin()) {
        Serial.println("Failed to find MPU6050 chip");
        while (1) { delay(10); }
    }
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
}

void LineFollower::setPID(float kp, float ki, float kd) {
    Kp = kp;
    Ki = ki;
    Kd = kd;
    Serial.print("PID settings updated - Kp: ");
    Serial.print(Kp);
    Serial.print(", Ki: ");
    Serial.print(Ki);
    Serial.print(", Kd: ");
    Serial.println(Kd);

}

void LineFollower::stopRobot() {
    M1_stop();
    M2_stop();
    Serial.println("Robot stopped.");
}

void LineFollower::readADC() {
    for (int i = 0; i < 8; i++) {
        adc1_buf[i] = adc1.readADC(i);
        adc2_buf[i] = adc2.readADC(i);
    }
    Serial.println("ADC readings updated.");
}

void LineFollower::digitalConvert() {
    int threshold = 700;
    for (int i = 0; i < 7; i++) {
        lineArray[2 * i] = (adc1_buf[i] > threshold) ? 0 : 1;
        if (i < 6) lineArray[2 * i + 1] = (adc2_buf[i] > threshold) ? 0 : 1;
    }
    Serial.println("Digital conversion complete.");
    Serial.print("lineArray: ");
    for (int i = 0; i < 13; i++) {
        Serial.print(lineArray[i]);
        Serial.print(" ");
    }
    Serial.println();
}

float LineFollower::getPosition() {
    float sum = 0.0;
    float count = 0.0;
    for (int i = 0; i < 13; i++) {
        if (lineArray[i] == 1) {
            sum += (float)i;
            count++;
        }
    }
    float position = (count != 0 ? sum / count : -1);
    Serial.print("Position: ");
    Serial.println(position);
    return position;
}

void LineFollower::M1_forward(int pwm_value) {
    ledcWrite(M1_IN_1_CHANNEL, 0);
    ledcWrite(M1_IN_2_CHANNEL, pwm_value);
}

void LineFollower::M2_forward(int pwm_value) {
    ledcWrite(M2_IN_1_CHANNEL, 0);
    ledcWrite(M2_IN_2_CHANNEL, pwm_value);
}

void LineFollower::M1_backward(int pwm_value) {
    ledcWrite(M1_IN_1_CHANNEL, pwm_value);
    ledcWrite(M1_IN_2_CHANNEL, 0);
}

void LineFollower::M2_backward(int pwm_value) {
    ledcWrite(M2_IN_1_CHANNEL, pwm_value);
    ledcWrite(M2_IN_2_CHANNEL, 0);
}

void LineFollower::M1_stop() {
    ledcWrite(M1_IN_1_CHANNEL, 0);
    ledcWrite(M1_IN_2_CHANNEL, 0);
}

void LineFollower::M2_stop() {
    ledcWrite(M2_IN_1_CHANNEL, 0);
    ledcWrite(M2_IN_2_CHANNEL, 0);
}

void LineFollower::turnCorner(bool cc) {
  turn_to(encoderCountsForTurn, cc);
}


void LineFollower::turn_to(float target, bool cc)
{
  Encoder enc1(M1_ENC_A, M1_ENC_B);
  Encoder enc2(M2_ENC_A, M2_ENC_B);
  enc1.write(0);
  enc2.write(0);
  int targetCounts = encoderCountsForTurn;

  if (cc)
  {
    M1_forward(TURN_PWM);
    M2_backward(TURN_PWM);
  }
  else
  {
    M1_backward(TURN_PWM);
    M2_forward(TURN_PWM);
  }

  Serial.println("Target: " + String(targetCounts));

  while (abs(enc1.read()) < targetCounts)
  {
    Serial.println("Enc1: " + String(enc1.read()));
    delay(10);
  }
  M1_stop();
  M2_stop();
  Serial.println("Finished Turning.");

}

void LineFollower::update() {
    readADC();
    digitalConvert();
    float pos = getPosition();
    pos = (pos < 0) ? prev_pos : pos;

    float error = mid - pos;
    unsigned long current_time = millis();
    float dt = (current_time - prev_time) / 1000.0;

    integral += error * dt;
    float derivative = (error - prev_error) / dt;
    float u = Kp * error + Ki * integral + Kd * derivative;

    int rightWheelPWM = constrain(base_pid + u, 0, PWM_MAX);
    int leftWheelPWM = constrain(base_pid - u, 0, PWM_MAX);

    M1_forward(rightWheelPWM);
    M2_forward(leftWheelPWM);

    Serial.print("Position: ");
    Serial.println(pos);
    Serial.print("Error: ");
    Serial.println(error);
    Serial.print("Right PWM: ");
    Serial.println(rightWheelPWM);
    Serial.print("Left PWM: ");
    Serial.println(leftWheelPWM);

    prev_error = error;
    prev_time = current_time;
    prev_pos = pos;

    int weighted_sum = 0;
    for (int i = 0; i < 13; i++) {
        weighted_sum += (i - 6) * lineArray[i];
    }
    Serial.print("Weighted Sum: ");
    Serial.println(weighted_sum);
    
    if (weighted_sum < -12) {
        Serial.println("Detected left turn...");
        M1_stop();
        M2_stop();
        delay(1000); // Pause before turning
        turnCorner(true);  // Execute 90-degree left turn
        return;
    }

    if (weighted_sum > 12) {
        Serial.println("Detected right turn...");
        M1_stop();
        M2_stop();
        delay(1000); // Pause before turning
        turnCorner(false);  // Execute 90-degree right turn
        return;
    }

    delay(10);  // Adjust delay as needed
}
