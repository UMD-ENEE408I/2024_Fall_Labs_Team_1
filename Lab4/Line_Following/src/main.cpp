#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_MCP3008.h>
#include <Encoder.h>

// ADC (line sensor)
Adafruit_MCP3008 adc1;
Adafruit_MCP3008 adc2;

Adafruit_MPU6050 mpu;

const unsigned int ADC_1_CS = 2;
const unsigned int ADC_2_CS = 17;

const unsigned int ADC_3_CS = 18;
const unsigned int ADC_4_CS = 19;

int adc1_buf[8];
int adc2_buf[8];

uint8_t lineArray[13];

// Encoders
const unsigned int M1_ENC_A = 39;
const unsigned int M1_ENC_B = 38;
const unsigned int M2_ENC_A = 37;
const unsigned int M2_ENC_B = 36;

// Motors
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

const unsigned int PWM_MAX = 110;
const unsigned int TURN_PWM = 90;
const int freq = 5000;
const int resolution = 8; // 8-bit resolution -> PWM values go from 0-255

// LED
const int ledChannel = 0;

// PID
const int base_pid = 90; // Base speed for robot
const float mid = 6;

float e;
float d_e;
float total_e;

// Assign values to the following feedback constants:
const float Kp = 2;
const float Kd = 50;
const float Ki = 0;

void turn_to(float target);

/*
 *  Line sensor functions
 */
void readADC()
{
  for (int i = 0; i < 8; i++)
  {
    adc1_buf[i] = adc1.readADC(i);
    adc2_buf[i] = adc2.readADC(i);
  }
}

// Converts ADC readings to binary array lineArray[] (Check threshold for your robot)
void digitalConvert()
{
  int threshold = 700;
  for (int i = 0; i < 7; i++)
  {
    if (adc1_buf[i] > threshold)
    {
      lineArray[2 * i] = 0;
    }
    else
    {
      lineArray[2 * i] = 1;
    }

    if (i < 6)
    {
      if (adc2_buf[i] > threshold)
      {
        lineArray[2 * i + 1] = 0;
      }
      else
      {
        lineArray[2 * i + 1] = 1;
      }
    }

    // print line sensor position
    // for(int i = 0; i < 13; i++) {
    //   Serial.print(lineArray[2*i+1]); Serial.print(" ");
    // }
  }
}


// Calculate robot's position on the line
float getPosition()
{
  /* Using lineArray[], which is an array of 13 Boolean values representing 1
   * if the line sensor reads a white surface and 0 for a dark surface,
   * this function returns a value between 0-12 for where the sensor thinks
   * the center of line is (6 being the middle)
   */
  float sum = 0.0;
  float c = 0.0;
  for (int i = 0; i < 13; i++)
  {
    if (lineArray[i] == 1)
    {
      sum += (float) i;
      c++;
    }
  }

  return (c != 0 ? sum/c : -1);
}

/*
 *  Movement functions
 */
void M1_forward(int pwm_value)
{
  ledcWrite(M1_IN_1_CHANNEL, 0);
  ledcWrite(M1_IN_2_CHANNEL, pwm_value);
}
void M2_forward(int pwm_value)
{
  ledcWrite(M2_IN_1_CHANNEL, 0);
  ledcWrite(M2_IN_2_CHANNEL, pwm_value);
}

void M1_backward(int pwm_value)
{
  ledcWrite(M1_IN_1_CHANNEL, pwm_value);
  ledcWrite(M1_IN_2_CHANNEL, 0);
}
void M2_backward(int pwm_value)
{
  ledcWrite(M2_IN_1_CHANNEL, pwm_value);
  ledcWrite(M2_IN_2_CHANNEL, 0);
}

void M1_stop()
{
  ledcWrite(M1_IN_1_CHANNEL, PWM_MAX);
  ledcWrite(M1_IN_2_CHANNEL, PWM_MAX);
}
void M2_stop()
{
  ledcWrite(M2_IN_1_CHANNEL, PWM_MAX);
  ledcWrite(M2_IN_2_CHANNEL, PWM_MAX);
}

void turnCorner(bool cc)
{
  turn_to(3.14/2.0);
}

void turn_to(float target)
{
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  
  float curr = 0;
  unsigned long prevtime = millis();

  while (curr < target)
  {
    Serial.print("turning... curr: " + String(curr) + " target: " + String(target));
    // spin motors opposite direction
    M1_forward(TURN_PWM);
    M2_backward(TURN_PWM);

    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    unsigned long t = millis();
    Serial.println(" | " + String(float(t - prevtime) / 1000.0) + " | " + String(g.gyro.x));
    
    // threshold in case of not moving (IMU sill reads small rotation)
    if (abs(g.gyro.x) > 0.1) 
    {
      curr += (float(t - prevtime) / 1000.0) * g.gyro.x; // update current heading (sec * rad/sec = rads traveled)
    }
    prevtime = t;
  }

  M1_stop();
  M2_stop();
}

void IMU_setup()
{
  delay(100);

  pinMode(ADC_3_CS, OUTPUT);
  pinMode(ADC_4_CS, OUTPUT);

  digitalWrite(ADC_3_CS, HIGH); // Without this the ADC's write
  digitalWrite(ADC_4_CS, HIGH); // to the SPI bus while the nRF24 is!!!!

  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
  case MPU6050_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case MPU6050_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case MPU6050_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case MPU6050_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
  case MPU6050_RANGE_250_DEG:
    Serial.println("+- 250 deg/s");
    break;
  case MPU6050_RANGE_500_DEG:
    Serial.println("+- 500 deg/s");
    break;
  case MPU6050_RANGE_1000_DEG:
    Serial.println("+- 1000 deg/s");
    break;
  case MPU6050_RANGE_2000_DEG:
    Serial.println("+- 2000 deg/s");
    break;
  }

  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
  case MPU6050_BAND_260_HZ:
    Serial.println("260 Hz");
    break;
  case MPU6050_BAND_184_HZ:
    Serial.println("184 Hz");
    break;
  case MPU6050_BAND_94_HZ:
    Serial.println("94 Hz");
    break;
  case MPU6050_BAND_44_HZ:
    Serial.println("44 Hz");
    break;
  case MPU6050_BAND_21_HZ:
    Serial.println("21 Hz");
    break;
  case MPU6050_BAND_10_HZ:
    Serial.println("10 Hz");
    break;
  case MPU6050_BAND_5_HZ:
    Serial.println("5 Hz");
    break;
  }

  Serial.println("");
  delay(100);

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
}

/*
 *  setup and loop
 */
void setup()
{
  Serial.begin(115200);

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

  //IMU_setup();

  delay(100);
}

void loop()
{

  Encoder enc1(M1_ENC_A, M1_ENC_B);
  Encoder enc2(M2_ENC_A, M2_ENC_B);

  unsigned long prev_time = 0;
  float prev_error = 0;
  float integral = 0;
  float prev_pos = 6;

  while (true)
  {
    float u;
    int rightWheelPWM;
    int leftWheelPWM;
    float pos;

    readADC();
    digitalConvert();

    pos = getPosition(); // Call your position function
    pos = (pos < 0) ? prev_pos : pos;

    Serial.println("Pos: " + String(pos));

    // Calculate PID error
    float error = mid - pos;
    unsigned long current_time = millis();
    float dt = ((float) (current_time - prev_time)) / 1000.0; // Convert to seconds

    // Update integral and derivative
    integral += error * dt;
    float derivative = (error - prev_error) / dt;

    //Serial.println("dt: " + String(dt));

    // PID output
    u = Kp * error + Ki * integral + Kd * derivative;

    //Serial.println("err: + " + String(error) + "; int: " + String(integral) + "; der: " + String(derivative));


    // Calculate PWM values for motors
    rightWheelPWM = ((base_pid + u) < 0) ? 0 : ( ((base_pid + u) > PWM_MAX) ? PWM_MAX : (base_pid + u));
    leftWheelPWM = ((base_pid - u) < 0) ? 0 : ( ((base_pid - u) > PWM_MAX) ? PWM_MAX : (base_pid - u));

    //Serial.println("u: + " + String(u) + "; rpwm: " + String(rightWheelPWM) + "; lpwm: " + String(leftWheelPWM));

    // Control motors
    M1_forward(rightWheelPWM);
    M2_forward(leftWheelPWM);

    // Update previous values
    prev_error = error;
    prev_time = current_time;
    prev_pos = pos;

    // Check for corners (condition to be defined)
    if (0)
    {
      M1_stop();
      M2_stop();
      delay(1000);
      turnCorner(1); // Implement corner turning logic
      (void) getPosition();
    }

    delay(10);
  }
}
