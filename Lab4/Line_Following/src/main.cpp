#include <Arduino.h>
#include <Adafruit_MCP3008.h>
#include <Encoder.h>

// ADC (line sensor)
Adafruit_MCP3008 adc1;
Adafruit_MCP3008 adc2;

const unsigned int ADC_1_CS = 2;
const unsigned int ADC_2_CS = 17;

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

const unsigned int PWM_MAX = 255;
const int freq = 5000;
const int resolution = 8; // 8-bit resolution -> PWM values go from 0-255

// LED
const int ledChannel = 0;

// PID
const int base_pid = 80; // Base speed for robot
const float mid = 6;

float e;
float d_e;
float total_e;

// Assign values to the following feedback constants:
float Kp;
float Kd;
float Ki;


/*
 *  Line sensor functions
 */
void readADC() {
  for (int i = 0; i < 8; i++) {
    adc1_buf[i] = adc1.readADC(i);
    adc2_buf[i] = adc2.readADC(i);
  }
}

// Converts ADC readings to binary array lineArray[] (Check threshold for your robot) 
void digitalConvert() {
  int threshold = 700;
  for (int i = 0; i < 7; i++) {
    if (adc1_buf[i]>threshold) {
      lineArray[2*i] = 0; 
    } else {
      lineArray[2*i] = 1;
    }

    if (i<6) {
      if (adc2_buf[i]>threshold){
        lineArray[2*i+1] = 0;
      } else {
        lineArray[2*i+1] = 1;
      }
    }

    // print line sensor position
    // for(int i = 0; i < 13; i++) {
    //   Serial.print(lineArray[2*i+1]); Serial.print(" ");
    // }
  }
}

// Calculate robot's position on the line 
float getPosition() {
  /* Using lineArray[], which is an array of 13 Boolean values representing 1 
   * if the line sensor reads a white surface and 0 for a dark surface, 
   * this function returns a value between 0-12 for where the sensor thinks 
   * the center of line is (6 being the middle)
   */
  float xor_1 = -1;
  float xor_2 = -1;
  for (int i = 0; i < sizeof(lineArray)/sizeof(lineArray[0]) - 1; i++)
  {
    if (lineArray[i] ^ lineArray[i+1])
    {
      if (xor_1 < 0)
      {
        xor_1 = (float) i+1;
      } 
      else if (xor_2 < 0)
      {
        xor_2 = (float) i;
        break;
      }
    }
  }

  if (xor_1 > 0 && xor_2 > 0)
  {
    return ((xor_1 + xor_2) / 2.0);
  }

  return -1;
}

/*
 *  Movement functions
 */
void M1_forward(int pwm_value) {
  ledcWrite(M1_IN_1_CHANNEL, 0);
  ledcWrite(M1_IN_2_CHANNEL, pwm_value);
}
void M2_forward(int pwm_value) {
  ledcWrite(M2_IN_1_CHANNEL, 0);
  ledcWrite(M2_IN_2_CHANNEL, pwm_value);
}

void M1_backward(int pwm_value) {
  ledcWrite(M1_IN_1_CHANNEL, pwm_value);
  ledcWrite(M1_IN_2_CHANNEL, 0);
}
void M2_backward(int pwm_value) {
  ledcWrite(M2_IN_1_CHANNEL, pwm_value);
  ledcWrite(M2_IN_2_CHANNEL, 0);
}

void M1_stop() {
  ledcWrite(M1_IN_1_CHANNEL, PWM_MAX);
  ledcWrite(M1_IN_2_CHANNEL, PWM_MAX);
}
void M2_stop() {
  ledcWrite(M2_IN_1_CHANNEL, PWM_MAX);
  ledcWrite(M2_IN_2_CHANNEL, PWM_MAX);
}

void turnCorner(bool cc) {
  /* 
   * Use the encoder readings to turn the robot 90 degrees clockwise or 
   * counterclockwise depending on the argument. You can calculate when the 
   * robot has turned 90 degrees using either the IMU or the encoders + wheel measurements
   */

}

/*
 *  setup and loop
 */
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

  adc1.begin(ADC_1_CS);  
  adc2.begin(ADC_2_CS);

  pinMode(M1_I_SENSE, INPUT);
  pinMode(M2_I_SENSE, INPUT);

  M1_stop();
  M2_stop();

  delay(100);
}

void loop() {


  Encoder enc1(M1_ENC_A, M1_ENC_B);
  Encoder enc2(M2_ENC_A, M2_ENC_B);

  while(true) {
    int u;
    int rightWheelPWM;
    int leftWheelPWM;
    float pos;

    readADC();
    digitalConvert();

    pos = getPosition();
    if (pos < 0)
    {
      // err case
      pos = 6;
    }
    
    // Define the PID errors
    e = ;
    d_e = ; 
    total_e= ;

    // Implement PID control (include safeguards for when the PWM values go below 0 or exceed maximum)
    u = ;
    rightWheelPWM = ;
    leftWheelPWM = ;

    M1_forward(rightWheelPWM);
    M2_forward(leftWheelPWM);

    // Check for corners
    if(0) {
      turnCorner(1);
    }

  }
}
import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import odeint
import numpy as np
import matplotlib.pyplot as plt

time = 0
integral = 0
time_prev = -1e-6
e_prev = 0

  def PID(Kp, Ki, Kd, setpoint, measurement):
    global time, integral, time_prev, e_prev

    # Value of offset - when the error is equal zero
    offset = 320
    
    # PID calculations
    e = setpoint - measurement
        
    P = Kp*e
    integral = integral + Ki*e*(time - time_prev)
    D = Kd*(e - e_prev)/(time - time_prev)

    # calculate manipulated variable - MV 
    MV = offset + P + integral + D
    
    # update stored data for next iteration
    e_prev = e
    time_prev = time
    return MV

  def system(t, temp, Tq):
    epsilon = 1
    tau = 4
    Tf = 300
    Q = 2
    dTdt = 1/(tau*(1+epsilon)) * (Tf-temp) + Q/(1+epsilon)*(Tq-temp)
    return dTdt

  tspan = np.linspace(0,10,50)
Tq = 320,
sol = odeint(system,300, tspan, args=Tq, tfirst=True)
plt.xlabel('Time')
plt.ylabel('Temperature')
plt.plot(tspan,sol)

