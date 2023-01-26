#include <Adafruit_MPU6050.h> // https://www.arduino.cc/reference/en/libraries/adafruit-mpu6050/
#include <Adafruit_Sensor.h> // part of above library
#include <SPI.h>
#include <SD.h>
#include <Wire.h>

/* IMU parameters */
Adafruit_MPU6050 mpu;
sensors_event_t accelerometer, gyro, temp;

double gyroOffset = 0.01;
// .04 for x axis offset
// .01 for z axis offset

uint8_t maxPWM = 250;
uint8_t minPWM = 80;

float dt;
double timeDelay = 75.0;

/* L298N Module */
int staticFriction = 130;
// Right Motor
#define in1 8
#define in2 7
#define enA 9

// Left Motor
#define in3 6
#define in4 5
#define enB 3

/* SD Card */
File general, imu_log, motor_log;
#define sdPort 4

/* Neural Network */
const int InputNodes = 3; // 3 if acceleration.z
const int HiddenNodes = 8;
const int OutputNodes = 1;
float Accum;
float Hidden[HiddenNodes];
float output;
float HiddenWeights[InputNodes+1][HiddenNodes] = { {5.09057, -4.00655, -8.87062, -0.871889, 1.70307, -1.60015, -1.23579, 1.4842}, {0.153815, -7.29709, -1.21556, -6.77342, 6.36775, -0.156223, 0.0280848, 0.13364}, {0.263409, -2.56881, -0.231012, -2.18577, -2.13998, -1.28245,  0.78955, 1.04223}, {-2.69084, 2.00648, -3.41166, -4, 0.039162, 7.89914, -6.455, -6.29689} };
float OutputWeights[HiddenNodes+1] = {-5.66203, 4.56084, -9.06113, -8.41671, 7.66029, 8.66586, 16.4839, -7.64896, -1.07353}; 
float Input[InputNodes];
int pwm_output = 0;

void setup() {
  Serial.begin(9600);
  while (!Serial)
    delay(10);
  
   // Setup IMU
   mpu.begin();

   // Setup SD card
   SD.begin(sdPort);
}

void loop() {
  mpu.getEvent(&accelerometer, &gyro, &temp);

  general = SD.open("rob.txt", FILE_WRITE);
  general.print(gyro.gyro.y);
  general.print("\t");
  general.print(accelerometer.acceleration.x);
  general.print("\t");
  general.print(accelerometer.acceleration.z);
  general.println("\t");
  general.close();
  
  Input[0] = gyro.gyro.y;
  Input[1] = accelerometer.acceleration.x;
  Input[2] = accelerometer.acceleration.z;

  neural_network();
  // The output from the neural network has a range from 0 and 1 and needs to be translated to PWM output range
  pwm_output = map((int) (output * 100), 0, 100, -255, 255);
  setMotor(pwm_output);

  // Log
  general = SD.open("nn.txt", FILE_WRITE);
  general.println(output);
  general.close();

  delay(timeDelay);
}

/*
 * Runs the neural network by going through the Hiiden and Output Weights 
 * and using the Input array that was updated before the method was called.
 */
void neural_network() {
  for(int i = 0 ; i < HiddenNodes ; i++ ) {    
      Accum = HiddenWeights[InputNodes][i] ;
      for(int j = 0 ; j < InputNodes ; j++ ) {
        Accum += Input[j] * HiddenWeights[j][i] ;
      }
      Hidden[i] = 1.0/(1.0 + exp(-Accum)) ;
  }
  Accum = OutputWeights[HiddenNodes];
  for(int j = 0 ; j < HiddenNodes ; j++ ) {
    Accum += Hidden[j] * OutputWeights[j];
  }
  output = 1.0/(1.0 + exp(-Accum)) ; // *510 - 255
}

/*
 * Sets the speed of the motor based on the PWM (0-256) value provided by motorSpeed.
 */
void setMotor(int motorSpeed) {
  if(abs(motorSpeed) < staticFriction) {
    // The motor will still move at the lowest possible PWM value to overcome static friction if motorSpeed is less than the necessary PWM.
    motorSpeed = abs(motorSpeed) / motorSpeed * staticFriction;
  }
  analogWrite(enA, abs(motorSpeed));
  analogWrite(enB, abs(motorSpeed));

  // Switches the direction the motor spins based on the sign of motorSpeed.
  if(motorSpeed < 0) { 
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
  }
  else {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
  }
}
