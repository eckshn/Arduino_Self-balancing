#include <Adafruit_MPU6050.h> // https://www.arduino.cc/reference/en/libraries/adafruit-mpu6050/
#include <Adafruit_Sensor.h> // part of above library
#include <SPI.h>
#include <SD.h>
#include <Wire.h>

/* IMU parameters */
Adafruit_MPU6050 mpu;
sensors_event_t accelerometer, gyro, temp;
double gyroOffset = 0.01;
// .04 x 
// .01 z

uint8_t maxPWM = 250;
uint8_t minPWM = 80;

float dt;
unsigned long previousTime;

/* L298N Module */
int staticFriction = 130; // TODO: Needs to be tested
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
const int InputNodes = 2; // 3 if acceleration.z
const int HiddenNodes = 8;
const int OutputNodes = 1;
float Accum;
float Hidden[HiddenNodes];
float output;
float HiddenWeights[InputNodes+1][HiddenNodes]; // TODO
float OutputWeights[HiddenNodes+1]; // TODO
float Input[InputNodes];

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  while (!Serial)
    delay(10);
  
   // Setup IMU
   mpu.begin();

   // Setup SD card
   SD.begin(sdPort);
}

void loop() {
  // put your main code here, to run repeatedly:
  mpu.getEvent(&accelerometer, &gyro, &temp);

  Input[0] = gyro.gyro.y;
  Input[1] = accelerometer.acceleration.y;
  
  neural_network();

  setMotor((int) output);
}

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

void setMotor(int motorSpeed) {
  if(abs(motorSpeed) < staticFriction) {
    motorSpeed = abs(motorSpeed) / motorSpeed * staticFriction;
  }
  analogWrite(enA, abs(motorSpeed));
  analogWrite(enB, abs(motorSpeed));
  
  if(motorSpeed < 0) { // may need to flip
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
