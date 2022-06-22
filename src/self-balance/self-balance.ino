#include <PID_v1.h> // https://www.arduino.cc/reference/en/libraries/pid/
#include <Adafruit_MPU6050.h> // https://www.arduino.cc/reference/en/libraries/adafruit-mpu6050/
#include <Adafruit_Sensor.h> // part of above library
#include <Wire.h>

// MPU parameters
Adafruit_MPU6050 mpu;
sensors_event_t accelerometer, gyro, temp;

uint8_t maxPWM = 250;
uint8_t minPWM = 80;

// PID parameters
double Kp = 0.0;
double Ki = 0.0;
double Kd = 0.0;

double setpoint = 0.0;
double input; // MPU6050
double output; // Motor

PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

void setup(void) {
  Serial.begin(115200);
  while (!Serial)
    delay(10);
  
   // Setup MPU
   mpu.begin();
   
   // Setup PID
   pid.SetMode(AUTOMATIC);
   pid.SetSampleTime(10);
   pid.SetOutputLimits(-255, 255);
   pid.SetTunings(Kp, Ki, Kd);
}

void loop() {
  mpu.getEvent(&accelerometer, &gyro, &temp); 

  //copied from old code need to understand what constants mean
  double z_accel = accelerometer.acceleration.z + 2.08;
  double y_accel = accelerometer.acceleration.y + 0.37;
  double x_accel = accelerometer.acceleration.x - 0.81;

  double angle_accel = atan(z_accel / y_accel);
  double angle_gyro = gyro.gyro.roll; // use roll as bot rotates forward

  // angle = (1 - alpha) * (angle + angle_gyro) + alpha*angle_accel; // end of copy paste
  
  // PID
  // input = mpu's input
  
  pid.Compute();

  analogWrite(0, output); // needs to be updated accordingly
 }