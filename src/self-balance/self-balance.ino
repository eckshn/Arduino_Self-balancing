#include <PID_v1.h> // https://www.arduino.cc/reference/en/libraries/pid/
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

/* PID parameters */
double Kp = 2150.0;
double Kd = 20.0;
double Ki = 0.0;

double setpoint = 0.0;
double input; // IMU
double output; // Motor

PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

/* SD Card */
File general, imu_log, motor_log;
#define sdPort 4

double runTime = 0;
void setup(void) {
  Serial.begin(9600);
  while (!Serial)
    delay(10);
  
   // Setup IMU
   mpu.begin();

   // Setup SD card
   SD.begin(sdPort);
   // file = SD.open("logs.txt", FILE_WRITE);
   // check if have to close
   
   // Setup PID
   pid.SetMode(AUTOMATIC);
   pid.SetSampleTime(10);
   pid.SetOutputLimits(-255, 255);
   pid.SetTunings(Kp, Ki, Kd);

   previousTime = millis();
}

void loop() {
  mpu.getEvent(&accelerometer, &gyro, &temp); 

  //copied from old code need to understand what constants mean
  // double z_accel = accelerometer.acceleration.z + 2.08;
  // double y_accel = accelerometer.acceleration.y + 0.37;
  // double x_accel = accelerometer.acceleration.x - 0.81;

  // double angle_accel = atan(z_accel / y_accel);
  // double angle_gyro = gyro.gyro.pitch; // use pitch as bot rotates forward
  // curAngle = (1 - alpha) * (curAngle + angle_gyro) + alpha*angle_accel; // end of copy paste
  dt = (millis() - previousTime) / 1000.;
  previousTime = millis();
  input = input + (gyro.gyro.y + gyroOffset) * dt;

  pid.Compute();
  
  setMotor(output);

  // Log
  general = SD.open("general.txt", FILE_WRITE);
  
  general.print(dt);
  general.print("\t");
  // general.close();

  Serial.print("Setpoint: ");
  Serial.println(input);
  Serial.print(",");
  // file.print(setpoint);
  Serial.print("\t");
  // file.print("\t");
  runTime += dt;
  if(abs(input) < 1.3) {
    imu_log = SD.open("imu.txt", FILE_WRITE);
    Serial.print("Current Gyro: ");
    Serial.print(gyro.gyro.y);
    Serial.print(",");
    Serial.print("\t");
    imu_log.print(runTime);
    imu_log.print("\t");
    imu_log.print(gyro.gyro.y);
    imu_log.print("\t");
    imu_log.print(accelerometer.acceleration.y);
    imu_log.print("\t");
    imu_log.print(accelerometer.acceleration.z);
    imu_log.print("\t");
    imu_log.println(input);
    // imu_log.print("\t");
    imu_log.close();
  }
  
  
  general.print(gyro.gyro.y);
  general.print("\t");
  general.print(accelerometer.acceleration.y);
  general.print("\t");
  general.println(accelerometer.acceleration.z);
  general.print("\t");


  motor_log = SD.open("motor.txt", FILE_WRITE);
  Serial.print("Output: ");
  Serial.print(output);
  Serial.println(",");
  motor_log.println(output);
  motor_log.close();
  
  general.println(output);
  general.close();
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
