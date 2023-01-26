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
// .04 for x axis offset
// .01 for z axis offset

uint8_t maxPWM = 250;
uint8_t minPWM = 80;

float dt;
unsigned long previousTime;

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

/* PID parameters */
double Kp = 2150.0;
double Kd = 20.0;
double Ki = 0.0;

double setpoint = 0.0;
double input; // IMU
double output; // Motor

PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

/* SD Card */
File general, imu_log, motor_log, x_log;
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

  // Figures out the current angle of the robot by multiplying the change in time by the angular velocity.
  dt = (millis() - previousTime) / 1000.;
  previousTime = millis();
  input = input + (gyro.gyro.y + gyroOffset) * dt;

  pid.Compute();
  
  setMotor(output);

  // Log
  general = SD.open("y_log.txt", FILE_WRITE);
  x_log = SD.open("x_log.txt", FILE_WRITE);

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

  x_log.print(gyro.gyro.y);
  x_log.print("\t");
  x_log.print(accelerometer.acceleration.x);
  x_log.print("\t");
  x_log.println(accelerometer.acceleration.z);
  
  motor_log = SD.open("motor.txt", FILE_WRITE);
  Serial.print("Output: ");
  Serial.print(output);
  Serial.println(",");
  motor_log.println(output);
  motor_log.close();
  
  // general.println(output);
  general.close();
  x_log.close();
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
