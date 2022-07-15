#include <PID_v1.h> // https://www.arduino.cc/reference/en/libraries/pid/
#include <Adafruit_MPU6050.h> // https://www.arduino.cc/reference/en/libraries/adafruit-mpu6050/
#include <Adafruit_Sensor.h> // part of above library
#include <Wire.h>

/* IMU parameters */
Adafruit_MPU6050 mpu;
sensors_event_t accelerometer, gyro, temp;
double gyroOffset = 0.04;

uint8_t maxPWM = 250;
uint8_t minPWM = 80;

float dt;
unsigned long previousTime;

/* L298N Module */
// Right Motor
#define in1 6
#define in2 7
#define enA 5

// Left Motor
#define in3 8
#define in4 9
#define enB 10

/* PID parameters */
double Kp = 300.0;
double Ki = 0.0;
double Kd = 0.0;

double setpoint = 0.0;
double input; // IMU
double output; // Motor

PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

void setup(void) {
  Serial.begin(115200);
  while (!Serial)
    delay(10);
  
   // Setup IMU
   mpu.begin();
   
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
  input = input + (gyro.gyro.x + gyroOffset) * dt;
  
  pid.Compute();

  setMotor(output);

  // Plot
  Serial.print(setpoint);
  Serial.print("\t");
  Serial.print(input);
  Serial.print("\t");
  Serial.println(output);
 }

void setMotor(int motorSpeed) {
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
