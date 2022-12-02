#include <Adafruit_MPU6050.h> // https://www.arduino.cc/reference/en/libraries/adafruit-mpu6050/
#include <Adafruit_Sensor.h> // part of above library
#include <SPI.h>
#include <SD.h>
#include <Wire.h>


/* MPU */
Adafruit_MPU6050 mpu;
double offsetAmount = 0.04; // at stationary reads -0.04
double input = 0;
float dt;
unsigned long previousTime;
/* L298N Module */
// Right Motor
#define in1 8
#define in2 7
#define enA 9

// Left Motor
#define in3 6
#define in4 5
#define enB 3

// Motor Constants
int motorSpeed = 255;
int timeDelay = 500;

/* HC-05 */
int state = 0;
int flag = 0;

/* SD Card */
File file;

void setup(void) {
  Serial.begin(9600);
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens

  // Set up ports
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(13, OUTPUT);

  // IMU_setup();
  sdcard_test();
}

void loop() {
  // motor_test();
  // bluetooth_test();
  // IMU_test();
  // angle_test();
}

void IMU_setup() {
  Serial.println("Adafruit MPU6050 test!");

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
}

void angle_test() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  dt = (millis() - previousTime) / 1000.;
  previousTime = millis();
  input = input + (g.gyro.x) * dt;
  Serial.println(input);
}
// copied from example
void IMU_test() {
  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  /* Print out the values */
  Serial.print("Acceleration X: ");
  Serial.print(a.acceleration.x);
  Serial.print(", Y: ");
  Serial.print(a.acceleration.y);
  Serial.print(", Z: ");
  Serial.print(a.acceleration.z);
  Serial.println(" m/s^2");

  Serial.print("Rotation X: ");
  Serial.print(g.gyro.x);
  Serial.print(", Y: ");
  Serial.print(g.gyro.y);
  Serial.print(", Z: ");
  Serial.print(g.gyro.z);
  Serial.println(" rad/s");

  /*
  Serial.print("Angle Roll: ");
  Serial.print(g.gyro.roll);
  Serial.print(", Pitch: ");
  Serial.print(g.gyro.pitch);
  // Serial.print(", Azimuth: ");
  // Serial.print(g.gyro.azimuth); gives error?
  Serial.println(" rad");
  */

  Serial.print("Temperature: ");
  Serial.print(temp.temperature);
  Serial.println(" degC");

  Serial.println("");
  delay(500);  
}

void motor_test() {
  analogWrite(enA, motorSpeed);
  analogWrite(enB, motorSpeed); 
  // run motors clockwise
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);

   /* delay(timeDelay); */

  //run motors counterclockwise
  // digitalWrite(in1, LOW);
  // digitalWrite(in2, HIGH);
  
  // digitalWrite(in3, LOW);
  // digitalWrite(in4, HIGH);
}

void bluetooth_test() {
   if(Serial.available()>0) {
      state = Serial.read();
      flag = 0;
   }

   if (state == '0') {
      // digitalWrite(13, LOW);
      if(flag==0) {
          Serial.println("LED: off");
          flag = 1;
      }
   }
   else if (state == '1') {
    // digitalWrite(13, HIGH);
    if(flag==0) {
      Serial.println("LED: on");
      flag = 1;
      }
   }
}

void sdcard_test() {
  Serial.println("Initializing SD card...");
  SD.begin(4);
  Serial.println("Initialization done!");
  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  file = SD.open("test.txt", FILE_WRITE);
  // if the file opened okay, write to it:
  if (file) {
    Serial.println("Writing to test.txt...");
    file.println("This is a test file :)");
    file.println("testing 1, 2, 3.");
    for (int i = 0; i < 20; i++) {
      file.println(i);
    }
    // close the file:
    file.close();
    Serial.println("done.");
  } else {
    // if the file didn't open, print an error:
    Serial.println("error opening test.txt");
  }
}
