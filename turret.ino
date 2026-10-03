#include <Servo.h>
#include <Wire.h>
#include <math.h>

// declare servos and position variables
Servo servo1, servo2;
int pos1 = 0;
int pos2 = 0;

// declare MPU6050 variables
const int MPU_ADDR = 0x68; // Who Am I register
int16_t ax, ay, az;
int verticalAngle;
uint8_t count = 0;
int angleSum = 0;
int angles[100];

void setup() {
  // servo setup
  servo1.attach(5); // D5 pin
  servo2.attach(4); // D4 pin
  servo1.write(90);
  servo2.write(90);
  delay(1000);
  servo1.write(180);
  servo2.write(180);

  // MPU setup
  Serial.begin(9600); // open serial port at 96 bps
  Wire.begin();

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); // PWR_MGMT_1 register address of MPU
  Wire.write(0); // Set to zero to wake up MPU6050
  Wire.endTransmission(true);
}

void loop() {
  
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B); // start reading at the accelerometer data address
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 6, true); // request 6 bytes from MPU

  ax = (Wire.read() << 8) | Wire.read();
  ay = (Wire.read() << 8) | Wire.read();
  az = (Wire.read() << 8) | Wire.read();

  float fax = (float) ax;
  float fay = (float) ay;
  float faz = (float) az;

  // Serial.print("X: "); Serial.print(fax);
  // Serial.print(" Y: "); Serial.print(fay);
  // Serial.print(" Z: "); Serial.println(faz);
  // Serial.print(" Roll: "); Serial.print(atan2f(fay, faz) * (180/M_PI));
  // Serial.print(" Pitch: "); Serial.println(atan2f(-fax, sqrtf(fay*fay + faz*faz)) * (180/M_PI));
  verticalAngle = -((180/M_PI) * atan2f(fay, faz)) + 90;
  servo1.write(verticalAngle);
  servo2.write(90 + (180/M_PI) * atan2f(-fax, sqrtf(fay*fay + faz*faz)));

  angles[count] = verticalAngle;
  angleSum += verticalAngle;
  count++;

  if (count >= 100) {
    int mean = angleSum / 100;
    Serial.print("Mean: "); Serial.print(mean);

    int16_t summation = 0;
    for (uint8_t i=0; i < 100; i++) {
      summation += (mean - angles[i]) * (mean - angles[i]);
    }
    int standardDeviation = sqrt(summation / 99);
    Serial.print(" Standard Deviation: "); Serial.println(standardDeviation);

    angleSum = 0;
    count = 0;
  }

  delay(100);
}
