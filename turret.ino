#include <Servo.h>
#include <Wire.h>
#include <math.h>

const float ALPHA = 0.98;

typedef struct {
  float fax, fay, faz, fgx, fgy, fgz;
} data;

// declare servos and position variables
Servo servo1, servo2;
int pos1 = 0;
int pos2 = 0;

// declare MPU6050 variables
const int MPU_ADDR = 0x68; // Who Am I register
float fax, fay, faz;
float fgx, fgy, fgz;
data accelGyroData;
float accelVerticalAngle, accelHorizontalAngle;
float gyroBiasX, gyroBiasY, gyroBiasZ;
float roll, pitch;
unsigned long previousTime, currentTime;

uint8_t count = 0;
int angleSum = 0;
int angles[128];
int first128 = 0;



void setup() {
  // servo setup
  servo1.attach(5); // D5 pin
  servo2.attach(4); // D4 pin

  // initial wake-up signifier
  servo1.write(0);
  servo2.write(180);
  delay(1000);
  servo1.write(90);
  servo2.write(90);

  // MPU setup
  Serial.begin(115200); // open serial port at 115200 bps
  Wire.begin();

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); // PWR_MGMT_1 register address of MPU
  Wire.write(0); // Set to zero to wake up MPU6050
  Wire.endTransmission(true);

  accelGyroData = getAccelAndGyroData();
  roll = ((180/M_PI) * atan2f(accelGyroData.fay, accelGyroData.faz));
  pitch = (180/M_PI) * atan2f(-accelGyroData.fax, sqrtf(accelGyroData.fay*accelGyroData.fay + accelGyroData.faz*accelGyroData.faz));

  calibrateGyro();

  previousTime = micros();
}

void loop() {
  accelGyroData = getAccelAndGyroData();

  accelVerticalAngle = ((180/M_PI) * atan2f(accelGyroData.fay, accelGyroData.faz));
  accelHorizontalAngle = (180/M_PI) * atan2f(-accelGyroData.fax, sqrtf(accelGyroData.fay*accelGyroData.fay + accelGyroData.faz*accelGyroData.faz));

  currentTime = micros();
  float dt = (currentTime - previousTime) / 1000000.0;
  previousTime = currentTime;

  roll = (ALPHA * (roll + (accelGyroData.fgx - gyroBiasX) * dt) + (1.0 - ALPHA) * accelVerticalAngle);

  pitch = ALPHA * (pitch + (accelGyroData.fgy - gyroBiasY) * dt) + (1.0 - ALPHA) * accelHorizontalAngle;

  printRawData(accelGyroData, accelVerticalAngle, accelHorizontalAngle, roll, pitch);

  servo1.write(constrain(90 - (roll * 3), 0, 180));
  servo2.write(constrain(90 + (pitch), 0, 180));
  
  if (first128 == 1) {
    angleSum -= angles[count];
  }
  angles[count] = roll;
  angleSum += roll;
  count++;

  if (count >= 128) {
    int mean = angleSum / 128;
    

    long summation = 0;
    for (uint8_t i=0; i < 128; i++) {
      summation += (mean - angles[i]) * (mean - angles[i]);
    }
    int standardDeviation = sqrt(summation / 127);

    Serial.println("");
    Serial.print("Mean: "); Serial.print(mean);
    Serial.print(" Standard Deviation: "); Serial.println(standardDeviation);
    Serial.println("");

    count = 0;
    first128 = 1;
  }
}


data getAccelAndGyroData() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B); // start reading at the accelerometer data address
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 6, true); // request 6 bytes from MPU

  fax = (float) ((Wire.read() << 8) | Wire.read());
  fay = (float) ((Wire.read() << 8) | Wire.read());
  faz = (float) ((Wire.read() << 8) | Wire.read());

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x43); // start reading at the gryoscope data address
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 6, true); // request 6 bytes from MPU

  fgx = (float) (((Wire.read() << 8) | Wire.read())) / 131.0;
  fgy = (float) (((Wire.read() << 8) | Wire.read())) / 131.0;
  fgz = (float) (((Wire.read() << 8) | Wire.read())) / 131.0;

  data readings = {fax, fay, faz, fgx, fgy, fgz};

  return readings;
}

void calibrateGyro() {
  const int samples = 500;

  float sumX = 0;
  float sumY = 0;
  float sumZ = 0;

  Serial.println("Keep the MPU6050 still...");

  for (int i = 0; i < samples; i++) {
    data readings = getAccelAndGyroData();

    sumX += readings.fgx;
    sumY += readings.fgy;
    sumZ += readings.fgz;

    delay(2);
  }

  gyroBiasX = sumX / samples;
  gyroBiasY = sumY / samples;
  gyroBiasZ = sumZ / samples;

  Serial.print("Gyro bias X: ");
  Serial.println(gyroBiasX);
  Serial.print("Gyro bias Y: ");
  Serial.println(gyroBiasY);
  Serial.print("Gyro bias Z: ");
  Serial.println(gyroBiasZ);
}

void printRawData(data accelGyroData, float accelVerticalAngle, float accelHorizontalAngle, float roll, float pitch) {
  Serial.print("AX: ");
  Serial.print(accelGyroData.fax);
  Serial.print(" AY: ");
  Serial.print(accelGyroData.fay);
  Serial.print(" AZ: ");
  Serial.print(accelGyroData.faz);
  Serial.print(" GX: ");
  Serial.print(accelGyroData.fgx);
  Serial.print(" GY: ");
  Serial.print(accelGyroData.fgy);
  Serial.print(" GZ: ");
  Serial.print(accelGyroData.fgz);
  Serial.print(" AccelRoll: ");
  Serial.print(accelVerticalAngle);
  Serial.print(" AccelPitch: ");
  Serial.print(accelHorizontalAngle);
  Serial.print(" ax"); Serial.print(atan2f(fay, faz) * (180/M_PI));
  Serial.print(" Roll: "); Serial.print(atan2f(fay, faz) * (180/M_PI));
  Serial.print(" Pitch: "); Serial.println(atan2f(-fax, sqrtf(fay*fay + faz*faz)) * (180/M_PI));
}
