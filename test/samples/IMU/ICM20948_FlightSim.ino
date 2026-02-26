// ICM20948 Flight Simulator Sketch
// SparkFun ICM-20948 breakout + Arduino Nano V3
// I2C: SDA=A4, SCL=A5  |  Address: 0x69
// Open FlightSimulator.html and connect via Web Serial (Chrome/Edge)

#include <Adafruit_ICM20X.h>
#include <Adafruit_ICM20948.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_ICM20948 icm;

void setup(void) {
  Serial.begin(9600);
  while (!Serial) delay(10);

  Serial.println("ICM20948_FLIGHTSIM_READY");

  if (!icm.begin_I2C(0x69)) {
    Serial.println("ERR:CHIP_NOT_FOUND");
    while (1) delay(10);
  }

  Serial.println("ICM20948_FOUND");
}

void loop() {
  sensors_event_t accel, gyro, mag, temp;
  icm.getEvent(&accel, &gyro, &temp, &mag);

  // Output compact CSV for the flight simulator to parse:
  // AX,AY,AZ,GX,GY,GZ,MX,MY,MZ,TEMP
  Serial.print(accel.acceleration.x, 3); Serial.print(",");
  Serial.print(accel.acceleration.y, 3); Serial.print(",");
  Serial.print(accel.acceleration.z, 3); Serial.print(",");
  Serial.print(gyro.gyro.x, 4);         Serial.print(",");
  Serial.print(gyro.gyro.y, 4);         Serial.print(",");
  Serial.print(gyro.gyro.z, 4);         Serial.print(",");
  Serial.print(mag.magnetic.x, 2);      Serial.print(",");
  Serial.print(mag.magnetic.y, 2);      Serial.print(",");
  Serial.print(mag.magnetic.z, 2);      Serial.print(",");
  Serial.println(temp.temperature, 1);

  delay(50); // 20Hz
}
