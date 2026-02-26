#include <HCSR04.h>
#include <SparkFun_TB6612.h>

// Ultra sonic sensor with 1 motor example

#define AIN1 9
#define AIN2 8
#define PWM  5
#define STBY 6

byte triggerPin = 11;
byte echoPin = 12;

const int off_set_A = 1;
Motor motor1 = Motor(AIN1, AIN2, PWM, off_set_A, STBY);

void setup() {
  Serial.begin(9600);

  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);   // Enable motor driver

  HCSR04.begin(triggerPin, echoPin);
}

void loop() {

  double* distances = HCSR04.measureDistanceCm();
  double distance = distances[0];   // FIXED

  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");

  if (distance > 30 || distance < 0) {
    motor1.drive(200);
  } else {
    motor1.brake();
  }

  delay(200);
}
