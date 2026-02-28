#include <HCSR04.h>
#include <SparkFun_TB6612.h>

#define AIN1 9
#define AIN2 8
#define PWMA 5
#define STBY 6

#define BIN1 2
#define BIN2 4
#define PWMB 3

byte triggerPin = 12;
byte echoPin = 11;

const int offsetA = 1;
const int offsetB = 1;

Motor motor1(AIN1, AIN2, PWMA, offsetA, STBY);
Motor motor2(BIN1, BIN2, PWMB, offsetB, STBY);

void setup() {
  Serial.begin(9600);

  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);

  HCSR04.begin(triggerPin, echoPin);
}

void loop() {
  double* distances = HCSR04.measureDistanceCm();
  double distance = distances[0];

  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");

  if (distance > 30 && distance < 400) {
    motor1.drive(255);
    motor2.drive(255);
  } else {
    motor1.brake();
    motor2.brake();
  }

  delay(150);
}
