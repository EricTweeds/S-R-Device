#include <SensorLib.h>

#define enA 9
#define enB 10
#define right1 6
#define right2 7
#define left1 5
#define left2 4

#define TRIG 11;
#define ECHO 12;

#define LED 13;

SensorLib sensorLib;

void setup() {
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);

  pinMode(right1, OUTPUT);
  pinMode(right2, OUTPUT);
  pinMode(left1, OUTPUT);
  pinMode(left2, OUTPUT);
  Serial.begin(9600);
}

void loop() {
  // Drive straight
  driveForward();

  // Drive until object is detected
  while(sensorLib.getSonarDistance(TRIG, ECHO) > 8) {}

  // Turn to avoid object
  turnLeft1();
  delay(3000);

  // Drive straight again for a bit
  driveForward();
  delay(5000);

  // Stop motors
  stopMotors();

  // Wait until fire is detected
  while(!sensorLib.candleDetected()) {}

  // Blink LED to indicate fire is detected
  sensorLib.toggleLED(LED);

  // continue with other sensors????

  delay(1000);
}

void driveForward(enA, enB, right1, right2, left1, left2) {
  // Write PWM to turn on motor (full power)
  analogWrite(enA, 255);
  analogWrite(enB, 255);

  digitalWrite(right1, HIGH);
  digitalWrite(right2, LOW);
  digitalWrite(left1, LOW);
  digitalWrite(left2, HIGH);
}

void stopMotors(enA, enB) {
  analogWrite(enA, 0);
  analogWrite(enB, 0);
}

void turnLeft1(enA, enB, right1, right2, left1, left2) {
  /*
  * Turn left by turning the left wheel backwards
  */

  analogWrite(enA, 255);
  analogWrite(enB, 255); 

  digitalWrite(right1, HIGH);
  digitalWrite(right2, LOW);
  digitalWrite(left1, HIGH);
  digitalWrite(left2, LOW);
}

void turnLeft2(enA, enB, right1, right2, left1, left2) {
  /*
  * Turn left by turning the left wheel off
  */

  analogWrite(enA, 255);
  analogWrite(enB, 0);

  digitalWrite(right1, HIGH);
  digitalWrite(right2, LOW);
}
