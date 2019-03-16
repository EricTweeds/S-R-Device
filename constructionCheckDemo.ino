#include <SensorLib.h>

#define ENA 10
#define ENB 9
#define Right1 5
#define Right2 4
#define Left1 6
#define Left2 7

#define LED 2
#define THERMAL_PIN A0
#define TRIG 11
#define ECHO 12

SensorLib sensorLib;

void setup() {
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  pinMode(Right1, OUTPUT);
  pinMode(Right2, OUTPUT);
  pinMode(Left1, OUTPUT);
  pinMode(Left2, OUTPUT);

  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);

  Serial.begin(9600);

  drive();
//  driveWithSensors();
}
 
void loop() {
  
}

void driveWithSensors() {
  float am_Temp = sensorLib.getAmbientT(THERMAL_PIN);

  driveForward(ENA, ENB, Right1, Right2, Left1, Left2);

  // Drive until object is detected
  while(sensorLib.getSonarDistance(TRIG, ECHO) > 10) {}

  // Turn to avoid object
  turnLeft1(ENA, ENB, Right1, Right2, Left1, Left2);
  delay(3500);

  // Drive straight again for a bit
  driveForward(ENA, ENB, Right1, Right2, Left1, Left2);
  delay(5000);

  // Stop motors
  stopMotors(ENA, ENB);

//   Serial.println(am_Temp);

  // Wait until fire is detected
  while(!sensorLib.candleDetected(THERMAL_PIN, am_Temp)) {}

  // Blink LED to indicate fire is detected
  sensorLib.toggleLED(LED);
}

void drive() {
  driveForward(ENA, ENB, Right1, Right2, Left1, Left2);

  delay(4000);

  driveBackwards(ENA, ENB, Right1, Right2, Left1, Left2);

  delay(4000);

  turnRight1(ENA, ENB, Right1, Right2, Left1, Left2);

  delay(3500);

  turnLeft1(ENA, ENB, Right1, Right2, Left1, Left2);

  delay(3500);

  stopMotors(ENA, ENB);
}

void driveForward(int enA, int enB, int right1, int right2, int left1, int left2) {
  // Write PWM to turn on motor (full power)
  analogWrite(enA, 255);
  analogWrite(enB, 255);

  digitalWrite(left1, LOW);
  digitalWrite(left2, HIGH);
  digitalWrite(right1, HIGH);
  digitalWrite(right2, LOW);
}

void driveBackwards(int enA, int enB, int right1, int right2, int left1, int left2) {
  // Write PWM to turn on motor (full power)
  analogWrite(enA, 255);
  analogWrite(enB, 255);

  digitalWrite(left1, HIGH);
  digitalWrite(left2, LOW);
  digitalWrite(right1, LOW);
  digitalWrite(right2, HIGH);
}


void stopMotors(int enA, int enB) {
  analogWrite(enA, 0);
  analogWrite(enB, 0);
}

void turnRight1(int enA, int enB, int right1, int right2, int left1, int left2) {
  analogWrite(enA, 255);
  analogWrite(enB, 255);

  digitalWrite(left1, LOW);
  digitalWrite(left2, HIGH);
  digitalWrite(right1, LOW);
  digitalWrite(right2, HIGH);
}

void turnLeft1(int enA, int enB, int right1, int right2, int left1, int left2) {
  /*
  * Turn left by turning the left wheel backwards
  */

  analogWrite(enA, 255);
  analogWrite(enB, 255);

  digitalWrite(left1, HIGH);
  digitalWrite(left2, LOW);
  digitalWrite(right1, HIGH);
  digitalWrite(right2, LOW);
}

void turnLeft2(int enA, int enB, int right1, int right2, int left1, int left2) {
  /*
  * Turn left by turning the left wheel off
  */

  analogWrite(enA, 255);
  analogWrite(enB, 0);

  digitalWrite(right1, HIGH);
  digitalWrite(right2, LOW);
}
