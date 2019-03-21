// define pins
#define TRIG 9;
#define ECHO 10;

void setup() {
  pinMode(TRIG, OUTPUT); // Sets the trigPin as an Output
  pinMode(ECHO, INPUT); // Sets the echoPin as an Input
  Serial.begin(9600); // Starts the serial communication
}

void loop() {
  // Calculating the distance
  float distance = getSonarDistance(TRIG, ECHO);
  // Prints the distance on the Serial Monitor
  Serial.print("Distance: ");
  Serial.println(distance);
  // Delay so that distance value is readable
  delay(2000);
}

float getSonarDistance(int trigPin, int echoPin) {
  /*
   * returns distance from given sonar to object in cm.
   * Called when distance for sonar is required.
   */
  
  // Clear the trigPin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  long duration = pulseIn(echoPin, HIGH);
  // Calculating the distance
  float distance = duration * 0.034 / 2;

  return distance;
}

// #include "SonarLib.h"

// // Front
// #define echoF 25
// #define trigF 24
// // Left
// #define echoL 23
// #define trigL 22
// // Right
// #define echoR 27
// #define trigR 26

// SonarLib s = SonarLib(trigF, echoF);

// void setup()
// {
//   Serial.begin(9600);
// }

// void loop()
// {
//   float distance = s.getDistance();
//   Serial.println(distance);
//   delay(500);
// }
