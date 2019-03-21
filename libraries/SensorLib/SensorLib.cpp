#include "Arduino.h"
#include "SensorLib.h"

SensorLib::SensorLib() {
    //constructor
}

void SensorLib::toggleLED(int pin) {
  //Toggles the output of a digital pin
  pinMode(pin, OUTPUT);
  if (digitalRead(pin) == HIGH) {
    digitalWrite(pin, LOW);
  } else {
    digitalWrite(pin, HIGH);
  }
}

void SensorLib::logColourVal(RGB colour) {
  Serial.print("R=");
  Serial.println(colour.R);
  Serial.print("G=");
  Serial.println(colour.G);
  Serial.print("B=");
  Serial.println(colour.B);
}

RGB SensorLib::getColour() {
  RGB colour;
  //red
  digitalWrite(S2,LOW);
  digitalWrite(S3,LOW);
  colour.R = pulseIn(sensorOut, LOW);

  //Green
  digitalWrite(S2,HIGH);
  digitalWrite(S3,HIGH);
  // Reading the output frequency
  colour.G = pulseIn(sensorOut, LOW);

  //Blue
  digitalWrite(S2,LOW);
  digitalWrite(S3,HIGH);
  // Reading the output frequency
  colour.B = pulseIn(sensorOut, LOW);

  return colour;
}

void SensorLib::setCSLights(bool on) {
  pinMode(CSLights, OUTPUT);
  if (on) {
    digitalWrite(CSLights, HIGH);
  } else {
    digitalWrite(CSLights, LOW);
  }
}

void SensorLib::setRGBColour(int colour) {
  if(colour == 0) { //off
    _setColour(false, false, false);
  } else if (colour == 1) { //red
    _setColour(true, false, false);
  } else if (colour == 2) { //green
    _setColour(false, true, false);
  } else if (colour == 3) { //blue
    _setColour(false, false, true);
  } else if (colour == 4) { //yellow
    _setColour(true, true, false);
  } else if (colour == 5) { //purple
    _setColour(true, false, true);
  } else if (colour == 6) { //aqua
    _setColour(false, true, true);
  }
}

void SensorLib::_setColour(bool r, bool g, bool b) {
  int redPin = 7;
  int greenPin = 5;
  int bluePin = 6;

  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);

  if (r) {
    digitalWrite(redPin, LOW);
  } else {
    digitalWrite(redPin, HIGH);
  }

  if (g) {
    digitalWrite(greenPin, LOW);
  } else {
    digitalWrite(greenPin, HIGH);
  }
  
  if (b) {
    digitalWrite(bluePin, LOW);
  } else {
    digitalWrite(bluePin, HIGH);
  } 
}