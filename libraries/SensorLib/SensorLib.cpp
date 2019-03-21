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

RGB SensorLib::_getColour() {
  RGB colour;
  //red
  digitalWrite(S2,LOW);
  digitalWrite(S3,LOW);
  int red = pulseIn(sensorOut, LOW);
  colour.R = red;

  delay(100);

  //Green
  digitalWrite(S2,HIGH);
  digitalWrite(S3,HIGH);
  // Reading the output frequency
  int green = pulseIn(sensorOut, LOW);
  colour.G = green;
  delay(100);

  //Blue
  digitalWrite(S2,LOW);
  digitalWrite(S3,HIGH);
  // Reading the output frequency
  int blue = pulseIn(sensorOut, LOW);
  colour.B = blue;
  delay(100);

  digitalWrite(S2, HIGH);
  digitalWrite(S3, LOW);
  int c = pulseIn(sensorOut, LOW);
  
  float sum = (red + blue + c + green)*1.0;

  colour.R = red/sum*100;
  colour.G = green/sum*100;
  colour.B = blue/sum*100;
  colour.C = c/sum*100;
  
  colour.sum = sum;
  return colour;
}

char SensorLib::determineObject() {
  //call when close to object (~2.5-3in) returns what it is
  RGB colour = _getColour();
  if (colour.B > 39 && colour.G < 33) {
    return PERSON;
  } else if (colour.B < 37 && colour.G > 35) {
    return GROUP;
  } else {
    return UNKNOWN;
  }
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
  int redPin = 43;
  int greenPin = 42;
  int bluePin = 40;

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