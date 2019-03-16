#include "Arduino.h"
#include "SensorLib.h"

SensorLib::SensorLib() {
    //constructor
}

float SensorLib::getAmbientT(int sensorPin) {
  /*
   * returns ambient Temperature of the surroundings.
   * To be called during start up of device.
   * Value needed for candleDetection.
   */
  const int WaitTime = 500; //milliseconds between each temperature scan.
  const int samples = 5; //Number of samples used in average
  
  int sensorSum = analogRead(sensorPin);

  for (int i = 1; i < samples; i++) {
    delay(WaitTime);
    sensorSum += analogRead(sensorPin);
  }

  float sensorVal = sensorSum/(samples*1.0);
  float voltage = (sensorVal/1024.0) * 5.0;

  float temperature = (voltage - 0.5) * 100;

  return temperature;
}

bool SensorLib::candleDetected(int sensorPin, float ambientT) {
  /*
   * Requires an input of the ambiet temperature of the surrounding
   * to be taken during start up.
   * Takes a specified amount of temperature readings spaced out by
   * a wait time. Then averages the values to get a more accurate value.
   * Boolean returned whether difference is greater than threshold.
   */
  const int WaitTime = 500; //milliseconds between each temperature scan.
  const int samples = 3; //Number of samples used in average
  const float threshold = 2; //temperature change that signifies candle is present

  int sensorSum = analogRead(sensorPin);

  for (int i = 1; i < samples; i++) {
    delay(WaitTime);
    sensorSum += analogRead(sensorPin);
  }

  float sensorVal = sensorSum/(samples*1.0);
  float voltage = (sensorVal/1024.0) * 5.0;

  float temperature = (voltage - 0.5) * 100;

  //Serial.println(temperature);
  return (temperature - ambientT) > threshold;
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
