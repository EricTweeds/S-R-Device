#ifndef SensorLib_h
#define SensorLib_h

#include "Arduino.h"

struct RGB {
  int R;
  int G;
  int B;
};

class SensorLib
{
  public:
    SensorLib();
    float getAmbientT(int sensorPin);
    bool candleDetected(int sensorPin, float ambientT);
    void toggleLED(int pin);
    void logColourVal(RGB colour);
    RGB getColour();
    void setCSLights(bool on);
};

#endif