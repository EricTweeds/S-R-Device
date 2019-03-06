#ifndef SensorLib_h
#define SensorLib_h

#include "Arduino.h"

class SensorLib
{
  public:
    SensorLib();
    float getAmbientT(int sensorPin);
    bool candleDetected(int sensorPin, float ambientT);
    void toggleLED(int pin);
};

#endif