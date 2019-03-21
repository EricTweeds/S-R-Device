#ifndef SensorLib_h
#define SensorLib_h

#define S0 3
#define S1 4
#define S2 5
#define S3 6
#define CSLights 7
#define sensorOut 2

#define OFF 0
#define RED 1
#define GREEN 2
#define BLUE 3
#define YELLOW 4
#define PURPLE 5
#define AQUA 6

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
    void toggleLED(int pin);
    void logColourVal(RGB colour);
    RGB getColour();
    void setCSLights(bool on);
    void setRGBColour(int colour);
    void _setColour(bool r, bool g, bool b);
};

#endif