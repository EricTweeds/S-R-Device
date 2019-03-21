#ifndef SensorLib_h
#define SensorLib_h

#define S0 50
#define S1 52
#define S2 53
#define S3 51
#define CSLights 49
#define sensorOut 12

#define OFF 0
#define RED 1
#define GREEN 2
#define BLUE 3
#define YELLOW 4
#define PURPLE 5
#define AQUA 6

#define UNKNOWN '?'
#define PERSON 'P'
#define GROUP 'A'

#include "Arduino.h"

struct RGB {
  float R;
  float G;
  float B;
  float C;
  float sum;
};

class SensorLib
{
  public:
    SensorLib();
    void toggleLED(int pin);
    void logColourVal(RGB colour);
    RGB _getColour();
    char determineObject();
    void setCSLights(bool on);
    void setRGBColour(int colour);
    void _setColour(bool r, bool g, bool b);
};

#endif