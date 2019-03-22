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
#define CANDLE 'C'

#include "Arduino.h"
#include "MPU9250.h"

struct RGB {
  float R;
  float G;
  float B;
  float C;
  float sum;
};

const int MAGNETTHRESHOLD = 100;

class SensorLib
{
  public:
    SensorLib();
    void toggleLED(int pin);
    void logColourVal(RGB colour);
    RGB _getColour();
    char determineObject();
    bool checkCandle();
    void setCSLights(bool on);
    void setRGBColour(int colour);
    void _setColour(bool r, bool g, bool b);
    int InitializeMagnetSensor();
    int _GetMagneticMagnitude();
    int _GetAdjustedMagneticMagnitude();
    bool IsMagnet();
    float _VectorMagnitude(float a, float b, float c);




  private:
    int ambientMagneticField;
};

#endif