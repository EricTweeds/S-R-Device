#ifndef SonarLib_h
#define SonarLib_h

#include "Arduino.h"

class SonarLib
{
    private:
        int trigPin;
        int echoPin;
    public:
        SonarLib(int trig, int echo);
        float getDistance();
        float getAverageDistance(int numSamples);
}; 

#endif
