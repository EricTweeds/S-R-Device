#include "SensorLib.h"

SensorLib sensors = SensorLib;

void setup()
{
    Serial.begin(9600);
}

void loop()
{
    // is square infront sand
    // turn on hall effect sensor
    // while on sand (distance travelled is < 30) or found magnet
        // drive straight slowly
        // poll hall effect for magnet
    // drive out of sand
}