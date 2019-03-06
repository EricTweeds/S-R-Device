#include <SensorLib.h>

SensorLib sensorLib;

void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:
  sensorLib.toggleLED(2);
  delay(1000);

}
