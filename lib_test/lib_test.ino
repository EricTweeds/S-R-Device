#include <SensorLib.h>

SensorLib sensorLib;

void setup() {
  Serial.begin(9600);
  sensorLib.InitializeMagnetSensor();
}

void loop() {
  if(sensorLib.IsMagnet()) {
    Serial.println("detected");  
  }
  delay(1000);

}
