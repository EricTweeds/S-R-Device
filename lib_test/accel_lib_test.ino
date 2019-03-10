#include <AccelerometerLib.h>

AccelerometerLib accel;
float value;

void setup() {
  // put your setup code here, to run once:
    accel = AccelerometerLib(2);

    Serial.begin(9600);
}

void loop() {
  
    if (accel.getYaw(value)) {
        Serial.println(value)
    } else {
        Serial.println('Error')
    }
    
}
