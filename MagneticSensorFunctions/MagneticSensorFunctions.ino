#include "MPU9250.h"

// an MPU9250 object with the MPU-9250 sensor on I2C bus 0 with address 0x69 (our other accelerometer occupies 0x68)
MPU9250 IMU(Wire,0x69);

int ambientMagneticField;

// might need more adjustments from testing
const int MAGNETTHRESHOLD = 200;

int InitializeMagnetSensor() {
  int magnetSensorStatus;
  // start communication with IMU 
  magnetSensorStatus = IMU.begin();
  if (magnetSensorStatus < 0) 
    {return -1;}

  // get the ambient field
  ambientMagneticField = 0;
  ambientMagneticField = GetMagneticMagnitude();
  return 1;
}

// returns absolute magnitude
int GetMagneticMagnitude() {
  // read the sensor
  IMU.readSensor();

  // Turns the magnetic field vector into a magnitude
  return VectorMagnitude(IMU.getMagX_uT(), IMU.getMagY_uT(), IMU.getMagZ_uT());
}

// returns magnitude relative to the initial ambient conditions
int GetAdjustedMagneticMagnitude() {

  // read the sensor
  IMU.readSensor();

  // Turns the magnetic field vector into a magnitude
  return VectorMagnitude(IMU.getMagX_uT(), IMU.getMagY_uT(), IMU.getMagZ_uT()) - ambientMagneticField;
}

// uses the MAGNETTHRESHOLD value to determine if a magnet is nearby
bool IsMagnet(){
  int magnitude = GetAdjustedMagneticMagnitude();

  if(magnitude - MAGNETTHRESHOLD > 0){
    return true;
  }
  return false;
}

double VectorMagnitude(double a, double b, double c){
  return sqrt(pow(a,2) + pow(b,2) + pow(c,2));
}

void setup(){}

void loop(){}
