#ifndef AccelerometerLib_h
#define AccelerometerLib_h

#include "Arduino.h"

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

class AccelerometerLib
{
    private:
        MPU6050 mpu;

        // MPU control/status vars
        bool dmpReady;          // set true if DMP init was successful
        uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
        uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
        uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
        uint16_t fifoCount;     // count of all bytes currently in FIFO
        uint8_t fifoBuffer[64]; // FIFO storage buffer

        static volatile bool mpuInterrupt;
        bool getAcceleration(float &accel);
        static void dmpDataReady();

    public:
        AccelerometerLib(int interruptPin);
        bool getYaw(float &yaw);
        bool getAcceleration(VectorInt16 &aaReal);
        bool getSideAcceleration(float &accel);
        bool getForwardAcceleration(float &accel);
};

#endif