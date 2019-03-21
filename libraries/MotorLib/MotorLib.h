#ifndef MotorLib_h
#define MotorLib_h

#include "Arduino.h"

struct Motor {
    int enablePin;
    int outPin1;
    int outPin2;
};

class MotorLib
{
    private:
        Motor leftMotor;
        Motor rightMotor;
    public:
        MotorLib(Motor right, Motor left);
        void updateSpeeds(int leftSpeed, int rightSpeed);
        void setDirectionLeft();
        void setDirectionRight();
        void driveForward();
        void driveBackwards();
        void turnLeft();
        void turnRight();
}; 

#endif
