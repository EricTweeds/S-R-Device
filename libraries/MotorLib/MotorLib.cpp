#include "Arduino.h"
#include "MotorLib.h"

MotorLib::MotorLib(Motor right, Motor left) {
    leftMotor = left;
    rightMotor = right;

    pinMode(leftMotor.enablePin, OUTPUT);
    pinMode(rightMotor.enablePin, OUTPUT);

    pinMode(leftMotor.outPin1, OUTPUT);
    pinMode(leftMotor.outPin2, OUTPUT);
    pinMode(rightMotor.outPin1, OUTPUT);
    pinMode(rightMotor.outPin2, OUTPUT);
}

void MotorLib::driveForward() {
    analogWrite(leftMotor.enablePin, 255);
    analogWrite(rightMotor.enablePin, 255);

    digitalWrite(rightMotor.outPin1, HIGH);
    digitalWrite(rightMotor.outPin2, LOW);
    digitalWrite(leftMotor.outPin1, LOW);
    digitalWrite(leftMotor.outPin2, HIGH);
}

void MotorLib::driveBackwards() {
    analogWrite(leftMotor.enablePin, 255);
    analogWrite(rightMotor.enablePin, 255);

    digitalWrite(rightMotor.outPin1, LOW);
    digitalWrite(rightMotor.outPin2, HIGH);
    digitalWrite(leftMotor.outPin1, HIGH);
    digitalWrite(leftMotor.outPin2, LOW);
}

void MotorLib::turn90Left() {
    analogWrite(leftMotor.enablePin, 255);
    analogWrite(rightMotor.enablePin, 255);

    digitalWrite(rightMotor.outPin1, LOW);
    digitalWrite(rightMotor.outPin2, HIGH);
    digitalWrite(leftMotor.outPin1, LOW);
    digitalWrite(leftMotor.outPin2, HIGH);
}

void MotorLib::turn90Right() {
    analogWrite(leftMotor.enablePin, 255);
    analogWrite(rightMotor.enablePin, 255);

    digitalWrite(rightMotor.outPin1, HIGH);
    digitalWrite(rightMotor.outPin2, LOW);
    digitalWrite(leftMotor.outPin1, HIGH);
    digitalWrite(leftMotor.outPin2, LOW);
}
