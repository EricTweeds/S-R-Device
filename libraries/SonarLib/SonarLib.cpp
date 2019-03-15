#include "Arduino.h"
#include "SonarLib.h"

SonarLib::SonarLib(int trig, int echo) {
    trigPin = trig;
    echoPin = echo;

    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);
}

float SonarLib::getDistance() {
    /*
    * returns distance from given sonar to object in cm.
    * Called when distance for sonar is required.
    */
    
    // Clear the trigPin
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    // Sets the trigPin on HIGH state for 10 micro seconds
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    // Reads the echoPin, returns the sound wave travel time in microseconds
    long duration = pulseIn(echoPin, HIGH);
    // Calculating the distance
    float distance = duration * 0.034 / 2;

    return distance;
}

float SonarLib::getAverageDistance(int numSamples) {
    float sum = 0;
    for (int i = 0; i < numSamples; i++) {
        sum += getDistance();
    }
    return sum / numSamples;
}
