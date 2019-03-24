#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

MPU6050 mpu;

#define INTERRUPT_PIN 2 // use pin 2 on Arduino Uno & most boards

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;   // [w, x, y, z]         quaternion container
float euler[3]; // [psi, theta, phi]    Euler angle container

volatile bool mpuInterrupt = false; // indicates whether MPU interrupt pin has gone high

#include "SonarLib.h"
#include "MotorLib.h"
#include "MapManagerLib.h"
#include "SensorLib.h"

SensorLib sensorLib;
MapManagerLib mapManager;

#define buttonPin 44
#define ledPin 30
#define fanPin 45
#define flameSensorPin A0

// Front
#define echoF 24
#define trigF 25
// Left
#define echoL 23
#define trigL 22
// Right
#define echoR 27
#define trigR 26
// Back
#define echoB 29
#define trigB 28

#define ENA 10
#define ENB 9
#define Right1 5
#define Right2 4
#define Left1 6
#define Left2 7

struct HousesFound {
    bool redHouseFound;
    bool yellowHouseFound;
};

struct Direction {
    bool isFacingX;
    bool isForward;
};

struct Position {
    int x;
    int y;
    Direction direction;
};

const int squareDistance = 25;
const int sonarAverages = 10;

// Direction S1Dir = { false, false };
// Position S1 = { 3, 5, currentDir };

// Direction S2Dir = { false, false };
// Position S1 = { 3, 5, currentDir };

// Start forward facing y @ (0,0)
const int startingX = 2;
const int startingY = 0;

Direction currentDir = { false, true };
Position current = {startingX, startingY, currentDir };
HousesFound houses = { false, false };

Motor leftMotor = {ENA, Left1, Left2};
Motor rightMotor = {ENB, Right1, Right2};
MotorLib motors = MotorLib(rightMotor, leftMotor);

SonarLib frontSonar = SonarLib(trigF, echoF);
SonarLib rightSonar = SonarLib(trigR, echoR);
SonarLib leftSonar = SonarLib(trigL, echoL);
SonarLib rearSonar = SonarLib(trigB, echoB);

bool lookingForHouses = false;

void dmpDataReady()
{
    mpuInterrupt = true;
}

void driveDistance(float distance)
{
    if (distance < 1) {
        return;
    } 

    Serial.print("Driving Distance: ");
    Serial.println(distance);

    float dt = 0.01;
    float kp = 0.02, ki = 0.05, kd = 0.01;
    float integral = 0.0;
    float lastError = 0.0;
    float error[20] = {0};
    int index = 0;

    int startingX = current.x;
    int startingY = current.y;

    float frontStartingDis = frontSonar.getAverageDistance(sonarAverages);
    float rearStartingDis = rearSonar.getAverageDistance(sonarAverages);
    float currAngle = 0.0;
    float startingAngle;
    float sumAngles = 0;
    int numSamples = 20;
    
    float currentDistance = rearSonar.getAverageDistance(sonarAverages);
    float prevDistance = currentDistance;
    int totalNumSquares = floor(distance / squareDistance);
    long numSquaresTraversed = 0;
    long numTimesMapped = 0;
    int distanceForMapping = 5;

    float sumDistance = 0;
    do {
        rearStartingDis = rearSonar.getAverageDistance(sonarAverages);
        frontStartingDis = frontSonar.getAverageDistance(sonarAverages);
        sumDistance = frontStartingDis + rearStartingDis;
        currentDistance = rearStartingDis;
        if (rearStartingDis > 500 && frontStartingDis < 190) {
            Serial.print(rearStartingDis);
            Serial.print("  ");
            Serial.println(frontStartingDis);
            motors.driveForward();
            delay(500);
            motors.updateSpeeds(0, 0);
        }
    } while (sumDistance > 165);

    for (int i = 0; i < numSamples; i++) {
        while (!getAngle(startingAngle)) {}
        sumAngles += startingAngle;
    }
    startingAngle = sumAngles / numSamples;

    Serial.print("Starting Distance: ");
    Serial.println(rearStartingDis);

    unsigned long startTime = millis();
    motors.driveForward();
    while (
        (currentDistance - rearStartingDis < distance || millis() - startTime < 5000 * totalNumSquares)
        &&
        frontSonar.getAverageDistance(sonarAverages) > 5 //do we need this? does this interfere with driving towards walls?
    )
    {
        if (currentDistance - rearStartingDis - (numSquaresTraversed * squareDistance) >= squareDistance) {
            if (current.direction.isFacingX) {
                if (current.direction.isForward) {
                    current.x++;
                } else {
                    current.x--;
                }
            } else {
                if (current.direction.isForward) {
                    current.y++;
                } else {
                    current.y--;
                }
            }
            numSquaresTraversed++;
        }

        // if(lookingForHouses) {
          
        //   char object = sensorLib.determineObject();
        //   if(object == PERSON || object == GROUP){
        //     Serial.print("Seeing Object: ");
        //     if (object == PERSON) {
        //         Serial.println("PERSON");
        //     } else {
        //         Serial.println("GROUP");
        //     }
        //     motors.updateSpeeds(0, 0);
        //     if(object == PERSON){
        //       sensorLib.setRGBColour(YELLOW);
        //       houses.yellowHouseFound = true;
        //     }
        //     else{
        //       sensorLib.setRGBColour(RED);
        //       houses.redHouseFound = true;
        //     }
        //     while(digitalRead(buttonPin) != HIGH) {}
        //     motors.updateSpeeds(255, 255);
        //   }
        // }
        // Map location every square
        // if (currentDistance - rearStartingDis - (numTimesMapped * distanceForMapping) >= distanceForMapping) {
        //     motors.updateSpeeds(0, 0);
        //     delay(50);
        //     mapCurrentLocation();
        //     numTimesMapped++;
        // }
        // Serial.println(currentDistance);
        integral -= error[index] * dt;
        error[index] = currAngle;
        integral += error[index] * dt;
        float output = kp * error[index] + ki * integral;
        index = (index + 1) % 20;
        // Serial.println(output);

        // Arduino map func => (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
        float ratio = output * 2;
        if (ratio > 0) 
        {
            motors.updateSpeeds(255 / (ratio + 1), 255); 
        }
        else if (ratio < 0)
        {
            motors.updateSpeeds(255, 255 / abs(ratio - 1));
        }
        else
        {
            motors.updateSpeeds(255, 255);
        }

        float angle;
        while (!getAngle(angle)) {}
        // Serial.println(angle);
        currAngle = angle - startingAngle;
        if (currAngle > 180) {
            currAngle -= 360;
        }
        else if (currAngle < -180) {
            currAngle += 360;
        }
        int counter = 0;
        do {
            currentDistance = rearSonar.getAverageDistance(sonarAverages);
            counter++;
        } while (abs(prevDistance - currentDistance) > 10 * counter);
        prevDistance = currentDistance;
    }
    if (frontSonar.getAverageDistance(sonarAverages) > 5) {
        int numSquares = floor(distance / squareDistance);
        if (current.direction.isFacingX) {
            if (current.direction.isForward) {
                current.x = startingX + numSquares;
            } else {
                current.x = startingX - numSquares;
            }
        } else {
            if (current.direction.isForward) {
                current.y = startingY + numSquares;
            } else {
                current.y = startingY - numSquares;
            }
        }
    }
    Serial.print("Final Location: ");
    Serial.println(currentDistance);
    motors.updateSpeeds(0, 0);
}

void turnController(float setP)
{
    Serial.print("Turning: ");
    Serial.println(setP);

    float dt = 0.01;
    float kp = 0.1, ki = 0.1, kd = 0.01;
    float integral = 0.0;
    float lastError = 0.0;
    float error[20] = {0};
    int index = 0;

    bool turned = false;
    float currAngle = 0;
    int numSamples = 10;
    float sumAngles = 0;
    float startingAngle;
    for (int i = 0; i < numSamples; i++) {
        while (!getAngle(startingAngle)) {}
        sumAngles += startingAngle;
    }
    startingAngle = sumAngles / numSamples;

    while (!turned)
    {
        integral -= error[index] * dt;
        error[index] = setP - currAngle;
        integral += error[index] * dt;
        float dError = (error[index] - lastError) / dt;
        float output = kp * error[index] + ki * integral;
        lastError = error[index];
        index = (index + 1) % 20;

        // Arduino map func => (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
        float ratio = output * 2;
        if (abs(output) < 0.02)
        {
            turned = true;
            motors.updateSpeeds(0, 0);
        }
        if (output > 1)
        {
            motors.setDirectionRight();
            motors.updateSpeeds(255, 255);
        }
        else if (output < -1)
        {
            motors.setDirectionLeft();
            motors.updateSpeeds(255, 255);
        }
        else if (ratio > 0)
        {
            motors.setDirectionRight();
            motors.updateSpeeds(255 / (ratio + 1), 255);
        }
        else if (ratio < 0)
        {
            motors.setDirectionLeft();
            motors.updateSpeeds(255, 255 / abs(ratio - 1));
        }
        else
        {
            turned = true;
            motors.updateSpeeds(0, 0);
        }

        float angle;
        while (!getAngle(angle))
        {
        }
        currAngle = angle - startingAngle;
        if (currAngle > 180)
        {
            currAngle -= 360;
        }
        else if (currAngle < -180)
        {
            currAngle += 360;
        }
    }
    motors.updateSpeeds(0, 0);
}

bool getAngle(float &angle)
{
    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize)
    {
        if (mpuInterrupt && fifoCount < packetSize)
        {
            // try to get out of the infinite loop
            fifoCount = mpu.getFIFOCount();
        }
    }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & _BV(MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount >= 1024)
    {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        fifoCount = mpu.getFIFOCount();

        return false;
        // otherwise, check for DMP data ready interrupt (this should happen frequently)
    }
    else if (mpuIntStatus & _BV(MPU6050_INTERRUPT_DMP_INT_BIT))
    {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize)
            fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);

        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetEuler(euler, &q);

        angle = euler[0] * 180.0 / M_PI;
        return true;
    }
}

void setupAccelerometer() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3V or Arduino
    // Pro Mini running at 3.3V, cannot handle this baud rate reliably due to
    // the baud timing being too misaligned with processor ticks. You must use
    // 38400 or slower in these cases, or use some kind of external separate
    // crystal solution for the UART timer.

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788);
    //mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0)
    {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
        Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
        Serial.println(F(")..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    }
    else
    {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
}

void setup()
{
    pinMode(buttonPin, INPUT);
    pinMode(ledPin, OUTPUT);
    pinMode(fanPin, OUTPUT);
    Serial.begin(9600);

    setupAccelerometer();
    
    sensorLib.InitializeMagnetSensor();

    sensorLib.setCSLights(true);

    // Loop over an grab garbage values from accelerometer
    float value;
    for (int i = 2000; i > 0; i--)
    {
        if (!getAngle(value))
        {
            i++;
        }
        Serial.println(i);
        int color = i % 2 == 0 ? RED : BLUE;
        sensorLib.setRGBColour(color);
    }
    Serial.println("DONE");
    sensorLib.setRGBColour(GREEN);
    while(digitalRead(buttonPin) != HIGH) {
      Serial.println(digitalRead(buttonPin));
    }
    sensorLib.setRGBColour(OFF);
}

void loop()
{
    if (!dmpReady)
    {
        Serial.println("Accelerometer Not Working!!!!");
        return;
    }

    // Serial.println(rightSonar.getAverageDistance(30));
    // Serial.println(leftSonar.getAverageDistance(30));
    // Serial.println("-----------");

    // current.x = 2;
    // current.y = 0;
    // current.direction.isFacingX = false;
    // current.direction.isForward = true;
    // findCandle();

    // while(digitalRead(buttonPin) != HIGH) {
    //     sensorLib.setRGBColour(AQUA);
    // }
    // sensorLib.setRGBColour(OFF);

    // current.x = 3;
    // current.y = 5;
    // current.direction.isFacingX = false;
    // current.direction.isForward = false;
    // findFood();

    // while(digitalRead(buttonPin) != HIGH) {
    //     sensorLib.setRGBColour(AQUA);
    // }
    // sensorLib.setRGBColour(OFF);

    // current.x = 2;
    // current.y = 0;
    // current.direction.isFacingX = false;
    // current.direction.isForward = true;
    // mapArea();

    // while(digitalRead(buttonPin) != HIGH) {
    //     sensorLib.setRGBColour(AQUA);
    // }
    // sensorLib.setRGBColour(OFF);

    driveDistance(120);

    while(true) {}

    // Serial.print(rearSonar.getAverageDistance(10));
    // Serial.print("  ");
    // Serial.println(frontSonar.getAverageDistance(10));

    // mapArea();
    // driveDistance(60);
    // findCandle();
    // driveDistance(60);
//    turnController(90);
//    delay(2000);
//    turnController(-90);
//      digitalWrite(fanPin, HIGH)
//    Serial.print("MAIN Current Location: ");
//    Serial.print(current.x);
//    Serial.print("   ");
//    Serial.println(current.y);
//    driveAvoidingObstacles(2,4);

    // current.x = 3;
    // current.y = 5;
    // current.direction.isForward = false;
    // current.direction.isFacingX = false;
    // findHouses();

    // while(digitalRead(buttonPin) != HIGH) {
    //     sensorLib.setRGBColour(AQUA);
    // }

    // current.x = 5;
    // current.y = 2;
    // current.direction.isForward = false;
    // current.direction.isFacingX = true;
    // findCandle();

    // while(digitalRead(buttonPin) != HIGH) {
    //     sensorLib.setRGBColour(AQUA);
    // }



    // current.x = 0;
    // current.y = 3;
    // current.direction.isForward = true;
    // current.direction.isFacingX = true;
    // findFood();

    // while(digitalRead(buttonPin) != HIGH) {
    //     sensorLib.setRGBColour(AQUA);
    // }

    // current.x = 2;
    // current.y = 0;
    // current.direction.isForward = true;
    // current.direction.isFacingX = false;
    // findHouses();

    // while(digitalRead(buttonPin) != HIGH) {
    //     sensorLib.setRGBColour(AQUA);
    // }


    // while (true) {}

    // driveDistance(60);
    // while(true) {}

    // Serial.print(leftSonar.getAverageDistance(10));
    // Serial.print("   ");
    // Serial.println(rightSonar.getAverageDistance(10));
    // Serial.print("   ");
    // Serial.print(frontSonar.getAverageDistance(10));
    // Serial.print("   ");
    // Serial.println(rearSonar.getAverageDistance(10));
    // delay(1000);
}

void findFood() {
    bool magnetDetected = false;
    int sandPitsVisited = 0;
    int targetX, targetY;
    int landingX, landingY;

    while (!magnetDetected && sandPitsVisited < 3) {
        targetX = 0;
        targetY = 0;
        landingX = 0;
        landingY = 0;

        findClosestNotVisited(targetX, targetY, current.x, current.y, mapManager.SAND);
        findBestSquareToLandAt(landingX, landingY, targetX, targetY);

        Serial.print("Landing location: ");
        Serial.print(landingX);
        Serial.print("   ");
        Serial.println(landingY);

        driveAvoidingObstacles(landingX, landingY);
        Serial.println("Got to landing");

        if (current.x > targetX && !current.direction.isFacingX) {
            if (current.direction.isForward) {
                turnController(90);
                current.direction.isForward = false;
            } else {
                turnController(-90);
                current.direction.isForward = false;
            }
            current.direction.isFacingX = true;
        } else if (current.x < targetX && !current.direction.isFacingX) {
            if (current.direction.isForward) {
                turnController(-90);
                current.direction.isForward = true;
            } else {
                turnController(90);
                current.direction.isForward = true;
            }
            current.direction.isFacingX = true;
        }

        if (current.y > targetY && current.direction.isFacingX) {
            if (current.direction.isForward) {
                turnController(-90);
                current.direction.isForward = false;
            } else {
                turnController(90);
                current.direction.isForward = false;
            }
            current.direction.isFacingX = false;
        } else if (current.y < targetY && current.direction.isFacingX) {
            if (current.direction.isForward) {
                turnController(90);
                current.direction.isForward = true;
            } else {
                turnController(-90);
                current.direction.isForward = true;
            }
            current.direction.isFacingX = false;
        }

        Serial.println("Food: Facing correct way");

        float rearStartingDis, frontStartingDis;
        float sumDistance = 0;

        do {
            rearStartingDis = rearSonar.getAverageDistance(sonarAverages);
            frontStartingDis = frontSonar.getAverageDistance(sonarAverages);
            sumDistance = frontStartingDis + rearStartingDis;
            if (rearStartingDis > 500 && frontStartingDis < 190) {
                Serial.print(rearStartingDis);
                Serial.print("  ");
                Serial.println(frontStartingDis);
                motors.driveForward();
                delay(500);
                motors.updateSpeeds(0, 0);
            }
        } while (sumDistance > 165);

        Serial.println("Entering Sand");

        motors.driveForward();

        while (rearSonar.getAverageDistance(10) - rearStartingDis < squareDistance * 2) {
            if (!magnetDetected) {
                magnetDetected = sensorLib.IsMagnet();
            }
        }

        motors.updateSpeeds(0, 0);

        Serial.println("Exited Sand");

        if (current.direction.isFacingX) {
            if (current.x > targetX) {
                current.x -= 2;
            } else {
                current.x += 2;
            }
        } else {
            if (current.y > targetY) {
                current.y -= 2;
            } else {
                current.y += 2;
            }
        }

        mapManager.setVisited(targetX, targetY);
        sandPitsVisited++;
    }

    if (magnetDetected) {
        sensorLib.setRGBColour(YELLOW);
        delay(1000);
    }
}

bool searchForCandle(float &offsetAngle, float minAngle) {
    bool found = false, switchedDir = false;
    float heatAngle, heatValue, prevHeatValue = 9999;

    float startingAngle;
    while (!getAngle(startingAngle)) {}
    float angle = startingAngle;
    int numTurns = -1;

    motors.turnLeft();

    while (!found && numTurns < 2) {
        while (!getAngle(angle)) {}
        if (angle - startingAngle < minAngle) {
            motors.turnRight();
        }

        if (angle > startingAngle - 0.5 && angle < startingAngle + 0.5) {
            numTurns++;
        }

        long sum = 0;
        int numSamples = 20;
        for (int i = 0; i < numSamples; i++) {
            sum += analogRead(flameSensorPin);
        }
        heatValue = sum / numSamples * 1.0;
        // Serial.print(prevHeatValue);
        // Serial.print("  ");
        // Serial.println(heatValue);
        if (heatValue - prevHeatValue > 5) {
            if (!switchedDir) {
                motors.turnRight();
                switchedDir = true;
            } else {
                motors.updateSpeeds(0, 0);
                while (!getAngle(heatAngle)) {}
                found = true;
            }
        }
        prevHeatValue = heatValue;
    }

    if (numTurns >= 2) {
        return false;
    }

    offsetAngle = startingAngle - heatAngle;
    if (offsetAngle > 180) {
        offsetAngle -= 360;
    } else if (offsetAngle < -180) {
        offsetAngle += 360;
    }

    // turn back to original orientation
    turnController(offsetAngle);
    offsetAngle *= -1;

    // Return relative angle pointing at the candle
    return true;
}

bool findClosestNotVisited(int &x, int &y, int currentX, int currentY, char value) {
    int minDistance = 100;
    bool foundValue = false;
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) {
            if (mapManager.getMapValue(i, j) == value && !mapManager.getVisited(i, j)) {
                foundValue = true;
                int distance = findNumSquaresToLocation(i, j, currentX, currentY, 0);
                if (distance < minDistance) {
                    x = i;
                    y = j;
                    minDistance = distance;
                }
            }
        }
    }
    return foundValue;
}

void findClosest(int &x, int &y, int currentX, int currentY, char value) {
    int minDistance = 100;
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) {
            if (mapManager.getMapValue(i, j) == value) {
                int distance = findNumSquaresToLocation(i, j, currentX, currentY, 0);
                if (distance < minDistance) {
                    x = i;
                    y = j;
                    minDistance = distance;
                }
            }
        }
    }
}

int findNumSquaresToLocation(int targetX, int targetY, int currentX, int currentY, int currentSum) {
    if (targetX == currentX && targetY == currentY) {
        return currentSum;
    }

    // Compensation for turns ==> want to add 1 to sum of squares
    // This does not take into the current direction of the robot
    // Cameron said this is OK
    if (targetX != currentX && targetY != currentY) {
        currentSum++;
    }

    // Cover special cases
    if (
        (currentX == 0 && currentY == 4)
        ||
        (currentX == 0 && currentY == 5)
        ||
        (currentX == 1 && currentY == 5)
    ) {
        if(
            (targetX == 0 && targetY == 4)
            ||
            (targetX == 0 && targetY == 5)
            ||
            (targetX == 1 && targetY == 5)
        ) {
            currentSum += abs(currentX - targetX) + abs(currentY -targetY);
        }
        currentSum += abs(currentX - 0) + abs(currentY - 3);
        return findNumSquaresToLocation(targetX, targetY, 0, 3, currentSum);
    }
    else if (
        (currentX == 4 && currentY == 0)
        ||
        (currentX == 5 && currentY == 0)
        ||
        (currentX == 5 && currentY == 1)
    ) {
        if(
            (targetX == 4 && targetY == 0)
            ||
            (targetX == 5 && targetY == 0)
            ||
            (targetX == 5 && targetY == 1)
        ) {
            currentSum += abs(currentX - targetX) + abs(currentY -targetY);
        }
        currentSum += abs(currentX - 5) + abs(currentY - 2);
        return findNumSquaresToLocation(targetX, targetY, 5, 2, currentSum);
    }
    else if (
        (currentX == 5 && currentY == 4)
        ||
        (currentX == 4 && currentY == 5)
        ||
        (currentX == 5 && currentY == 5)
    ) {
        if (
            (targetX == 5 && targetY == 4)
            ||
            (targetX == 4 && targetY == 5)
            ||
            (targetX == 5 && targetY == 5)
        ) {
            currentSum += abs(currentX - targetX) + abs(currentY -targetY);
        }
        currentSum += abs(currentX - 3) + abs(currentY - 5);
        return findNumSquaresToLocation(targetX, targetY, 3, 5, currentSum);
    }
    else if (
        (currentX == 0 && currentY == 0)
        ||
        (currentX == 0 && currentY == 1)
        ||
        (currentX == 1 && currentY == 0)
    ) {
        if (
            (targetX == 0 && targetY == 0)
            ||
            (targetX == 0 && targetY == 1)
            ||
            (targetX == 1 && targetY == 0)
        ) {
            currentSum += abs(currentX - targetX) + abs(currentY -targetY);
        }
        currentSum += abs(currentX - 2) + abs(currentY - 0);
        return findNumSquaresToLocation(targetX, targetY, 2, 0, currentSum);
    }
    else if (
        (currentX == 1 && currentY == 2)
    ) {
        currentSum += 1;
        return findNumSquaresToLocation(targetX, targetY, 1, 3, currentSum);
    }
    else if (
        (currentX == 2 && currentY == 0)
        ||
        (currentX == 2 && currentY == 1)
    ) {
        if ((targetX == 2 && (targetY == 1 || targetY == 2)) 
            ||
            ((targetX == 0 || targetX == 1) && targetY == 0)
        )
        {
            return currentSum + abs(currentX - targetX) + abs(currentY - targetY);
        }
        else if (targetX == 0 && targetY == 1){
            currentSum += abs(currentX - 0) + abs(currentY - 0);
            return findNumSquaresToLocation(targetX, targetY, 0, 0, currentSum);
        }
        
        currentSum += abs(currentX - 3) + abs(currentY - 1);
        return findNumSquaresToLocation(targetX, targetY, 3, 1, currentSum);
    }
    else if (
        (targetX == 0 && targetY == 4)
        ||
        (targetX == 0 && targetY == 5)
        ||
        (targetX == 1 && targetY == 5)
    ) {
        currentSum += abs(currentX - 0) + abs(currentY - 3);
        currentSum += abs(targetX - 0) + abs(targetY - 3);
        return currentSum;
    }
    else if (
        (targetX == 4 && targetY == 0)
        ||
        (targetX == 5 && targetY == 0)
        ||
        (targetX == 5 && targetY == 1)
    ) {
        currentSum += abs(currentX - 5) + abs(currentY - 2);
        currentSum += abs(targetX - 5) + abs(targetY - 2);
        return currentSum;
    }
    else if (
        (targetX == 5 && targetY == 4)
        ||
        (targetX == 4 && targetY == 5)
        ||
        (targetX == 5 && targetY == 5)
    ) {
        currentSum += abs(currentX - 3) + abs(currentY - 5);
        currentSum += abs(targetX - 3) + abs(targetY - 5);
        return currentSum;
    }
    else if (
        (targetX == 0 && targetY == 0)
        ||
        (targetX == 0 && targetY == 1)
        ||
        (targetX == 1 && targetY == 0)
    ) {
        currentSum += abs(currentX - 3) + abs(currentY - 1);
        currentSum += 2;
        currentSum += abs(2 - targetX) + abs(0 - targetY);
        return currentSum;
    }
    else if (
        (targetX == 1 && targetY == 2)
    ) {
        currentSum += abs(currentX - 1) + abs(currentY - 3);
        currentSum += 1;
        return currentSum;
    }
    else if (
        (targetX == 2 && targetY == 0)
        ||
        (targetX == 2 && targetY == 1)
    ) {
        currentSum += abs(currentX - 3) + abs(currentY - 1);
        currentSum += abs(targetX - 3) + abs(targetY - 1);
        return currentSum;
    }

    currentSum += abs(currentX - targetX) + abs(currentY - targetY);
    return currentSum;
}

void findBestSquareToLandAt(int &x, int &y, int targetX, int targetY) {
    int bestX = (current.x > targetX) ? targetX + 1 : targetX - 1;
    int bestY = (current.y > targetY) ? targetY + 1 : targetY - 1;
    int otherX = (current.x > targetX) ? targetX - 1 : targetX + 1;
    int otherY = (current.y > targetY) ? targetY - 1 : targetY + 1;

    char bestXGroundType = mapManager.getMapValue(bestX, targetY);
    char bestYGroundType = mapManager.getMapValue(targetX, bestY);
    char otherXGroundType = mapManager.getMapValue(otherX, targetY);
    char otherYGroundType = mapManager.getMapValue(targetX, otherY);

    int bestXDistance = 100;
    int bestYDistance = 100;
    int otherXDistance = 100;
    int otherYDistance = 100;

    int minDistance = 100;

    if (bestXGroundType == mapManager.GROUND) {
        bestXDistance = findNumSquaresToLocation(bestX, targetY, current.x, current.y, 0);
        if (bestXDistance < minDistance) {
            minDistance = bestXDistance;
            x = bestX;
            y = targetY;
        }
    }
    if (bestYGroundType == mapManager.GROUND) {
        bestYDistance = findNumSquaresToLocation(targetX, bestY, current.x, current.y, 0);
        if (bestYDistance < minDistance) {
            minDistance = bestYDistance;
            x = targetX;
            y = bestY;
        }
    }
    if (otherXGroundType == mapManager.GROUND) {
        otherXDistance = findNumSquaresToLocation(otherX, targetY, current.x, current.y, 0);
        if (otherXDistance < minDistance) {
            minDistance = otherXDistance;
            x = otherX;
            y = targetY;
        }
    }
    if (otherYGroundType == mapManager.GROUND) {
        otherYDistance = findNumSquaresToLocation(targetX, otherY, current.x, current.y, 0);
        if (otherYDistance < minDistance) {
            minDistance = otherYDistance;
            x = targetX;
            y = otherY;
        }
    }
}

void moveForwardOneSquare() {
    int targetX = current.x;
    int targetY = current.y;
    if (current.direction.isFacingX) {
        if (current.direction.isForward) {
            targetX++;
        } else {
            targetX--;
        }
    } else {
        if (current.direction.isForward) {
            targetY++;
        } else {
            targetY--;
        }
    }

    char groundType = mapManager.getMapValue(targetX, targetY);
    if (groundType == mapManager.GROUND || groundType == UNKNOWN) {
        driveDistance(squareDistance);
    } 
    // else if (groundType == mapManager.ITEM) {

    // } 
    else {
        if (current.direction.isFacingX) {
            int firstY = targetY + 1;
            int secondY = targetY - 1;
            int firstDis = findNumSquaresToLocation(targetX, firstY, current.x, current.y, 0);
            int secondDis = findNumSquaresToLocation(targetX, secondY, current.x, current.y, 0);
            if (firstDis == secondDis) {
                if (current.y <= 2) {
                    targetY = firstY;
                } else {
                    targetY = secondY;
                }
            } else if (firstDis > secondDis) {
                targetY = secondY;
            } else {
                targetY = firstY;
            }
        } else {
            int firstX = targetX + 1;
            int secondX = targetX - 1;
            int firstDis = findNumSquaresToLocation(firstX, targetY, current.x, current.y, 0);
            int secondDis = findNumSquaresToLocation(secondX, targetY, current.x, current.y, 0);
            if (firstDis == secondDis) {
                if (current.x <= 2) {
                    targetX = firstX;
                } else {
                    targetX = secondX;
                }
            } else if (firstDis > secondDis) {
                targetX = secondX;
            } else {
                targetX = firstX;
            }
        }
        driveAvoidingObstacles(targetX, targetY);
    }
}

void driveBackwards(float distance) {
    if (distance < 1) {
        return;
    }

    // add tolerance since driving towards sound waves
    distance += 5;
    float frontStartingDis;
    float rearStartingDis;
    int numSmaples = 20;
    float currentDistance;
    float sumDistance = 0;
    int numSquares = floor(distance / squareDistance);
    // rearStartingDis = rearSonar.getAverageDistance(sonarAverages);
    // frontStartingDis = frontSonar.getAverageDistance(sonarAverages);
    // currentDistance = rearStartingDis;
    // if (frontStartingDis > 500 && rearStartingDis < 190) {
    //     Serial.print(rearStartingDis);
    //     Serial.print("  ");
    //     Serial.println(frontStartingDis);
    //     motors.driveBackwards();
    //     delay(500);
    //     motors.updateSpeeds(0, 0);
    // }
   do {
       rearStartingDis = rearSonar.getAverageDistance(sonarAverages);
       frontStartingDis = frontSonar.getAverageDistance(sonarAverages);
       sumDistance = frontStartingDis + rearStartingDis;
       currentDistance = rearStartingDis;
       if (frontStartingDis > 500 && rearStartingDis < 190) {
           Serial.print(rearStartingDis);
           Serial.print("  ");
           Serial.println(frontStartingDis);
           motors.driveBackwards();
           delay(500);
           motors.updateSpeeds(0, 0);
       }
   } while (sumDistance > 165);

    unsigned long startTime = millis();
    motors.driveBackwards();
    while ((rearStartingDis - currentDistance < distance || millis() - startTime < 5000 * numSquares)) {
        currentDistance = rearSonar.getAverageDistance(sonarAverages);
    }
    motors.updateSpeeds(0, 0);

//    if (current.direction.isFacingX) {
//        if (current.direction.isForward) {
//            current.x -= numSquares;
//        } else {
//            current.x += numSquares;
//        }
//    } else {
//        if (current.direction.isForward) {
//            current.y -= numSquares;
//        } else {
//            current.y += numSquares;
//        }
//    }
}

void findCandle() {
    int numSquaresTravelled = 0;
    int flameValue = 999, prevFlameValue = 10000;

    do {
        if (numSquaresTravelled >= 5) {
            turnController(90);
            delay(500);
            turnController(90);
            current.direction.isForward = !current.direction.isForward;
        }
        numSquaresTravelled = 0;
        prevFlameValue = flameValue;
        flameValue = analogRead(flameSensorPin);

        while (numSquaresTravelled < 5 && flameValue < prevFlameValue + 15 && !sensorLib.checkCandle()) {

            moveForwardOneSquare();
            
            prevFlameValue = flameValue;
            flameValue = analogRead(flameSensorPin);
            Serial.print("Flame Value: ");
            Serial.println(flameValue);
            Serial.print("Prev Value: ");
            Serial.println(prevFlameValue);
            numSquaresTravelled++;
            Serial.print("numSquaresTravelled: ");
            Serial.println(numSquaresTravelled);
            Serial.print("Current Location: ");
            Serial.print(current.x);
            Serial.print("  ");
            Serial.println(current.y);
        }
        Serial.println("Out of inner loop");
    } while(numSquaresTravelled >= 5);

    Serial.println("Out of both loops");

    if (!sensorLib.checkCandle()) {
        driveBackwards(squareDistance);
        if (current.direction.isFacingX) {
          if (current.direction.isForward) {
            current.x--;
          } else {
            current.x++;
          }
        } else {
          if (current.direction.isForward) {
            current.y--;
          } else {
            current.y++;
          }
        }
        Serial.print("Went backwards: ");
        Serial.print(current.x);
        Serial.print("  ");
        Serial.println(current.y);
        Serial.println("Candle found");

        turnController(-90);
        if (current.direction.isFacingX) {
            current.direction.isForward = !current.direction.isForward;
        }
        current.direction.isFacingX = !current.direction.isFacingX;
    }
    float startingDistance;
    do {
      startingDistance = rearSonar.getAverageDistance(10);
    } while(startingDistance > 170);
    
    motors.driveForward();
    while (!sensorLib.checkCandle()) {}
    motors.updateSpeeds(0, 0);

    bool colorCandleOut = false;
    float frontDistance = frontSonar.getAverageDistance(10);
    while (!colorCandleOut && (frontDistance > 5 && frontDistance < 1300)) {
        digitalWrite(fanPin, HIGH);
        Serial.println("Fan on");
        motors.driveForward();
        delay(750);
        motors.updateSpeeds(0, 0);
        delay(400);
        for (int i = 0; i < 10; i++) {
            colorCandleOut = colorCandleOut || !sensorLib.checkCandle();
            delay(50);
        }
        frontDistance = frontSonar.getAverageDistance(10);
    }
    while (sensorLib.checkCandle()) {
        digitalWrite(fanPin, HIGH);
        Serial.println("Fan on");
        delay(2000);
        digitalWrite(fanPin, LOW);
        sensorLib.setRGBColour(YELLOW);
    }
    sensorLib.setRGBColour(OFF);

    sensorLib.setRGBColour(RED);

    float endingDistance;
    do {
      endingDistance = rearSonar.getAverageDistance(10);
    } while(endingDistance > 170);

    driveBackwards(endingDistance - startingDistance);
}

void mapArea() {
    int numSquaresTravelled = 0;
    int x, y;
    mapCurrentLocation();
    mapManager.printMap();
    while (numSquaresTravelled < 5) {
        moveForwardOneSquare();
        mapCurrentLocation();
        mapManager.printMap();
        
        numSquaresTravelled++;
        Serial.print("numSquaresTravelled: ");
        Serial.println(numSquaresTravelled);
    }
    mapManager.printMap();
    

    while (driveToItem()) {
        mapManager.printMap();
    }
}

bool driveToItem() {
    int targetX, targetY, driveX, driveY;
    if (!findClosestNotVisited(targetX, targetY, current.x, current.y, mapManager.ITEM)) {
        return false;
    }
    findBestSquareToLandAt(driveX, driveY, targetX, targetY);

    driveAvoidingObstacles(driveX, driveY);

    if (current.x > targetX && !current.direction.isFacingX) {
        if (current.direction.isForward) {
            turnController(90);
            current.direction.isForward = false;
        } else {
            turnController(-90);
            current.direction.isForward = false;
        }
        current.direction.isFacingX = true;
    } else if (current.x < targetX && !current.direction.isFacingX) {
        if (current.direction.isForward) {
            turnController(-90);
            current.direction.isForward = true;
        } else {
            turnController(90);
            current.direction.isForward = true;
        }
        current.direction.isFacingX = true;
    }

    if (current.y > targetY && current.direction.isFacingX) {
        if (current.direction.isForward) {
            turnController(-90);
            current.direction.isForward = false;
        } else {
            turnController(90);
            current.direction.isForward = false;
        }
        current.direction.isFacingX = false;
    } else if (current.y < targetY && current.direction.isFacingX) {
        if (current.direction.isForward) {
            turnController(90);
            current.direction.isForward = true;
        } else {
            turnController(-90);
            current.direction.isForward = true;
        }
        current.direction.isFacingX = false;
    }

    if (frontSonar.getAverageDistance(50) < 25) {
        float startingRearDistance = rearSonar.getAverageDistance(10);
        motors.driveForward();
        float frontDistance;
        do {
            frontDistance = frontSonar.getAverageDistance(10);
        } while (frontDistance > 5 && frontDistance < 1000);
        motors.updateSpeeds(0, 0);
        int loopGuard = 0;
        sensorLib.setRGBColour(PURPLE);
        char houseType;
        do {
            houseType = sensorLib.determineObject();
            loopGuard++;
        } while(houseType == UNKNOWN && loopGuard < 100);
        sensorLib.setRGBColour(OFF);
        Serial.print("houseType: ");
        Serial.println(houseType);
        if (houseType == GROUP) {
            sensorLib.setRGBColour(RED);
            delay(2000);
        } else if (houseType == PERSON) {
            sensorLib.setRGBColour(GREEN);
            delay(2000);
        }
        if(houseType != UNKNOWN){
            mapManager.setMapValue(targetX, targetY, houseType);
        }
        else{
            mapManager.setMapValue(targetX, targetY, mapManager.GROUND);
        }
        float endRearDistance = rearSonar.getAverageDistance(10);
        driveBackwards(startingRearDistance - endRearDistance);
    } else {
        mapManager.setMapValue(targetX, targetY, mapManager.GROUND);
    }

    sensorLib.setRGBColour(OFF);
    mapManager.setVisited(targetX, targetY);
    return true;
}

int roundToSquare(float distanceValue) {
    int tolerance = 3;
    if (distanceValue >= 0 && distanceValue < 30+tolerance) {
        return 1;
    }
    else if (distanceValue >= 30+tolerance && distanceValue < 60+tolerance) {
        return 2;
    }
    else if (distanceValue >= 60+tolerance && distanceValue < 90+tolerance) {
        return 3;
    }
    else if (distanceValue >= 90+tolerance && distanceValue < 120+tolerance) {
        return 4;
    }
    else if (distanceValue >= 120+tolerance && distanceValue < 150+tolerance) {
        return 5;
    }
    else if (distanceValue >= 150+tolerance && distanceValue < 180+tolerance) {
        return 6;
    } else  {
        return 10;
    }
}

void innerDriveAvoidingObstacles(int targetX, int targetY) {
    Serial.print("innerDriveAvoidingObstacles: ");
    Serial.print(targetX);
    Serial.print(" ");
    Serial.println(targetY);
    while (current.x != targetX || current.y != targetY) {
        Serial.print("Current Location: ");
        Serial.print(current.x);
        Serial.print(" ");
        Serial.println(current.y);
        int xCompensator = (targetX > current.x) ? 1 : -1;
        int yCompensator = (targetY > current.y) ? 1 : -1;
        int x = 0;
        int y = 0;

        while (mapManager.getMapValue(xCompensator * x + current.x, current.y) == mapManager.GROUND && (xCompensator * x + current.x) != targetX) {
            x++;
        }
        while (mapManager.getMapValue(current.x, yCompensator * y + current.y) == mapManager.GROUND && (yCompensator * y + current.y) != targetY) {
            y++;
        }

        Serial.print("Determined Values: ");
        Serial.print(x);
        Serial.print(" ");
        Serial.println(y);

        if (mapManager.getMapValue(xCompensator * x + current.x, current.y) != mapManager.GROUND) {
            x--;
        }
        if (mapManager.getMapValue(current.x, yCompensator * y + current.y) != mapManager.GROUND) {
            y--;
        }

        if (x > y || (xCompensator * x + current.x == targetX && x != 0)) {
            int driveToX = xCompensator * x + current.x;
            Serial.print("Driving to: ");
            Serial.print(driveToX);
            Serial.print(" ");
            Serial.println(current.y);
            driveToLocation(driveToX, current.y);
        } else {
            int driveToY = yCompensator * y + current.y;
            Serial.print("Driving to: ");
            Serial.print(current.x);
            Serial.print(" ");
            Serial.println(driveToY);
            driveToLocation(current.x, driveToY);
        }
    }
}

void driveAvoidingObstacles(int targetX, int targetY) {
    Serial.print("driveAvoidingObstacles: ");
    Serial.print(targetX);
    Serial.print(", ");
    Serial.println(targetY);

    if(current.x == targetX && current.y == targetY){
        return;
    }

    // Cover special cases
    if (
        (current.x == 0 && current.y == 4)
        ||
        (current.x == 0 && current.y == 5)
        ||
        (current.x == 1 && current.y == 5)
    ) {
        if (
            (targetX == 0 && targetY == 4)
            ||
            (targetX == 0 && targetY == 5)
            ||
            (targetX == 1 && targetY == 5)
        ) {
            innerDriveAvoidingObstacles(targetX, targetY);
        } else {
            innerDriveAvoidingObstacles(0, 3);
            driveAvoidingObstacles(targetX, targetY);
        }
    }
    else if (
        (current.x == 4 && current.y == 0)
        ||
        (current.x == 5 && current.y == 0)
        ||
        (current.x == 5 && current.y == 1)
    ) {
        if (
            (targetX == 4 && targetY == 0)
            ||
            (targetX == 5 && targetY == 0)
            ||
            (targetX == 5 && targetY == 1)
        ) {
            innerDriveAvoidingObstacles(targetX, targetY);
        } else {
            innerDriveAvoidingObstacles(5, 2);
            driveAvoidingObstacles(targetX, targetY);
        }
    }
    else if (
        (current.x == 5 && current.y == 4)
        ||
        (current.x == 4 && current.y == 5)
        ||
        (current.x == 5 && current.y == 5)
    ) {
        if (
            (targetX == 5 && targetY == 4)
            ||
            (targetX == 4 && targetY == 5)
            ||
            (targetX == 5 && targetY == 5)
        ) {
            innerDriveAvoidingObstacles(targetX, targetY);
        } else {
            innerDriveAvoidingObstacles(3, 5);
            driveAvoidingObstacles(targetX, targetY);
        }
    }
    else if (
        (current.x == 0 && current.y == 0)
        ||
        (current.x == 0 && current.y == 1)
        ||
        (current.x == 1 && current.y == 0)
    ) {
        if (
            (targetX == 0 && targetY == 0)
            ||
            (targetX == 0 && targetY == 1)
            ||
            (targetX == 1 && targetY == 0)
        ) {
            innerDriveAvoidingObstacles(targetX, targetY);
        } else {
            innerDriveAvoidingObstacles(2, 0);
            if (targetX == 2 && targetY == 1) {
                innerDriveAvoidingObstacles(2, 1);
            } else {
                driveAvoidingObstacles(targetX, targetY);
            }
        }
    }
    else if (
        (current.x == 1 && current.y == 2)
    ) {
        innerDriveAvoidingObstacles(1, 3);
        driveAvoidingObstacles(targetX, targetY);
    }
    else if (current.x == 2 && current.y == 1)
    {
        if (
            (targetX == 2 && targetY == 0)
            ||
            (targetX == 1 && targetY == 0)
            ||
            (targetX == 0 && targetY == 0)
        ) {
            innerDriveAvoidingObstacles(targetX, targetY);
        } else if (targetX == 0 && targetY == 1) {
            innerDriveAvoidingObstacles(0, 0);
            innerDriveAvoidingObstacles(targetX, targetY);
        } else {
            innerDriveAvoidingObstacles(3, 1);
            driveAvoidingObstacles(targetX, targetY);
        }
    }
    else if (current.x == 2 && current.y == 0)
    {
        if (
            (targetX == 2 && targetY == 1)
            ||
            (targetX == 1 && targetY == 0)
            ||
            (targetX == 0 && targetY == 0)
        ) {
            innerDriveAvoidingObstacles(targetX, targetY);
        } else if (targetX == 0 && targetY == 1) {
            innerDriveAvoidingObstacles(0, 0);
            innerDriveAvoidingObstacles(targetX, targetY);
        } else {
            innerDriveAvoidingObstacles(3, 1);
            driveAvoidingObstacles(targetX, targetY);
        }
    }
    else if (
        (targetX == 0 && targetY == 4)
        ||
        (targetX == 0 && targetY == 5)
        ||
        (targetX == 1 && targetY == 5)
    ) {
        innerDriveAvoidingObstacles(0, 3);
        innerDriveAvoidingObstacles(targetX, targetY);
    }
    else if (
        (targetX == 4 && targetY == 0)
        ||
        (targetX == 5 && targetY == 0)
        ||
        (targetX == 5 && targetY == 1)
    ) {
        innerDriveAvoidingObstacles(5, 2);
        innerDriveAvoidingObstacles(targetX, targetY);
    }
    else if (
        (targetX == 5 && targetY == 4)
        ||
        (targetX == 4 && targetY == 5)
        ||
        (targetX == 5 && targetY == 5)
    ) {
        innerDriveAvoidingObstacles(3, 5);
        innerDriveAvoidingObstacles(targetX, targetY);
    }
    else if (
        (targetX == 0 && targetY == 0)
        ||
        (targetX == 0 && targetY == 1)
        ||
        (targetX == 1 && targetY == 0)
    ) {
        innerDriveAvoidingObstacles(3, 1);
        innerDriveAvoidingObstacles(2, 0);
        innerDriveAvoidingObstacles(targetX, targetY);
    }
    else if (
        (targetX == 1 && targetY == 2)
    ) {
        innerDriveAvoidingObstacles(1, 3);
        innerDriveAvoidingObstacles(targetX, targetY);
    }
    else if (
        (targetX == 2 && targetY == 0)
        ||
        (targetX == 2 && targetY == 1)
    ) {
        innerDriveAvoidingObstacles(3, 1);
        innerDriveAvoidingObstacles(targetX, targetY);
    }
    else {
        innerDriveAvoidingObstacles(targetX, targetY);
    }   
}

// Drives robot to given location
// Perhaps this function should go into the motor lib
void driveToLocation(int targetX, int targetY) {
    if (current.direction.isFacingX) {
        bool isFacingWrongDirection = (current.direction.isForward) ? current.x > targetX : current.x < targetX;
        if (isFacingWrongDirection && current.y == targetY) {
            // y is correct but facing wrong dir
            turnController(90);
            delay(500);
            turnController(90);
            current.direction.isForward = !current.direction.isForward;
        } else if (isFacingWrongDirection) {
            int directionCompensator = (current.direction.isForward) ? 1 : -1;
            if (current.y > targetY) {
                turnController(-90.0 * directionCompensator);
                current.direction.isForward = false;
            } else if (current.y < targetY) {
                turnController(90.0 * directionCompensator);
                current.direction.isForward = true;
            }
            current.direction.isFacingX = false;
        }
    } else {
        bool isFacingWrongDirection = (current.direction.isForward) ? current.y > targetY : current.y < targetY;
        if (isFacingWrongDirection && current.x == targetX) {
            // y is correct but facing wrong dir
            turnController(90);
            delay(500);
            turnController(90);
            current.direction.isForward = !current.direction.isForward;
        } else if (isFacingWrongDirection) {
            int directionCompensator = (current.direction.isForward) ? 1 : -1;
            if (current.x > targetX) {
                turnController(90.0 * directionCompensator);
                current.direction.isForward = false;
            } else if (current.x < targetX) {
                turnController(-90.0 * directionCompensator);
                current.direction.isForward = true;
            }
            current.direction.isFacingX = true;
        }
    }

    Serial.print("isFacingX: ");
    Serial.println(current.direction.isFacingX);

    // Drive until inline with target location
    if (current.direction.isFacingX) {
        float distance = (abs(current.x - targetX)) * squareDistance * 1.0;
        driveDistance(distance);

        // Update visited
        int startIndex = min(current.x, targetX);
        int endIndex = max(current.x, targetX);
        for (int i = startIndex; i <= endIndex; i++) {
            mapManager.setVisited(i, current.y);
        }

        // Update current location
        current.x = targetX;
    } else {
        float distance = (abs(current.y - targetY)) * squareDistance * 1.0;
        driveDistance(distance);

        // Update visited
        int startIndex = min(current.y, targetY);
        int endIndex = max(current.y, targetY);
        for (int i = startIndex; i <= endIndex; i++) {
            mapManager.setVisited(current.x, i);
        }

        // Update current location
        current.y = targetY;
    }

    Serial.println("Drove");

    int directionCompensator = (current.direction.isForward) ? 1 : -1;
    if (current.direction.isFacingX) {
        if (current.y > targetY) {
            turnController(-90.0 * directionCompensator);
            current.direction.isFacingX = false;
            current.direction.isForward = false;
        } else if (current.y < targetY) {
            turnController(90.0 * directionCompensator);
            current.direction.isFacingX = false;
            current.direction.isForward = true;
        }
    } else {
        if (current.x > targetX) {
            turnController(90.0 * directionCompensator);
            current.direction.isFacingX = true;
            current.direction.isForward = false;
        } else if (current.x < targetX) {
            turnController(-90.0 * directionCompensator);
            current.direction.isFacingX = true;
            current.direction.isForward = true;
        }
    }

    Serial.print("isFacingX: ");
    Serial.println(current.direction.isFacingX);

    // Drive until inline with target location
    if (current.direction.isFacingX) {
        float distance = (abs(current.x - targetX)) * squareDistance * 1.0;
        driveDistance(distance);

        // Update visited
        int startIndex = min(current.x, targetX);
        int endIndex = max(current.x, targetX);
        for (int i = startIndex; i <= endIndex; i++) {
            mapManager.setVisited(i, current.y);
        }

        // Update current location
        current.x = targetX;
    } else {
        float distance = (abs(current.y - targetY)) * squareDistance * 1.0;
        driveDistance(distance);

        // Update visited
        int startIndex = min(current.y, targetY);
        int endIndex = max(current.y, targetY);
        for (int i = startIndex; i <= endIndex; i++) {
            mapManager.setVisited(current.x, i);
        }

        // Update current location
        current.y = targetY;
    }

    Serial.println("Now facing correct way");

    motors.updateSpeeds(0, 0);
}

void mapCurrentLocation() {
    // Get distances from sonars
    float rightDistance = rightSonar.getAverageDistance(10);
    float leftDistance = leftSonar.getAverageDistance(10);
    // float frontDistance = frontSonar.getAverageDistance(10);
    int countBadData = 0;

    while(rightDistance > 200 && countBadData < 10){
        rightDistance = rightSonar.getAverageDistance(10);
        countBadData++;
    }
    
    countBadData = 0;
    while(leftDistance > 200 && countBadData < 10){
        leftDistance = leftSonar.getAverageDistance(10);
        countBadData++;
    }

    // countBadData = 0;
    // while(frontDistance > 200 && countBadData < 10){
    //     frontDistance = frontSonar.getAverageDistance(10);
    //     countBadData++;
    // }
    
    Serial.print("Right: ");
    Serial.println(rightDistance);
    Serial.print("Left: ");
    Serial.println(leftDistance);
    // Serial.print("Front: ");
    // Serial.println(frontDistance);

    // Find number of squares before something has been detected
    // This is assuming the given distance from sonar reaches the end
    // of the last square
    int rightSquares = roundToSquare(rightDistance);
    int leftSquares = roundToSquare(leftDistance);
    // int frontSquares = roundToSquare(frontDistance);
    Serial.print("Left Square: ");
    Serial.println(leftSquares);
    Serial.print("Right Square: ");
    Serial.println(rightSquares);
    // Serial.print("Front Square: ");
    // Serial.println(frontSquares);

    if (current.direction.isFacingX) {
        // Facing x dir

        // Set pos and neg directions based on robot direction
        int positiveSquares, negativeSquares, frontX;
        if (current.direction.isForward) {
            positiveSquares = rightSquares;
            negativeSquares = leftSquares;
            // frontX = current.x + frontSquares;
        } else {
            positiveSquares = leftSquares;
            negativeSquares = rightSquares;
            // frontX = current.x - frontSquares;
        }

        // if (frontSquares == 1) {
        //     mapManager.setMapValue(frontX, current.y, mapManager.ITEM);
        // }

        if (positiveSquares <= 3) {
            mapManager.setMapValue(current.x, current.y + positiveSquares, mapManager.ITEM);
        }
        if (negativeSquares <= 3) {
            mapManager.setMapValue(current.x, current.y - negativeSquares, mapManager.ITEM);
        }
    } else {
        // Facing y dir

        // Set pos and neg directions based on robot direction
        int positiveSquares, negativeSquares, frontY;
        if (current.direction.isForward) {
            positiveSquares = leftSquares;
            negativeSquares = rightSquares;
            // frontY = current.y + frontSquares;
        } else {
            positiveSquares = rightSquares;
            negativeSquares = leftSquares;
            // frontY = current.y - frontSquares;
        }

        // if (frontSquares == 1) {
        //     mapManager.setMapValue(current.x, frontY, mapManager.ITEM);            
        // }

        if (positiveSquares <= 3) {
            mapManager.setMapValue(current.x + positiveSquares, current.y, mapManager.ITEM);
        }
        if (negativeSquares <= 3) {
            mapManager.setMapValue(current.x - negativeSquares, current.y, mapManager.ITEM);
        }
    }
}

void findHouses() {
  lookingForHouses = true;
  houses.redHouseFound = false;
  houses.yellowHouseFound = false;

  int startingLocation = 0;
  if (startingX == 3 && startingY == 5) {
      startingLocation = 0;
  } else if (startingX == 5 && startingY == 2) {
      startingLocation = 1;
  } else if (startingX == 2 && startingY == 0) {
      startingLocation = 2;
  } else if (startingX == 0 && startingY == 3) {
      startingLocation = 3;
  }
  // Change this value depending on start location
  initializeStartingLocation(startingLocation);

  int currentLocation = startingLocation;
  
  while(!houses.yellowHouseFound || !houses.redHouseFound){
    houseSegment(currentLocation);
    currentLocation++;
    // loop if max value reached
    if(currentLocation == 4){currentLocation == 0;}
  }
}

void initializeStartingLocation(int location){
  if(location == 0){
    turnController(90);
  }
  else if(location == 1){
    turnController(90);
  }
  else if(location == 2){
    turnController(90);
  }
  else if(location == 3){
    turnController(90);
  }
}

void houseSegment(int location){
  if(location == 0) {
      Serial.println("Track 0");
      if (houses.yellowHouseFound && houses.redHouseFound) {
          current.x = 3;
          current.y = 5;
          current.direction.isFacingX = true;
          current.direction.isForward = true;
          if (startingX == 3 && startingY == 5) {
            sensorLib.setRGBColour(BLUE);
          } else {
              driveToLocation(startingX, startingY);
              sensorLib.setRGBColour(BLUE);
          }
      }
      // go to 5,5
      driveDistance(60);

      // go to 5,4
      turnController(-90);
      driveDistance(20);

      // return to 5,5
      turnController(-90);
      turnController(-90);
      driveDistance(20);

      // go to 3,5
      turnController(90);
      driveDistance(60);

      // go to 3,3
      turnController(90);
      driveDistance(60);

      // go to 4,3
      turnController(90);
      driveDistance(30);

      // go to 4,2
      turnController(-90);
      driveDistance(30);

      // go to 5,2
      turnController(90);
      driveDistance(30);
      turnController(-90);
  }
  else if(location == 1){
      Serial.println("Track 1");
      if (houses.yellowHouseFound && houses.redHouseFound) {
          current.x = 5;
          current.y = 2;
          current.direction.isFacingX = false;
          current.direction.isForward = false;
          if (startingX == 5 && startingY == 2) {
            sensorLib.setRGBColour(BLUE);
          } else {
              driveToLocation(startingX, startingY);
              sensorLib.setRGBColour(BLUE);
          }
      }
      // go to 5,0
      driveDistance(60);

      // go to 4,0
      turnController(-90);
      driveDistance(20);

      // return to 5,0
      turnController(-90);
      turnController(-90);
      driveDistance(20);

      // go to 5,2
      turnController(90);
      driveDistance(60);

      // go to 3,2
      turnController(90);
      driveDistance(60);

      // go to 3,1
      turnController(90);
      driveDistance(30);

      // go to 2,1
      turnController(-90);
      driveDistance(30);

      // go to 2,0
      turnController(90);
      driveDistance(30);
      turnController(-90);
  }
  else if(location == 2) {
      Serial.println("Track 2");
      if (houses.yellowHouseFound && houses.redHouseFound) {
          current.x = 2;
          current.y = 0;
          current.direction.isFacingX = true;
          current.direction.isForward = false;
          if (startingX == 2 && startingY == 0) {
            sensorLib.setRGBColour(BLUE);
          } else {
              driveToLocation(startingX, startingY);
              sensorLib.setRGBColour(BLUE);
          }
      }
      // go to 0,0
      driveDistance(60);

      // go to 0,1
      turnController(-90);
      driveDistance(20);

      // return to 0,0
      turnController(-90);
      turnController(-90);
      driveDistance(20);

      // go to 2,0
      turnController(90);
      driveDistance(60);

      // go to 2,1
      turnController(90);
      driveDistance(30);

      // go to 3,1
      turnController(-90);
      driveDistance(30);

      // go to 3,3
      turnController(90);
      driveDistance(60);

      // go to 1,3
      turnController(90);
      driveDistance(60);

      // go to 1,2
      turnController(90);
      driveDistance(20);

      // go to 1,3
      turnController(-90);
      turnController(-90);
      driveDistance(20);

      // go to 0,3
      turnController(90);
      driveDistance(30);
      turnController(-90);
  }
  else if(location == 3) { 
      Serial.println("Track 2");
      if (houses.yellowHouseFound && houses.redHouseFound) {
          current.x = 0;
          current.y = 3;
          current.direction.isFacingX = false;
          current.direction.isForward = true;
          if (startingX == 0 && startingY == 3) {
            sensorLib.setRGBColour(BLUE);
          } else {
              driveToLocation(startingX, startingY);
              sensorLib.setRGBColour(BLUE);
          }
      }
      // go to 0,5
      driveDistance(60);

      // go to 1,5
      turnController(-90);
      driveDistance(20);

      // return to 0,5
      turnController(-90);
      turnController(-90);
      driveDistance(20);

      // go to 0,3
      turnController(90);
      driveDistance(60);

      // go to 2,3
      turnController(90);
      driveDistance(60);

      // go to 2,4
      turnController(90);
      driveDistance(30);

      // go to 3,4
      turnController(-90);
      driveDistance(30);

      // go to 3,5
      turnController(90);
      driveDistance(30);
      turnController(-90);
  }
}
