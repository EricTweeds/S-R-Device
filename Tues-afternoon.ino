#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

#include "SonarLib.h"
#include "MotorLib.h"
#include "MapManagerLib.h"

MapManagerLib mapManager;

MPU6050 mpu;

#define flameSensorPin A0
#define fanPin 53
#define ledPin 30

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

struct Direction {
    bool isFacingX;
    bool isForward;
};

struct Position {
    int x;
    int y;
    Direction direction;
};

const int squareDistance = 30;

// Start forward facing y @ (0,0)
Direction currentDir = { true, true };
Position current = { 0, 0, currentDir };

Motor leftMotor = {ENA, Left1, Left2};
Motor rightMotor = {ENB, Right1, Right2};
MotorLib motors = MotorLib(rightMotor, leftMotor);

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

//SensorLib sensorLib;

float dt = 0.01;
float kp = 0.1, ki = 0.05, kd = 0.01;
float integral = 0.0;
float lastError = 0.0;
float error[20] = {0};
int index = 0;
float finalSP = 0;
float currentAngle = 0;
bool setDt = false;
bool done = false;
float startTime;

volatile bool mpuInterrupt = false; // indicates whether MPU interrupt pin has gone high
int numSamples = 50;
int prevHeatValue = 9999;
float minHeatAngle = 0;
bool switchedDir = false;

SonarLib frontSonar = SonarLib(trigF, echoF);
SonarLib rightSonar = SonarLib(trigR, echoR);
SonarLib leftSonar = SonarLib(trigL, echoL);
SonarLib rearSonar = SonarLib(trigB, echoB);

void dmpDataReady()
{
    mpuInterrupt = true;
}

void driveDistance(float distance)
{
    float frontStartingDis = frontSonar.getAverageDistance(5);
    float rearStartingDis = rearSonar.getAverageDistance(5);
    float currAngle = 0.0;
    float startingAngle;
    while (!getAngle(startingAngle))
    {
    }

    motors.driveForward();
    Serial.print("Starting Distance: ");
    Serial.println(frontStartingDis);
    float currentDistance = rearSonar.getAverageDistance(5);
    float prevDistance = currentDistance;
    // while (currentDistance > frontStartingDis) {
    //   frontStartingDis = frontSonar.getAverageDistance(5);
    //   currentDistance = frontSonar.getAverageDistance(5);
    // }
    while (currentDistance - rearStartingDis < distance)
    {
        // Serial.println(currentDistance);
        integral -= error[index] * dt;
        error[index] = currAngle;
        integral += error[index] * dt;
        float output = kp * error[index] + ki * integral;
        index = (index + 1) % 20;

        // Arduino map func => (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
        float ratio = output * 2;
        if (ratio > 0)
        {
            motors.updateSpeeds(255 / ratio, 255);
        }
        else if (ratio < 0)
        {
            motors.updateSpeeds(255, 255 / (-1 * ratio));
        }
        else
        {
            motors.updateSpeeds(255, 255);
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
        int counter = 0;
        do {
            currentDistance = rearSonar.getAverageDistance(5);
            counter++;
        } while (abs(prevDistance - currentDistance) > 10 * counter);
    }
    Serial.print("Final Location: ");
    Serial.println(currentDistance);
    motors.updateSpeeds(0, 0);
}

void turnController(float setP)
{
    bool turned = false;
    float currAngle = 0;
    float startingAngle;
    while (!getAngle(startingAngle))
    {
    }
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
            motors.updateSpeeds(255, 255);
            motors.setDirectionRight();
        }
        else if (output < -1)
        {
            motors.updateSpeeds(255, 255);
            motors.setDirectionLeft();
        }
        else if (ratio > 0)
        {
            motors.updateSpeeds(255 / ratio, 255);
            motors.setDirectionRight();
        }
        else if (ratio < 0)
        {
            motors.updateSpeeds(255, 255 / (-1 * ratio));
            motors.setDirectionLeft();
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

void setup()
{
    pinMode(53, INPUT);
    Serial.begin(9600);

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

    // Loop over an grab garbage values from accelerometer
    float value;
    for (int i = 2000; i > 0; i--)
    {
        if (!getAngle(value))
        {
            i++;
        }
        Serial.println("LOOPING");
    }
    Serial.println("DONE");
    delay(1000);
    while(digitalRead(53) != HIGH) {
      Serial.println(digitalRead(53));
    }
}

void loop()
{
    if (!dmpReady)
    {
        Serial.println("Accelerometer Not Working!!!!");
        return;
    }
    // driveToLocation(2,3);
    driveAvoidingObstacles(2, 1);
    // mapManager.printMap();
    // for (int i = 0; i < 2; i++) {
    //     mapCurrentLocation();
    //     mapManager.printMap();
    //     driveDistance(30);
    //     current.x += 1;
    //     delay(2000);
    // }
    // mapCurrentLocation();
    // mapManager.printMap();
//    delay(2000);
//     turnController(-90);
//     driveDistance(30, frontSonar);
//     turnController(90);
//     driveDistance(30, frontSonar);
//     current.x -= 1;
//     current.y -= 1;
//     mapCurrentLocation();

    while (true) {}
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
    }
}

void driveAvoidingObstacles(int targetX, int targetY) {
    // Cover special cases

    while (current.x != targetX || current.y != targetY) {
        int xCompensator = (targetX > current.x) ? 1 : -1;
        int yCompensator = (targetY > current.y) ? 1 : -1;
        int x = 0;
        int y = 0;

        while (mapManager.getMapValue(x, current.y) == mapManager.GROUND && (xCompensator * x + current.x) != targetX) {
            x++;
        }
        while (mapManager.getMapValue(current.x, y) == mapManager.GROUND && (yCompensator * y + current.y) != targetY) {
            y++;
        }

        if (x > y) {
            if (mapManager.getMapValue(x, current.y) != mapManager.GROUND) {
                x--;
            }
            int driveToX = xCompensator * x + current.x;
            Serial.print("Driving to: ");
            Serial.print(driveToX);
            Serial.print(" ");
            Serial.println(current.y);
            driveToLocation(driveToX, current.y);
        } else {
            if (mapManager.getMapValue(current.x, y) != mapManager.GROUND) {
                y--;
            }
            int driveToY = yCompensator * y + current.y;
            Serial.print("Driving to: ");
            Serial.print(current.x);
            Serial.print(" ");
            Serial.println(driveToY);
            driveToLocation(current.x, driveToY);
        }
    }
}

// Drives robot to given location
// Perhaps this function should go into the motor lib
void driveToLocation(int targetX, int targetY) {
    if (current.direction.isFacingX) {
        bool isFacingWrongDirection = (current.direction.isForward) ? current.x > targetX : current.x < targetX;
        if (isFacingWrongDirection && current.y == targetY) {
            // y is correct but facing wrong dir
            turnController(180);
            current.direction.isForward = !current.direction.isForward;
        } else if (isFacingWrongDirection) {
            int directionCompensator = (current.direction.isForward) ? 1 : -1;
            if (current.y > targetY) {
                turnController(-90.0 * directionCompensator);
                current.direction.isForward = !current.direction.isForward;
            } else if (current.y < targetY) {
                turnController(90.0 * directionCompensator);
            }
            current.direction.isFacingX = !current.direction.isFacingX;
        }
    } else {
        bool isFacingWrongDirection = (current.direction.isForward) ? current.y > targetY : current.y < targetY;
        if (isFacingWrongDirection && current.x == targetX) {
            // y is correct but facing wrong dir
            turnController(180);
            current.direction.isForward = !current.direction.isForward;
        } else if (isFacingWrongDirection) {
            int directionCompensator = (current.direction.isForward) ? 1 : -1;
            if (current.x > targetX) {
                turnController(90.0 * directionCompensator);
                current.direction.isForward = !current.direction.isForward;
            } else if (current.x < targetX) {
                turnController(-90.0 * directionCompensator);
            }
            current.direction.isFacingX = !current.direction.isFacingX;
        }
    }

    Serial.print("isFacingX: ");
    Serial.println(current.direction.isFacingX);
    delay(2000);

    // Drive until inline with target location
    if (current.direction.isFacingX) {
        float distance = (abs(current.x - targetX)) * squareDistance * 1.0;
        driveDistance(distance);
        // Update current location
        current.x = targetX;
    } else {
        float distance = (abs(current.y - targetY)) * squareDistance * 1.0;
        driveDistance(distance);
        // Update current location
        current.y = targetY;
    }

    Serial.println("Drove");
    delay(2000);

    int directionCompensator = (current.direction.isForward) ? 1 : -1;
    if (current.direction.isFacingX) {
        if (current.y > targetY) {
            turnController(-90.0 * directionCompensator);
            current.direction.isFacingX = !current.direction.isFacingX;
            current.direction.isForward = !current.direction.isForward; 
        } else if (current.y < targetY) {
            turnController(90.0 * directionCompensator);
            current.direction.isFacingX = !current.direction.isFacingX;
        }
    } else {
        if (current.x > targetX) {
            turnController(90.0 * directionCompensator);
            current.direction.isFacingX = !current.direction.isFacingX;
            current.direction.isForward = !current.direction.isForward;
        } else if (current.x < targetX) {
            turnController(-90.0 * directionCompensator);
            current.direction.isFacingX = !current.direction.isFacingX;
        }
    }

    Serial.print("isFacingX: ");
    Serial.println(current.direction.isFacingX);
    delay(2000);

    // Drive until inline with target location
    if (current.direction.isFacingX) {
        float distance = (abs(current.x - targetX)) * squareDistance * 1.0;
        driveDistance(distance);
        // Update current location
        current.x = targetX;
    } else {
        float distance = (abs(current.y - targetY)) * squareDistance * 1.0;
        driveDistance(distance);
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
    Serial.print("Right: ");
    Serial.println(rightDistance);
    Serial.print("Left: ");
    Serial.println(leftDistance);

    // Find number of squares before something has been detected
    // This is assuming the given distance from sonar reaches the end
    // of the last square
    int rightSquares = roundToSquare(rightDistance);
    int leftSquares = roundToSquare(leftDistance);
    Serial.print("Left Square: ");
    Serial.println(leftSquares);
    Serial.print("Right Square: ");
    Serial.println(rightSquares);

    if (current.direction.isFacingX) {
        // Facing x dir

        // Set pos and neg directions based on robot direction
        int positiveSquares, negativeSquares;
        if (current.direction.isForward) {
            positiveSquares = rightSquares;
            negativeSquares = leftSquares;
        } else {
            positiveSquares = leftSquares;
            negativeSquares = rightSquares;
        }

        // Set the last square to ITEM == something here
        // The last square is not guaranteed to be an objective, eg could be water square
        if (positiveSquares <= 2) {
            mapManager.setMapValue(current.x, current.y + positiveSquares, mapManager.ITEM);
        }
        if (negativeSquares <= 2) {
            mapManager.setMapValue(current.x, current.y - negativeSquares, mapManager.ITEM);
        }
    } else {
        // Facing y dir

        // Set pos and neg directions based on robot direction
        int positiveSquares, negativeSquares;
        if (current.direction.isForward) {
            positiveSquares = leftSquares;
            negativeSquares = rightSquares;
        } else {
            positiveSquares = rightSquares;
            negativeSquares = leftSquares;
        }

        // Set the last square to ITEM == something here
        // The last square is not guaranteed to be an objective, eg could be water square
        if (positiveSquares <= 2) {
            mapManager.setMapValue(current.x + positiveSquares, current.y, mapManager.ITEM);
        }
        if (negativeSquares <= 2) {
            mapManager.setMapValue(current.x - negativeSquares, current.y, mapManager.ITEM);
        }
    }
}
