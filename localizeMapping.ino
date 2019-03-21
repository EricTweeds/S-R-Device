#include "SonarLib.h"
#include "MapManagerLib.h"
#include "MotorLib.h"

#include "I2Cdev.h"

 #include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

 // Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

#define INTERRUPT_PIN 2 // use pin 2 on Arduino Uno & most boards

MPU6050 mpu;

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
float dt = 0.01;
float kp = 0.1, ki = 0.05, kd = 0.01;
float integral = 0.0;
float lastError = 0.0;
float error[20] = {0};
int index = 0;

// Setup sonars
// Front
#define echoF 24
#define trigF 25
// Left
#define echoL 23
#define trigL 22
// Right
#define echoR 27
#define trigR 26

SonarLib frontSonar = SonarLib(trigF, echoF);
SonarLib leftSonar = SonarLib(trigL, echoL);
SonarLib rightSonar = SonarLib(trigR, echoR);

// Setup motors
#define ENA 10
#define ENB 9
#define Right1 5
#define Right2 4
#define Left1 6
#define Left2 7
Motor leftMotor = {ENA, Left1, Left2};
Motor rightMotor = {ENB, Right1, Right2};
MotorLib motors = MotorLib(rightMotor, leftMotor);

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
Direction currentDir = { false, true };
Position current = { 0, 0, currentDir };

MapManagerLib mapManager;

void setupAcclerometer() {
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
    delay(1000);
}

void setup() {
    Serial.begin(9600);
    setupAcclerometer();
    if (!dmpReady)
    {
        Serial.println("Accelerometer Not Working!!!!");
        while (true) {}
    }
}
 
void loop() {
    int x;
    int y;

    // x = 0;
    // y = 1;
    // driveToLocation(x, y);

    // current.x = 0;
    // current.y = 1;
    // x = 0;
    // y = 0;
    // driveToLocation(x, y);

    current.x = 2;
    current.y = 2;
    current.direction.isForward = true;
    current.direction.isFacingX = false;
    x = 3;
    y = 1;
    driveToLocation(x, y);

    Serial.println("Arrived");

    // current.x = 1;
    // current.y = 0;
    // current.direction.isForward = true;
    // current.direction.isFacingX = false;
    // x = 0;
    // y = 0;
    // driveToLocation(x, y);

    while (true) {}
}

void generalFunc() {
    MapManagerLib.setMapValue(current.x, current.y, MapManagerLib.FINISH);
    bool putOutCandle = false, foundFood = false, foundGroup = false, foundSolo = false, fedPeople = false;

    float frontDistance = frontSonar.getAverageDistance(5);
    if (frontDistance / squareDistance > 5.5) {
        for (int i = 0; i < 5; i++) {
            driveDistance(squareDistance, frontSonar);
            mapCurrentLocation();
        }
        turnController(180);
        driveDistance(squareDistance * 2, frontSonar);
        turnController(180);
    } else {
        // Drive to middle of area
        for (int i = 0; i < 3; i++) {
            driveDistance(squareDistance, frontSonar);
            mapCurrentLocation();
        }
    }

    int x, y;
    if (searchForCandle(x, y)) {
        driveToLocation(x, y);
    }

    while (!(putOutCandle && foundFood && foundGroup && foundSolo && fedPeople)) {

    }

    if (!MapManagerLib.getLocation(x, y, MapManagerLib.FINISH)) {
        // Uhoo not sure how this would happen
    }
    driveToLocation(x, y);
}

bool driveAndLookForFood() {
    // while (distanceTravelled < 30)
        // drive straigh slowly motors.driveForward();
        // motors.updateSpeeds(100, 100);
        // poll for magnet
        // if (foundMagnet) foundMagnet = true;
    // return foundMagnet
}

void driveAroundObject() {
    float frontDistance = frontSonar.getAverageDistance(5);
    float leftDistance = leftSonar.getAverageDistance(5);
    float rightDistance = rightSonar.getAverageDistance(5);
    float turnAngle;
    if (leftDistance > rightDistance) {
        // turn left
        turnAngle = -90.0;
        turnController(turnAngle);
    } else {
        // turn right
        turnAngle = 90.0;
        turnController(turnAngle);
    }
    driveDistance(squareDistance, frontSonar);
    turnController(-1.0 * turnAngle);
    driveDistance(floor(frontDistance / squareDistance) + 1, frontSonar);
    turnController(-1.0 * turnAngle);
    driveDistance(squareDistance, frontSonar);
    turnController(turnAngle);
}

void driveAndMap() {
    mapCurrentLocation();

    char groundInFront = SensorLib.getGroundInFront();
    switch (groundInFront) {
        case MapManagerLib.GROUND:
            driveDistance(squareDistance, frontSonar);
            break;
        case MapManagerLib.SAND:
            driveAndLookForFood();
            break;
        case MapManagerLib.WATER:
            driveAroundObject();
            break;
        // case MapManagerLib.:

        // case MapManagerLib.:

        // case MapManagerLib.:

        default:

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
                turnController(90.0 * directionCompensator);
            } else if (current.y < targetY) {
                turnController(-90.0 * directionCompensator);
                current.direction.isForward = !current.direction.isForward;
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
                turnController(-90.0 * directionCompensator);
                current.direction.isForward = !current.direction.isForward;
            } else if (current.x < targetX) {
                turnController(90.0 * directionCompensator);
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
        driveDistance(distance, frontSonar);
        // Update current location
        current.x = targetX;
    } else {
        float distance = (abs(current.y - targetY)) * squareDistance * 1.0;
        driveDistance(distance, frontSonar);
        // Update current location
        current.y = targetY;
    }

    Serial.println("Drove");
    delay(2000);

    int directionCompensator = (current.direction.isForward) ? 1 : -1;
    if (current.direction.isFacingX) {
        if (current.y > targetY) {
            turnController(90.0 * directionCompensator);
            current.direction.isFacingX = !current.direction.isFacingX;
        } else if (current.y < targetY) {
            turnController(-90.0 * directionCompensator);
            current.direction.isFacingX = !current.direction.isFacingX;
            current.direction.isForward = !current.direction.isForward;
        }
    } else {
        if (current.x > targetX) {
            turnController(-90.0 * directionCompensator);
            current.direction.isFacingX = !current.direction.isFacingX;
            current.direction.isForward = !current.direction.isForward;
        } else if (current.x < targetX) {
            turnController(90.0 * directionCompensator);
            current.direction.isFacingX = !current.direction.isFacingX;
        }
    }

    Serial.print("isFacingX: ");
    Serial.println(current.direction.isFacingX);
    delay(2000);

    // Drive until inline with target location
    if (current.direction.isFacingX) {
        float distance = (abs(current.x - targetX)) * squareDistance * 1.0;
        driveDistance(distance, frontSonar);
        // Update current location
        current.x = targetX;
    } else {
        float distance = (abs(current.y - targetY)) * squareDistance * 1.0;
        driveDistance(distance, frontSonar);
        // Update current location
        current.y = targetY;
    }

    Serial.println("Now facing correct way");
    delay(2000);

    motors.updateSpeeds(0, 0);
}

// Uses the side sonars to update the map
// Assuming that the robot is on GROUND
void mapCurrentLocation() {
    // Get distances from sonars
    float rightDistance = rightSonar.getAverageDistance(5);
    float leftDistance = leftSonar.getAverageDistance(5);

    // Find number of squares before something has been detected
    // This is assuming the given distance from sonar reaches the end
    // of the last square
    int rightSquares = floor(rightDistance / squareDistance);
    int leftSquares = floor(leftDistance / squareDistance);

    if (current.direction.isFacingX) {
        // Facing x dir

        // Set pos and neg directions based on robot direction
        int positiveSquares, negativeSquares;
        if (current.direction.isForward) {
            positiveSquares = leftSquares;
            negativeSquares = rightSquares;
        } else {
            positiveSquares = rightSquares;
            negativeSquares = leftSquares;
        }

        // Loop over all squares on each side setting them to PROPABLY_GROUND
        // Since sonar passes over all the squares we know that there is a large
        // possibly that they are GROUND, but this could be false, eg on the side there
        // are 2 or more water squares, so we would be incorrectly setting water squars to GROUND
        // After loop sent the last square to ITEM == something here
        // The last square is not guaranteed to be an objective, eg could be water square
        mapManager.setMapValue(current.x, current.y + positiveSquares, mapManager.ITEM);
        mapManager.setMapValue(current.x, current.y - negativeSquares, mapManager.ITEM);
    } else {
        // Facing y dir

        // Set pos and neg directions based on robot direction
        int positiveSquares, negativeSquares;
        if (current.direction.isForward) {
            positiveSquares = rightSquares;
            negativeSquares = leftSquares;
        } else {
            positiveSquares = leftSquares;
            negativeSquares = rightSquares;
        }

        // Loop over all squares on each side setting them to PROPABLY_GROUND
        // Since sonar passes over all the squares we know that there is a large
        // possibly that they are GROUND, but this could be false, eg on the side there
        // are 2 or more water squares, so we would be incorrectly setting water squars to GROUND
        // After loop sent the last square to ITEM == something here
        // The last square is not guaranteed to be an objective, eg could be water square
        mapManager.setMapValue(current.x + positiveSquares, current.y, mapManager.ITEM);
        mapManager.setMapValue(current.x - negativeSquares, current.y, mapManager.ITEM);
    }
}

void driveDistance(float distance, SonarLib sonar)
{
    if (distance < 1) {
        return;
    }
    Serial.print("Driving: ");
    Serial.println(distance);
    delay(2000);
    float startingDis = sonar.getAverageDistance(5);
    float currAngle = 0.0;
    float startingAngle;
    while (!getAngle(startingAngle))
    {
    }

    motors.driveForward();

     while (startingDis - sonar.getDistance() < distance)
    {
        Serial.println(sonar.getDistance());
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
    }
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

void dmpDataReady()
{
    mpuInterrupt = true;
}
