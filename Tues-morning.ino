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

//char newMap[6][6] = {
//             {'C', 'C', 'C', 'C', 'C', 'C'},
//             {'C', 'C', 'C', 'C', 'C', 'C'},
//             {'C', 'C', 'C', 'C', 'C', 'C'},
//             {'C','C','C','C','C','C'},
//             {'C', 'C', 'C', 'C', 'C', 'C'},
//             {'C', 'C', 'C', 'C', 'C', 'C'}
//         };
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

#define ENA 10
#define ENB 9
#define Right1 5
#define Right2 4
#define Left1 6
#define Left2 7

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

void dmpDataReady()
{
    mpuInterrupt = true;
}

void driveDistance(float distance, SonarLib sonar)
{
    float startingDis = sonar.getAverageDistance(5);
    float currAngle = 0.0;
    float startingAngle;
    while (!getAngle(startingAngle))
    {
    }

    motors.driveForward();
    Serial.print("Starting Distance: ");
    Serial.println(startingDis);
    float currentDistance = sonar.getAverageDistance(5);
    while (currentDistance > startingDis) {
      startingDis = sonar.getAverageDistance(5);
      currentDistance = sonar.getAverageDistance(5);
    }
    while (startingDis - currentDistance < distance)
    {
        Serial.println(currentDistance);
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
        currentDistance = sonar.getAverageDistance(5);
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

float getDistanceFromFlame(int heatValue)
{
    // Returns distance in cm
    return 0.25 * pow(M_E, 0.0014 * heatValue) * 0.01;
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
    delay(1000);
}

int foundCount = 0;
float firstAngle = 0;
float secondAngle = 0;

void loop()
{ // if programming accelerometer failed, don't try to do anything
    if (!dmpReady)
    {
        Serial.println("Accelerometer Not Working!!!!");
        return;
    }

    if (done)
        return;

    float startingAngle;
    while (!getAngle(startingAngle))
    {
    }

    motors.turnLeft();
    int heatValue;
    while (!done)
    {
        long sum = 0;
        for (int i = 0; i < numSamples; i++)
        {
            sum += analogRead(flameSensorPin);
        }
        heatValue = sum / numSamples;
        Serial.print(prevHeatValue);
        Serial.print("  ");
        Serial.println(heatValue);
        if (heatValue - prevHeatValue > 5)
        {
            if (!switchedDir)
            {
                motors.turnRight();
                switchedDir = true;
            }
            else
            {
                float angle;
                while (!getAngle(angle))
                {
                }
                minHeatAngle = angle;
                analogWrite(ENA, 0);
                analogWrite(ENB, 0);
                done = true;
            }
        }
        prevHeatValue = heatValue;
    }
    
    foundCount++;

    Serial.println("Found candle");
//    float distance = getDistanceFromFlame(heatValue);
//    int x = distance * sin(minHeatAngle * M_PI / 180);
//    int y = distance * cos(minHeatAngle * M_PI / 180);
//    Serial.print("X: ");
//    Serial.print(x);
//    Serial.print(" Y: ");
//    Serial.println(y);
//    analogWrite(ENA, 0);
//    analogWrite(ENB, 0);
//    delay(2000);
    prevHeatValue = 10000;

    float offsetAngle = startingAngle - minHeatAngle;
    if (offsetAngle > 180)
    {
        offsetAngle -= 360;
    }
    else if (offsetAngle < -180)
    {
        offsetAngle += 360;
    }

    if (offsetAngle >= -45 && offsetAngle < 0) {
      if (foundCount == 1) {
        firstAngle = offsetAngle;
        turnController(offsetAngle);
      }
      else if (foundCount == 2) {
        secondAngle = 180 + offsetAngle;
      }
    }
    else if (offsetAngle >= 0 && offsetAngle < 45) {
      if (foundCount == 1) {
        firstAngle = offsetAngle;
        turnController(offsetAngle);
      }
      else if (foundCount == 2) {
        secondAngle = 180 - offsetAngle;
      }
    }
    else if (offsetAngle >= 45 && offsetAngle < 135) {
      if (foundCount == 1) {
        firstAngle = offsetAngle - 90;
        turnController(offsetAngle + 90.0);
      }
      else if (foundCount == 2) {
        secondAngle = 180 - offsetAngle;
      }
    }
    else if (offsetAngle >= 135 && offsetAngle <= 180) {
      if (foundCount == 1) {
        firstAngle = offsetAngle - 180;
        turnController(offsetAngle + 180);
      }
      else if (foundCount == 2) {
        secondAngle = 180 - offsetAngle;
      }
    }
    else if (offsetAngle >= -180 && offsetAngle < -135) {
      if (foundCount == 1) {
        firstAngle = offsetAngle + 180;
        turnController(offsetAngle - 180);
      }
      else if (foundCount == 2) {
        secondAngle = offsetAngle + 180;
      }
    }
    else if (offsetAngle >= -135 && offsetAngle < -45) {
      if (foundCount == 1) {
        firstAngle = offsetAngle + 90;
        turnController(offsetAngle - 90);
      }
      else if (foundCount == 2) {
        secondAngle = offsetAngle + 180;
      }
    }

    if (foundCount == 1) {
      driveDistance(30, frontSonar);
      delay(2000);
    }
    done = false;

    if (foundCount == 2) {
      float thirdAngle  = 180 - firstAngle - secondAngle;
      Serial.print("First Angle: ");
      Serial.println(firstAngle);
      Serial.print("Third Angle: ");
      Serial.println(thirdAngle);
      float distance = 30 * sin(firstAngle * M_PI / 180) / sin(thirdAngle * M_PI / 180);
      float x = distance * sin(offsetAngle * M_PI / 180) + 10;
      float y = distance * cos(offsetAngle * M_PI / 180) + 10;
      Serial.print("X: ");
      Serial.print(x);
      Serial.print(" Y: ");
      Serial.println(y);
      Serial.print("Offset Angle: ");
      Serial.println(offsetAngle);
      Serial.print("Distance: ");
      Serial.println(distance);
      int angleToTurn;
      if (offsetAngle > 0) {
          angleToTurn = -90;
      }
      else {
          angleToTurn = 90;
      }
  
      turnController(offsetAngle);
      analogWrite(ENA, 0);
      analogWrite(ENB, 0);
      Serial.println("Turned back");
      delay(3000);
  
      int ySquares = floor((y) / 30);
      driveDistance(ySquares * 30, frontSonar);
      Serial.print("Drove y: ");
      Serial.println(ySquares);
      delay(3000);
  
      turnController(angleToTurn);
      Serial.println("Turned -90s");
      delay(4000);
  
      int xSquares = floor((x) / 30);
      driveDistance(xSquares * 30, frontSonar);
      Serial.print("Drove x: ");
      Serial.println(xSquares);
      delay(4000);
  
      while (true)
      {
      }
    }
    

//
//    // If offsetAngle is > 0 then candle is to the left of the robot
//    // If offsetAngle is < 0 then candle is to the right of the robot
//    int angleToTurn;
//    if (offsetAngle > 0) {
//        angleToTurn = -90;
//    }
//    else {
//        angleToTurn = 90;
//    }
//
////  turnController(offsetAngle);
//    analogWrite(ENA, 0);
//    analogWrite(ENB, 0);
//    Serial.println("Turned back");
//    delay(3000);
//
//    int ySquares = floor(y / 30);
//    driveDistance(ySquares * 30, frontSonar);
//    Serial.print("Drove y: ");
//    Serial.println(ySquares);
//    delay(3000);
//
//    turnController(angleToTurn);
//    Serial.println("Turned -90s");
//    delay(4000);
//
//    int xSquares = floor(x - 15 / 30);
//    driveDistance(xSquares * 30, frontSonar);
//    Serial.print("Drove x: ");
//    Serial.println(xSquares);
//    delay(4000);
//
//    while (true)
//    {
//    }
}

// Drives robot to given location
// Perhaps this function should go into the motor lib
//void driveToLocation(int targetX, int targetY) {
//    if (current.direction.isFacingX) {
//        bool isFacingWrongDirection = (current.direction.isForward) ? current.x > targetX : current.x < targetX;
//        if (isFacingWrongDirection && current.y == targetY) {
//            // y is correct but facing wrong dir
//            turnController(180);
//            current.direction.isForward = !current.direction.isForward;
//        } else if (isFacingWrongDirection) {
//            int directionCompensator = (current.direction.isForward) ? 1 : -1;
//            if (current.y > targetY) {
//                turnController(90.0 * directionCompensator);
//            } else if (current.y < targetY) {
//                turnController(-90.0 * directionCompensator);
//                current.direction.isForward = !current.direction.isForward;
//            }
//            current.direction.isFacingX = !current.direction.isFacingX;
//        }
//    } else {
//        bool isFacingWrongDirection = (current.direction.isForward) ? current.y > targetY : current.y < targetY;
//        if (isFacingWrongDirection && current.x == targetX) {
//            // y is correct but facing wrong dir
//            turnController(180);
//            current.direction.isForward = !current.direction.isForward;
//        } else if (isFacingWrongDirection) {
//            int directionCompensator = (current.direction.isForward) ? 1 : -1;
//            if (current.x > targetX) {
//                turnController(-90.0 * directionCompensator);
//                current.direction.isForward = !current.direction.isForward;
//            } else if (current.x < targetX) {
//                turnController(90.0 * directionCompensator);
//            }
//            current.direction.isFacingX = !current.direction.isFacingX;
//        }
//    }
//
//    Serial.print("isFacingX: ");
//    Serial.println(current.direction.isFacingX);
//    delay(2000);
//
//    // Drive until inline with target location
//    if (current.direction.isFacingX) {
//        float distance = (abs(current.x - targetX)) * squareDistance * 1.0;
//        driveDistance(distance, frontSonar);
//        // Update current location
//        current.x = targetX;
//    } else {
//        float distance = (abs(current.y - targetY)) * squareDistance * 1.0;
//        driveDistance(distance, frontSonar);
//        // Update current location
//        current.y = targetY;
//    }
//
//    Serial.println("Drove");
//    delay(2000);
//
//    int directionCompensator = (current.direction.isForward) ? 1 : -1;
//    if (current.direction.isFacingX) {
//        if (current.y > targetY) {
//            turnController(90.0 * directionCompensator);
//            current.direction.isFacingX = !current.direction.isFacingX;
//        } else if (current.y < targetY) {
//            turnController(-90.0 * directionCompensator);
//            current.direction.isFacingX = !current.direction.isFacingX;
//            current.direction.isForward = !current.direction.isForward;
//        }
//    } else {
//        if (current.x > targetX) {
//            turnController(-90.0 * directionCompensator);
//            current.direction.isFacingX = !current.direction.isFacingX;
//            current.direction.isForward = !current.direction.isForward;
//        } else if (current.x < targetX) {
//            turnController(90.0 * directionCompensator);
//            current.direction.isFacingX = !current.direction.isFacingX;
//        }
//    }
//
//    Serial.print("isFacingX: ");
//    Serial.println(current.direction.isFacingX);
//    delay(2000);
//
//    // Drive until inline with target location
//    if (current.direction.isFacingX) {
//        float distance = (abs(current.x - targetX)) * squareDistance * 1.0;
//        driveDistance(distance, frontSonar);
//        // Update current location
//        current.x = targetX;
//    } else {
//        float distance = (abs(current.y - targetY)) * squareDistance * 1.0;
//        driveDistance(distance, frontSonar);
//        // Update current location
//        current.y = targetY;
//    }
//
//    Serial.println("Now facing correct way");
//    delay(2000);
//
//    motors.updateSpeeds(0, 0);
//}
