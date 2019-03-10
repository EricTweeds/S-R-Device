#include "Arduino.h"
#include "AccelerometerLib.h"

// // I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// // for both classes must be in the include path of your project
// #include "I2Cdev.h"

// #include "MPU6050_6Axis_MotionApps20.h"
// //#include "MPU6050.h" // not necessary if using MotionApps include file

// // Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// // is used in I2Cdev.h
// #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
//     #include "Wire.h"
// #endif

// Function used by interrupt
static void AccelerometerLib::dmpDataReady() {
    AccelerometerLib::mpuInterrupt = true;
}

AccelerometerLib::AccelerometerLib(int interruptPin) {
    AccelerometerLib::mpuInterrupt = false;
    dmpReady = false;

    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    mpu.initialize();
    pinMode(interruptPin, INPUT);

    if (!mpu.testConnection()) {
        // ERROR!
    }

    // load and configure the DMP
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        attachInterrupt(digitalPinToInterrupt(interruptPin), AccelerometerLib::dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
    }
}

bool AccelerometerLib::getYaw(float &yaw) {
    // if programming failed, don't try to do anything
    if (!dmpReady) return false;

    // wait for MPU interrupt or extra packet(s) available
    while (!AccelerometerLib::mpuInterrupt && fifoCount < packetSize) {
        if (AccelerometerLib::mpuInterrupt && fifoCount < packetSize) {
          // try to get out of the infinite loop 
          fifoCount = mpu.getFIFOCount();
        }  
        // other program behavior stuff here
        // .
        // .
        // .
        // if you are really paranoid you can frequently test in between other
        // stuff to see if mpuInterrupt is true, and if so, "break;" from the
        // while() loop to immediately process the MPU data
        // .
        // .
        // .
    }

    // reset interrupt flag and get INT_STATUS byte
    AccelerometerLib::mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & _BV(MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount >= 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        fifoCount = mpu.getFIFOCount();

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & _BV(MPU6050_INTERRUPT_DMP_INT_BIT)) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

        Quaternion q;           // [w, x, y, z]         quaternion container
        float euler[3];         // [psi, theta, phi]    Euler angle container
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetEuler(euler, &q);
        // Serial.print("euler\t");
        // Serial.print(euler[0] * 180/M_PI);
        // Serial.print("\t");
        // Serial.print(euler[1] * 180/M_PI);
        // Serial.print("\t");
        // Serial.println(euler[2] * 180/M_PI);

        yaw = euler[0] * 180/M_PI;
        return true;
    }

    return false;
}

bool AccelerometerLib::getAcceleration(VectorInt16 &aaReal) {
    // if programming failed, don't try to do anything
    if (!dmpReady) return false;

    // wait for MPU interrupt or extra packet(s) available
    while (!AccelerometerLib::mpuInterrupt && fifoCount < packetSize) {
        if (AccelerometerLib::mpuInterrupt && fifoCount < packetSize) {
          // try to get out of the infinite loop 
          fifoCount = mpu.getFIFOCount();
        }  
        // other program behavior stuff here
        // .
        // .
        // .
        // if you are really paranoid you can frequently test in between other
        // stuff to see if mpuInterrupt is true, and if so, "break;" from the
        // while() loop to immediately process the MPU data
        // .
        // .
        // .
    }

    // reset interrupt flag and get INT_STATUS byte
    AccelerometerLib::mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & _BV(MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount >= 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        fifoCount = mpu.getFIFOCount();

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & _BV(MPU6050_INTERRUPT_DMP_INT_BIT)) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

        Quaternion q;           // [w, x, y, z]         quaternion container
        VectorInt16 aa;         // [x, y, z]            accel sensor measurements
        VectorFloat gravity;    // [x, y, z]            gravity vector
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetAccel(&aa, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
        // Serial.print("areal\t");
        // Serial.print(aaReal.x);
        // Serial.print("\t");
        // Serial.print(aaReal.y);
        // Serial.print("\t");
        // Serial.println(aaReal.z);

        return true;
    }

    return false;
}

bool AccelerometerLib::getSideAcceleration(float &accel) {
    VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
    if (getAcceleration(aaReal)) {
        accel = aaReal.x;
        return true;
    }
    return false;
}

bool AccelerometerLib::getForwardAcceleration(float &accel) {
    VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
    if (getAcceleration(aaReal)) {
        accel = aaReal.y;
        return true;
    }
    return false;
}
