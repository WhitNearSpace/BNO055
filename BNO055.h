/* Embedded Systems Final Project: 9-Axis Absolute Orientation imu (BNO055) ndof.h
*/ 

#ifndef BNO055_H
#define BNO055_H

#include "mbed.h"

#define BNO055_MODE_NDOF   0 //used to call the 9 axis orientation of imu

#define BNO055_ANGLE_UNITS_DEGREE  0  
#define BNO055_ANGLE_UNITS_RADIAN  1


class BNO055 {//class for imu
    public: 
        BNO055(I2C i2c, int addr);//constructor
        float readHeading(void);//class methods
        float readRoll(void);
        float readPitch(void);
        char checkID(void); //Check for proper communication with sensor
        void setMode(int modeCode); //Set program to NDOF forge mode
        int setAngleUnits(int unitsCode); //Select Euler Angles as units
    private:
        I2C _i2c;
        int _addr; 
        int _units;
};

#endif
