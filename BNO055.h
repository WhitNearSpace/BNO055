/* Embedded Systems Final Project: 9-Axis Absolute Orientation imu (BNO055) ndof.h
*/ 

#ifndef BNO055_H
#define BNO055_H

#include "mbed.h"

#define BNO055_MODE_NDOF   0 //used to call the 9 axis orientation of imu

#define BNO055_ANGLE_UNITS_DEGREE  0  
#define BNO055_ANGLE_UNITS_RADIAN  1
#define BNO055_ACCEL_UNITS_SI 0
#define BNO055_ACCEL_UNITS_MG 1


class BNO055 {//class for imu
    public: 
        BNO055(I2C* i2c, int addr);
        float readHeading(void);
        float readRoll(void);
        float readPitch(void);
        char checkID(void); // Return 0xA0 if found BNO055 sensor
        void setMode(int modeCode); // Set operating mode
        int setAngleUnits(int unitsCode); // Select rad or deg as units
        int setAccelerationUnits(int unitsCode); // select m/s^2 (SI) or mg
        int getGyroData(float omegas[3]);
        int getEulerAngles(float eulers[3]);
        int getAcceleration(float a[3]);
    private:
        I2C* _i2c;
        int _addr; 
        int _angle_units;
        int _accel_units;
};

#endif
