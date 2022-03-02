/* Embedded Systems Final Project: 9-Axis Absolute Orientation imu (BNO055) ndof.cpp 
*/

#include "BNO055.h"
#include "mbed.h"

#define REG_ADDR_OPR_MODE   0x3D //addresses for registers which are needed for this code
#define REG_ADDR_UNIT_SEL   0x3B //address of unit selection

// Constructor using shared I2C object
BNO055::BNO055(I2C* i2c, int addr) {
    _i2c = i2c;
    _addr = addr;
}

// Get ID from BNO055. Useful for verifying communication with device.
char BNO055::checkID() {
    char buff[1];
    buff[0] = 0x00;
    _i2c->write(_addr, buff, 1);
    _i2c->read(_addr, buff, 1);
    return buff[0];
}

void BNO055::setMode(int modeCode) {
    char buff[2];
    char code;

    buff[0] = REG_ADDR_OPR_MODE;
    _i2c->write(_addr, buff, 1);
    _i2c->read(_addr, buff, 1);
    code = buff[0];

    if(modeCode == BNO055_MODE_NDOF) { // Set NDOF fusion mode
        code = code & 0xF0;
        code = code | 0x0C;
    }
    buff[0] = REG_ADDR_OPR_MODE;
    buff[1] = code;
    _i2c->write(_addr, buff, 2);
    ThisThread::sleep_for(7ms);
}

// will set whether it displays information in degrees or radians
int BNO055::setAngleUnits(int unitsCode) {
    _units = unitsCode;
    char buff[3];
    char code;
    buff[0] = REG_ADDR_UNIT_SEL;
    _i2c->write(_addr, buff, 1);
    _i2c->read(_addr, buff, 1);
    code = buff[0];

    if (unitsCode == BNO055_ANGLE_UNITS_DEGREE) {
        code = code & (0xFF - 0x02 - 0x04);
    } else if (unitsCode == BNO055_ANGLE_UNITS_RADIAN) {
        code = code & (0xFF - 0x02 - 0x04);
        code = code | 0x04;
        code = code | 0x02;
    }
    buff[0] = REG_ADDR_UNIT_SEL;
    buff[1] = code;
    _i2c->write(_addr, buff, 2);
    return unitsCode;//returns int for main to be able to manipulate
}

float BNO055::readHeading() {
    char buff[2];
    int16_t rawHeading;
    buff[0] = 0x1A;
    _i2c->write(_addr, buff, 1, true);
    _i2c->read(_addr, buff, 2);
    rawHeading = (buff[1]<<8) | buff[0];
    float euler;
    switch (_units) {
        case BNO055_ANGLE_UNITS_DEGREE: euler = rawHeading/16.0; break;   //1 deg = 16LSB  from data sheet
        case BNO055_ANGLE_UNITS_RADIAN: euler = rawHeading/900.0; break;  //1 rad = 900LSB  
    }
    return euler;
}

float BNO055::readRoll() {
    char buff[2];
    int16_t rawRoll;
    buff[0] = 0x1C;
    _i2c->write(_addr, buff, 1, true);
    _i2c->read(_addr, buff , 2);
    rawRoll = (buff[1]<< 8) | buff[0];
    float euler;
    switch(_units) {
        case BNO055_ANGLE_UNITS_DEGREE: euler = rawRoll/16.0; break;     
        case BNO055_ANGLE_UNITS_RADIAN: euler = rawRoll/900.0; break;   
    }
    return euler;
}

float BNO055::readPitch() {
    char buff[2];
    int16_t rawPitch;
    buff[0] = 0x1E;
    _i2c->write(_addr, buff, 1, true);
    _i2c->read(_addr, buff, 2);
    rawPitch = (buff[1]<< 8) | buff[0];
    float euler;
    switch(_units) {
        case BNO055_ANGLE_UNITS_DEGREE: euler = -rawPitch/16.0; break;     
        case BNO055_ANGLE_UNITS_RADIAN: euler = -rawPitch/900.0; break;  
    }
    return euler;
}
