/* Embedded Systems Final Project: 9-Axis Absolute Orientation imu (BNO055) ndof.cpp 
*/

#include "BNO055.h"
#include "mbed.h"

#define REG_ADDR_OPR_MODE   0x3D //addresses for registers which are needed for this code
#define REG_ADDR_UNIT_SEL   0x3B //address of unit selection
#define REG_ADDR_ACC_X_LSB 0x08
#define REG_ADDR_EULER_HEADING_LSB 0x1A
#define REG_ADDR_EULER_ROLL_LSB 0x1C
#define REG_ADDR_EULER_PITCH_LSB 0x1E
#define REG_ADDR_GYRO_DATA_X_LSB 0x14
#define RAW_TO_DEG 16.0
#define RAW_TO_RAD 900.0
#define RAW_TO_SI_ACCEL 100.0
#define RAW_TO_MG_ACCEL 1.0


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

// set angle units to degrees or radians
// returns NAK
int BNO055::setAngleUnits(int unitsCode) {
    int nak;
    char buff[2];
    char code;
    buff[0] = REG_ADDR_UNIT_SEL;
    nak = _i2c->write(_addr, buff, 1);
    if (nak) return nak;
    nak = _i2c->read(_addr, buff, 1);
    if (nak) return nak;
    code = buff[0];

    if (unitsCode == BNO055_ANGLE_UNITS_DEGREE) {
        code = code & (0xFF - 0x02 - 0x04); // unset Euler and gyro bits
    } else if (unitsCode == BNO055_ANGLE_UNITS_RADIAN) {
        code = code | 0x04; // set Euler bit
        code = code | 0x02; // set gyro bit
    }
    buff[0] = REG_ADDR_UNIT_SEL;
    buff[1] = code;
    nak = _i2c->write(_addr, buff, 2);
    if (nak) return nak;
    _angle_units = unitsCode; // only update this if write was a success
    return nak;
}

// set linear acceleration units to m/s^2 or mg
// returns NAK
int BNO055::setAccelerationUnits(int unitsCode) {
    int nak;
    char buff[2];
    char code;
    buff[0] = REG_ADDR_UNIT_SEL;
    nak = _i2c->write(_addr, buff, 1);
    if (nak) return nak;
    nak = _i2c->read(_addr, buff, 1);
    if (nak) return nak;
    code = buff[0]; // current unit byte

    // change bit 0 of unit byte to set acceleration units
    if (unitsCode == BNO055_ACCEL_UNITS_SI) {
        code = code & (0xFF - 0x01); // unset accel bit (so m/s^2)
    } else if (unitsCode == BNO055_ACCEL_UNITS_MG) {
        code = code | 0x01; // set accel bit (so mg)
    }
    buff[0] = REG_ADDR_UNIT_SEL;
    buff[1] = code;
    nak = _i2c->write(_addr, buff, 2);
    if (nak) return nak;
    _accel_units = unitsCode; // only update if write was a success
    return nak;
}

// Pass three-element float array, filled with x,y,z accel data on return
// Returns NAK response
int BNO055::getAcceleration(float a[3]) {
    int nak;
    char buff[6];
    int16_t raw_a[3];
    buff[0] = REG_ADDR_ACC_X_LSB;
    nak = _i2c->write(_addr, buff, 1, true);
    if (nak) return nak;
    nak = _i2c->read(_addr, buff, 6);
    if (nak) return nak;
    for (int i = 0; i < 3; i++) {
        raw_a[i] = (buff[2*i+1]<<8) | buff[2*i];
        switch (_accel_units) {
            case BNO055_ACCEL_UNITS_SI: a[i] = raw_a[i]/RAW_TO_SI_ACCEL; break;
            case BNO055_ACCEL_UNITS_MG: a[i] = raw_a[i]/RAW_TO_MG_ACCEL; break;
        }
    }
    return nak;
}

// Pass three-element float array, filled with x,y,z gyro data on return
// Returns NAK response
int BNO055::getGyroData(float omegas[3]) {
    int nak;
    char buff[6];
    int16_t raw_omega[3];
    buff[0] = REG_ADDR_GYRO_DATA_X_LSB;
    nak = _i2c->write(_addr, buff, 1, true);
    if (nak) return nak;
    nak = _i2c->read(_addr, buff, 6);
    if (nak) return nak;
    for (int i = 0; i < 3; i++) {
        raw_omega[i] = (buff[2*i+1]<<8) | buff[2*i];
        switch (_angle_units) {
            case BNO055_ANGLE_UNITS_DEGREE: omegas[i] = raw_omega[i]/RAW_TO_DEG; break;
            case BNO055_ANGLE_UNITS_RADIAN: omegas[i] = raw_omega[i]/RAW_TO_RAD; break;
        }
    }
    return nak;
}

// Pass three-element float array, filled with heading,roll,pitch angles on return
// Returns NAK response
int BNO055::getEulerAngles(float eulers[3]) {
    int nak;
    char buff[6];
    int16_t raw_euler[3];
    buff[0] = REG_ADDR_EULER_HEADING_LSB;
    nak = _i2c->write(_addr, buff, 1, true);
    if (nak) return nak;
    nak = _i2c->read(_addr, buff, 6);
    if (nak) return nak;
    for (int i = 0; i < 3; i++) {
        raw_euler[i] = (buff[2*i+1]<<8) | buff[2*i];
        switch (_angle_units) {
            case BNO055_ANGLE_UNITS_DEGREE: eulers[i] = raw_euler[i]/RAW_TO_DEG; break;
            case BNO055_ANGLE_UNITS_RADIAN: eulers[i] = raw_euler[i]/RAW_TO_RAD; break;
        }
    }
    return nak;
}

float BNO055::readHeading() {
    char buff[2];
    int16_t rawHeading;
    buff[0] = REG_ADDR_EULER_HEADING_LSB;
    _i2c->write(_addr, buff, 1, true);
    _i2c->read(_addr, buff, 2);
    rawHeading = (buff[1]<<8) | buff[0];
    float euler;
    switch (_angle_units) {
        case BNO055_ANGLE_UNITS_DEGREE: euler = rawHeading/16.0; break;   //1 deg = 16LSB  from data sheet
        case BNO055_ANGLE_UNITS_RADIAN: euler = rawHeading/900.0; break;  //1 rad = 900LSB  
    }
    return euler;
}

float BNO055::readRoll() {
    char buff[2];
    int16_t rawRoll;
    buff[0] = REG_ADDR_EULER_ROLL_LSB;
    _i2c->write(_addr, buff, 1, true);
    _i2c->read(_addr, buff , 2);
    rawRoll = (buff[1]<< 8) | buff[0];
    float euler;
    switch(_angle_units) {
        case BNO055_ANGLE_UNITS_DEGREE: euler = rawRoll/16.0; break;     
        case BNO055_ANGLE_UNITS_RADIAN: euler = rawRoll/900.0; break;   
    }
    return euler;
}

float BNO055::readPitch() {
    char buff[2];
    int16_t rawPitch;
    buff[0] = REG_ADDR_EULER_PITCH_LSB;
    _i2c->write(_addr, buff, 1, true);
    _i2c->read(_addr, buff, 2);
    rawPitch = (buff[1]<< 8) | buff[0];
    float euler;
    switch(_angle_units) {
        case BNO055_ANGLE_UNITS_DEGREE: euler = -rawPitch/16.0; break;     
        case BNO055_ANGLE_UNITS_RADIAN: euler = -rawPitch/900.0; break;  
    }
    return euler;
}
