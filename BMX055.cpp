/*
 * BMX055.cpp
 *
 *  Created on: 23 Sep 2020
 *      Author: casperbroekhuizen
 */

#include "BMX055.h"

BMX055::BMX055(DWire &i2c, unsigned char acc_address, unsigned char gyro_address, unsigned char mag_address):
wire(i2c), accAddress(acc_address), gyroAddress(gyro_address), magAddress(mag_address)
{
};

void BMX055::init(){

      // RESTART MAGNETOMETER
      wire.beginTransmission(magAddress);
      wire.write(0x4B);
      // Soft reset
      wire.write(0x83);
      wire.endTransmission();

      // Normal Mode
      wire.beginTransmission(magAddress);
      wire.write(0x4C);
      wire.write(0x00);
      wire.endTransmission();

      // X, Y, Z-Axis enabled
      wire.beginTransmission(magAddress);
      wire.write(0x4E);
      wire.write(0x00);
      wire.endTransmission();
}

void BMX055::test()
{

  // Output data to serial monitor
  short xAccl = getAccX();
  short yAccl = getAccY();
  short zAccl = getAccZ();

  short xGyro = getGyroX();
  short yGyro = getGyroY();
  short zGyro = getGyroZ();

  short xMag = getMagX();
  short yMag = getMagY();
  short zMag = getMagZ();


  Console::log("Acceleration in X-Axis : %s%d", xAccl < 0 ? "-":"", xAccl < 0 ? -xAccl:xAccl);
  Console::log("Acceleration in Y-Axis : %s%d", yAccl < 0 ? "-":"", yAccl < 0 ? -yAccl:yAccl);
  Console::log("Acceleration in Z-Axis : %s%d", zAccl < 0 ? "-":"", zAccl < 0 ? -zAccl:zAccl);
  Console::log("X-Axis of rotation : %s%d", xGyro < 0 ? "-":"", xGyro < 0 ? -xGyro:xGyro);
  Console::log("Y-Axis of rotation : %s%d", yGyro < 0 ? "-":"", yGyro < 0 ? -yGyro:yGyro);
  Console::log("Z-Axis of rotation : %s%d", zGyro < 0 ? "-":"", zGyro < 0 ? -zGyro:zGyro);
  Console::log("Magnetic field in X-Axis : %s%d", xMag < 0 ? "-":"", xMag < 0 ? -xMag:xMag);
  Console::log("Magnetic field in Y-Axis : %s%d", yMag < 0 ? "-":"", yMag < 0 ? -yMag:yMag);
  Console::log("Magnetic filed in Z-Axis : %s%d", zMag < 0 ? "-":"", zMag < 0 ? -zMag:zMag);
}

short BMX055::getAccX(){
    unsigned short tmp = 0;
    this->readShort(accAddress, 0x02, tmp);
    short xAccl = (((tmp << 8) & 0xFF00) | ((tmp >> 8) & 0x00FF)) >> 4;
    if (xAccl & 0x01 << 11)
    {
    xAccl |= 0xF800;
    }
    return xAccl;
}

short BMX055::getAccY(){
    unsigned short tmp = 0;
    this->readShort(accAddress, 0x04, tmp);
    short yAccl = (((tmp << 8) & 0xFF00) | ((tmp >> 8) & 0x00FF)) >> 4;
    if (yAccl & 0x01 << 11)
    {
    yAccl |= 0xF800;
    }
    return yAccl;
}

short BMX055::getAccZ(){
    unsigned short tmp = 0;
    this->readShort(accAddress, 0x06, tmp);
    short zAccl = (((tmp << 8) & 0xFF00) | ((tmp >> 8) & 0x00FF)) >> 4;
    if (zAccl & 0x01 << 11)
    {
    zAccl |= 0xF800;
    }
    return zAccl;
}

unsigned char BMX055::getAccId(){
    wire.beginTransmission(accAddress);
    wire.write(BMX055_ACC_ID_REGISTER);
    wire.requestFrom(accAddress,1);
    uint8_t idNumber = wire.read();
    wire.endTransmission();
    return idNumber;
}

unsigned char BMX055::pingAcc(){
    return  getAccId() == BMX055_ACC_IDNUMBER;
}

short BMX055::getGyroX(){
    unsigned short tmp = 0;
    this->readShort(gyroAddress, 0x02, tmp);
    short xGyro = (((tmp << 8) & 0xFF00) | ((tmp >> 8) & 0x00FF));
    return xGyro;
}

short BMX055::getGyroY(){
    unsigned short tmp = 0;
    this->readShort(gyroAddress, 0x04, tmp);
    short yGyro = (((tmp << 8) & 0xFF00) | ((tmp >> 8) & 0x00FF));
    return yGyro;
}

short BMX055::getGyroZ(){
    unsigned short tmp = 0;
    this->readShort(gyroAddress, 0x06, tmp);
    short zGyro = (((tmp << 8) & 0xFF00) | ((tmp >> 8) & 0x00FF));
    return zGyro;
}

unsigned char BMX055::getGyroId(){
    wire.beginTransmission(gyroAddress);
    wire.write(BMX055_GYRO_ID_REGISTER);
    wire.requestFrom(gyroAddress,1);
    uint8_t idNumber = wire.read();
    wire.endTransmission();
    return idNumber;
}

unsigned char BMX055::pingGyro(){
    return  getGyroId() == BMX055_GYRO_IDNUMBER;
}

short BMX055::getMagX(){
    unsigned short tmp = 0;
    this->readShort(magAddress, 0x42, tmp);
    short xMag = (((tmp << 8) & 0xFF00) | ((tmp >> 8) & 0x00FF)) >> 3;
    if (xMag & 0x01 << 12)
    {
        xMag |= 0xF000;
    }
    return xMag;
}

short BMX055::getMagY(){
    unsigned short tmp = 0;
    this->readShort(magAddress, 0x44, tmp);
    short yMag = (((tmp << 8) & 0xFF00) | ((tmp >> 8) & 0x00FF)) >> 3;
    if (yMag & 0x01 << 12)
    {
        yMag |= 0xF000;
    }
    return yMag;
}

short BMX055::getMagZ(){
    unsigned short tmp = 0;
    this->readShort(magAddress, 0x46, tmp);
    short zMag = (((tmp << 8) & 0xFF00) | ((tmp >> 8) & 0x00FF)) >> 1;
    if (zMag & 0x01 << 14)
    {
        zMag |= 0x8000;
    }
    return zMag;
}

unsigned char BMX055::getMagId(){
    wire.beginTransmission(0x10);
    // Select Mag register
    wire.write(0x4B);
    // Soft reset
    wire.write(0x83);
    // Stop I2C Transmission
    wire.endTransmission();


    wire.beginTransmission(0x10);
    wire.write(0x40);
    wire.requestFrom(0x10,1);
    uint8_t idNumber = wire.read();
    wire.endTransmission();
    return idNumber;
}

unsigned char BMX055::pingMag(){
    return  getMagId() == BMX055_MAG_IDNUMBER;
}

unsigned char BMX055::readShort(unsigned char address, unsigned char reg, unsigned short &output)
{
    wire.beginTransmission(address);
    wire.write(reg);

    unsigned char res = wire.requestFrom(address, 2);
    if (res == 2)
    {
        output = ((unsigned short)wire.read()) << 8;
        output |= wire.read() & 0xFF;
        return 0;
    }
    else
    {
        return 1;
    }
}

/**
 *
 *   Sets the value of the selected internal register
 *
 *   Parameters:
 *   unsigned char reg     register number
 *   unsigned short        register value
 *
 *   Returns:
 *   unsigned char         0 success
 *                         1 fail
 *
 */
unsigned char BMX055::writeShort(unsigned char address, unsigned char reg, unsigned short val)
{
    wire.beginTransmission(address);
    wire.write(reg);
    wire.write((val >> 8) & 0xFF);
    wire.write(val & 0xFF);
    return wire.endTransmission();
}
