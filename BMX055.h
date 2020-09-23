/*
 * BMX055.h
 *
 *  Created on: 23 Sep 2020
 *      Author: casperbroekhuizen
 */

#ifndef BMX055_H_
#define BMX055_H_

#include "DWire.h"
#include "Console.h"

#define BMX055_ACC_IDNUMBER 0xFA
#define BMX055_GYRO_IDNUMBER 0x0F
#define BMX055_MAG_IDNUMBER 0x32



#define BMX055_ACC_ID_REGISTER 0x00

#define BMX055_GYRO_ID_REGISTER 0x00

#define BMX055_MAG_ID_REGISTER 0x40



class BMX055
{
protected:
    DWire &wire;
    unsigned char accAddress;
    unsigned char gyroAddress;
    unsigned char magAddress;

public:
    BMX055(DWire &i2c, unsigned char acc_address, unsigned char gyro_address, unsigned char mag_address);
    virtual ~BMX055( ) {};

    void init();
    void test();

//    // configure the accelerometer
//    unsigned char setAccRange(uint8_t);
//    unsigned char setAccBandwidth(uint8_t);
//    unsigned char setAccLPM(uint8_t);
//
//    // configure the gyro
//    unsigned char setGyroRange(uint8_t);
//    unsigned char setGyroBandwidth(uint8_t);
//    unsigned char setGyroLPM(uint8_t);
//
//    // configure the magnetometer
//    unsigned char setMagRepetitions(uint8_t);

    // functions used to retrieve the measurements from the device
    unsigned char getAccId();
    unsigned char pingAcc();
    short getAccX();
    short getAccY();
    short getAccZ();

    unsigned char getGyroId();
    unsigned char pingGyro();
    short getGyroX();
    short getGyroY();
    short getGyroZ();

    unsigned char getMagId();
    unsigned char pingMag();
    short getMagX();
    short getMagY();
    short getMagZ();

private:
    // only for use
    unsigned char readShort(unsigned char, unsigned char, unsigned short &);
    unsigned char writeShort(unsigned char, unsigned char, unsigned short);

};


#endif /* BMX055_H_ */
