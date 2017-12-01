//ported from arduino library: https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050
//written by szymon gaertig (email: szymon@gaertig.com.pl)
//
//Changelog: 
//2013-01-08 - first beta release

// I2Cdev library collection - MPU6050 I2C device class
// Based on InvenSense MPU-6050 register map document rev. 2.0, 5/19/2011 (RM-MPU-6000A-00)
// 10/3/2011 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
//
// Changelog:
//     ... - ongoing debug release

// NOTE: THIS IS ONLY A PARIAL RELEASE. THIS DEVICE CLASS IS CURRENTLY UNDERGOING ACTIVE
// DEVELOPMENT AND IS STILL MISSING SOME IMPORTANT FEATURES. PLEASE KEEP THIS IN MIND IF
// YOU DECIDE TO USE THIS PARTICULAR CODE FOR ANYTHING.

/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2012 Jeff Rowberg

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

#ifndef _MPU6050_H_
#define _MPU6050_H_

#include "I2Cdev.h"
#include "mbed.h"

#define MPU6050_ADDRESS_AD0_LOW     0x68 // address pin low (GND), default for InvenSense evaluation board
#define MPU6050_ADDRESS_AD0_HIGH    0x69 // address pin high (VCC)
#define MPU6050_DEFAULT_ADDRESS     MPU6050_ADDRESS_AD0_LOW

#define MPU6050_CLOCK_PLL_XGYRO         0x01
#define MPU6050_GYRO_FS_250         0x00
#define MPU6050_ACCEL_FS_2          0x00
#define MPU6050_RA_PWR_MGMT_1       0x6B
#define MPU6050_PWR1_CLKSEL_BIT         2
#define MPU6050_PWR1_CLKSEL_LENGTH      3
#define MPU6050_RA_GYRO_CONFIG      0x1B
#define MPU6050_GCONFIG_FS_SEL_BIT      4
#define MPU6050_GCONFIG_FS_SEL_LENGTH   2
#define MPU6050_RA_ACCEL_CONFIG     0x1C
#define MPU6050_ACONFIG_AFS_SEL_BIT         4
#define MPU6050_ACONFIG_AFS_SEL_LENGTH      2
#define MPU6050_PWR1_SLEEP_BIT          6
#define MPU6050_RA_WHO_AM_I         0x75
#define MPU6050_WHO_AM_I_BIT        7
#define MPU6050_WHO_AM_I_LENGTH     8
#define MPU6050_TC_OFFSET_BIT       6
#define MPU6050_TC_OFFSET_LENGTH    6

#define MPU6050_RA_XG_OFFS_TC       0x00 //[7] PWR_MODE, [6:1] XG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU6050_RA_YG_OFFS_TC       0x01 //[7] PWR_MODE, [6:1] YG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU6050_RA_ZG_OFFS_TC       0x02 //[7] PWR_MODE, [6:1] ZG_OFFS_TC, [0] OTP_BNK_VLD

#define MPU6050_RA_XA_OFFS_H        0x06 //[15:0] XA_OFFS
#define MPU6050_RA_YA_OFFS_H        0x08 //[15:0] YA_OFFS
#define MPU6050_RA_ZA_OFFS_H        0x0A //[15:0] ZA_OFFS

#define MPU6050_RA_ACCEL_XOUT_H     0x3B
#define MPU6050_RA_USER_CTRL        0x6A
#define MPU6050_USERCTRL_FIFO_RESET_BIT         2

class MPU6050 {
private:
    I2Cdev i2Cdev;
    Serial debugSerial;
    Timer timer;
public:
    MPU6050();
    MPU6050(uint8_t address);

    void initialize();
    bool testConnection();

    uint8_t getFullScaleGyroRange();
    void setFullScaleGyroRange(uint8_t range);

    uint8_t getFullScaleAccelRange();
    void setFullScaleAccelRange(uint8_t range);

    void getMotion6(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz);

    void resetFIFO();

    void setSleepEnabled(bool enabled);
    void setClockSource(uint8_t source);

    // WHO_AM_I register
    uint8_t getDeviceID();

    int8_t getXGyroOffset();
    void setXGyroOffset(int8_t offset);

    // YG_OFFS_TC register
    int8_t getYGyroOffset();
    void setYGyroOffset(int8_t offset);

    // ZG_OFFS_TC register
    int8_t getZGyroOffset();
    void setZGyroOffset(int8_t offset);

    // XA_OFFS_* registers
    int16_t getXAccelOffset();
    void setXAccelOffset(int16_t offset);

    // YA_OFFS_* register
    int16_t getYAccelOffset();
    void setYAccelOffset(int16_t offset);

    // ZA_OFFS_* register
    int16_t getZAccelOffset();
    void setZAccelOffset(int16_t offset);

    // XG_OFFS_USR* registers
    int16_t getXGyroOffsetUser();
    void setXGyroOffsetUser(int16_t offset);

    // YG_OFFS_USR* register
    int16_t getYGyroOffsetUser();
    void setYGyroOffsetUser(int16_t offset);

    // ZG_OFFS_USR* register
    int16_t getZGyroOffsetUser();
    void setZGyroOffsetUser(int16_t offset);

private:
    uint8_t devAddr;
    uint8_t buffer[14];
    
};

#endif /* _MPU6050_H_ */