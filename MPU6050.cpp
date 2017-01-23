//ported from arduino library: https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050
//written by szymon gaertig (email: szymon@gaertig.com.pl)
//
//Changelog:
//2013-01-08 - first beta release

// I2Cdev library collection - MPU6050 I2C device class
// Based on InvenSense MPU-6050 register map document rev. 2.0, 5/19/2011 (RM-MPU-6000A-00)
// 8/24/2011 by Jeff Rowberg <jeff@rowberg.net>
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

#include "MPU6050.h"

#define useDebugSerial

//instead of using pgmspace.h
typedef const unsigned char prog_uchar;
#define pgm_read_byte_near(x) (*(prog_uchar*)x)
#define pgm_read_byte(x) (*(prog_uchar*)x)

/** Default constructor, uses default I2C address.
 * @see MPU6050_DEFAULT_ADDRESS
 */
MPU6050::MPU6050() : debugSerial(USBTX, USBRX)
{
    //i2Cdev.frequency(400000);
    first_update = 0;
    last_update = 0;
    cur_time = 0;
    GyroMeasError = PI * (40.0f / 180.0f);
    beta = sqrt(3.0f / 4.0f) * GyroMeasError;
    GyroMeasDrift = PI * (2.0f / 180.0f);
    zeta = sqrt(3.0f / 4.0f) * GyroMeasDrift;
    timer.start();
    q[0] = 1.0f;
    q[1] = 0.0f;
    q[2] = 0.0f;
    q[3] = 0.0f;
    devAddr = MPU6050_DEFAULT_ADDRESS;

    setClockSource(MPU6050_CLOCK_PLL_XGYRO);
    setFullScaleGyroRange(MPU6050_GYRO_FS_250);
    setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
    setSleepEnabled(false); // thanks to Jack Elston for pointing this one out!

}

/** Specific address constructor.
 * @param address I2C address
 * @see MPU6050_DEFAULT_ADDRESS
 * @see MPU6050_ADDRESS_AD0_LOW
 * @see MPU6050_ADDRESS_AD0_HIGH
 */
MPU6050::MPU6050(uint8_t address) : debugSerial(USBTX, USBRX)
{
    first_update = 0;
    last_update = 0;
    cur_time = 0;
    GyroMeasError = PI * (60.0f / 180.0f);
    beta = sqrt(3.0f / 4.0f) * GyroMeasError;
    GyroMeasDrift = PI * (1.0f / 180.0f);
    zeta = sqrt(3.0f / 4.0f) * GyroMeasDrift;
    q[0] = 1.0f;
    q[1] = 0.0f;
    q[2] = 0.0f;
    q[3] = 0.0f;
    timer.start();
    
    devAddr = address;

    setClockSource(MPU6050_CLOCK_PLL_XGYRO);
    setFullScaleGyroRange(MPU6050_GYRO_FS_250);
    setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
    setSleepEnabled(false); // thanks to Jack Elston for pointing this one out!

}

/** Power on and prepare for general usage.
 * This will activate the device and take it out of sleep mode (which must be done
 * after start-up). This function also sets both the accelerometer and the gyroscope
 * to their most sensitive settings, namely +/- 2g and +/- 250 degrees/sec, and sets
 * the clock source to use the X Gyro for reference, which is slightly better than
 * the default internal clock source.
 */
void MPU6050::initialize()
{

#ifdef useDebugSerial
    debugSerial.printf("MPU6050::initialize start\n\r");
#endif

    setClockSource(MPU6050_CLOCK_PLL_XGYRO);
    setFullScaleGyroRange(MPU6050_GYRO_FS_250);
    setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
    setSleepEnabled(false); // thanks to Jack Elston for pointing this one out!
}

/** Verify the I2C connection.
 * Make sure the device is connected and responds as expected.
 * @return True if connection is valid, false otherwise
 */
bool MPU6050::testConnection()
{
#ifdef useDebugSerial
    debugSerial.printf("MPU6050::testConnection start\n\r");
#endif
    uint8_t deviceId = getDeviceID();
#ifdef useDebugSerial
    debugSerial.printf("DeviceId = 0x%x\n\r",deviceId);
#endif
    return deviceId == MPU6050_ADDRESS_AD0_LOW;
}

/* Check if we need to calibrate the device
 * Check the factory trim values and see
 * if they are +/- 14 or less. The integer
 * that is returned is a bitmap of the sensors
 * Bit  5: Z-Gyro Fail
 *      4: Y-Gyro Fail
 *      3: X-Gyro Fail
 *      2: Z-Accel Fail  
 *      1: Y-Accel Fail
 *      0: X-Accel Fail
 * If any of the sensors fail the self test
 * then there will be a 1 at that bit
 * 
 * The destination argument is used to give back
 * the values of the self test.
 */
int MPU6050::selfTest(float* destination) {
    float accel_var[3], gyro_var[3];
    long gyro_st[3], accel_st[3], gyro[3], accel[3];
    unsigned char accel_result, gyro_result;

    uint8_t result;

    // Get data without self test
    get_st_biases(gyro, accel, 0);
    get_st_biases(gyro_st, accel_st, 1);

    // Get the results
    accel_result = accel_self_test(accel, accel_st, accel_var);
    gyro_result = gyro_self_test(gyro, gyro_st, gyro_var);

    result = gyro_result << 3 | accel_result;
    
    destination[5] = gyro_var[0]; // X-GYRO
    destination[4] = gyro_var[1]; // Y-GYRO
    destination[3] = gyro_var[2]; // Z-GYRO
    destination[2] = accel_var[0]; // X-ACCL
    destination[1] = accel_var[1]; // Y-ACCL
    destination[0] = accel_var[2]; // Z-ACCL
    
    return result;
}

/* This function gets gyro and accel data
 * If hw_test then it will get the self-test values
 */
void MPU6050::get_st_biases(long *gyro, long *accel, unsigned char hw_test)
{
    unsigned char data[12];
    unsigned char packet_count, i;
    unsigned short fifo_count;

    // Set the power management registers
    data[0] = 0x01; // Use x-gyro reference
    data[1] = 0x00;
    i2Cdev.writeByte(devAddr, MPU6050_RA_PWR_MGMT_1, data[0]);
    i2Cdev.writeByte(devAddr, MPU6050_RA_PWR_MGMT_2, data[1]);
    wait(0.2); // Let the PLL lock

    data[0] = 0x00;
    // Disable interrupts
    i2Cdev.writeByte(devAddr, MPU6050_RA_INT_ENABLE, data[0]);
    // Disable fifo
    i2Cdev.writeByte(devAddr, MPU6050_RA_FIFO_EN, data[0]);
    // -----Set clock to 8 MHz internal? why?
    i2Cdev.writeByte(devAddr, MPU6050_RA_PWR_MGMT_1, data[0]);
    // Disable i2c master
    i2Cdev.writeByte(devAddr, MPU6050_RA_I2C_MST_CTRL, data[0]);
    // Disable the stuff in user control
    i2Cdev.writeByte(devAddr, MPU6050_RA_USER_CTRL, data[0]);

    // reset fifo and dmp
    data[0] = MPU6050_USERCTRL_DMP_RESET_BIT | MPU6050_USERCTRL_FIFO_RESET_BIT;
    i2Cdev.writeByte(devAddr, MPU6050_RA_USER_CTRL, data[0]);
    wait(0.015);

    // Set lpf
    i2Cdev.writeByte(devAddr, MPU6050_RA_CONFIG, MPU6050_DLPF_BW_188);

    // Set rate_div
    i2Cdev.writeByte(devAddr, MPU6050_RA_SMPLRT_DIV, 0x00);

    // Configure the Accelerometer and Gyroscope registers
    if (hw_test) {
        // Configure the accelerometer and gyroscope for self-test
        i2Cdev.writeByte(devAddr, MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_SELF_TEST_BYTE);
        i2Cdev.writeByte(devAddr, MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_SELF_TEST_BYTE);
        wait(.2);
    }
    else {
        // Configure the accelerometer and gyroscope, not for self-test
        i2Cdev.writeByte(devAddr, MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_BYTE);
        i2Cdev.writeByte(devAddr, MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_BYTE);
    }

    // Fill FIFO for 50 ms, and get the count
    i2Cdev.writeBit(devAddr, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_FIFO_EN_BIT, 0x1);
    i2Cdev.writeByte(devAddr, MPU6050_RA_FIFO_EN, MPU6050_ENABLE_ALL_FIFO);
    wait(.05);
    i2Cdev.writeByte(devAddr, MPU6050_RA_FIFO_EN, MPU6050_DISABLE_ALL_FIFO);
    i2Cdev.readBytes(devAddr, MPU6050_RA_FIFO_COUNTH, 2, data);

    // Get the number of packets
    // A packet is 12 bytes: xAcc, yAcc, zAcc, xGyro, yGyro, zGyro
    // Each has two bytes for high and low
    fifo_count = (data[0] << 8) | data[1]; // Fix the data count high, low
    packet_count = fifo_count / 12; // Get the number of packets (12 bytes per packet)
    gyro[0] = gyro[1] = gyro[2] = 0;
    accel[0] = accel[1] = accel[2] = 0;

    for (i = 0; i < packet_count; i++) {
        short accel_cur[3], gyro_cur[3];

        // Get a packet of data
        i2Cdev.readBytes(devAddr, MPU6050_RA_FIFO_R_W, 12, data);

        accel_cur[0] = ((short) data[0] << 8) | data[1];
        accel_cur[1] = ((short) data[2] << 8) | data[3];
        accel_cur[2] = ((short) data[4] << 8) | data[5];

        accel[0] += (long) accel_cur[0];
        accel[1] += (long) accel_cur[1];
        accel[2] += (long) accel_cur[2];

        gyro_cur[0] = (((short) data[6] << 8) | data[7]);
        gyro_cur[1] = (((short) data[8] << 8) | data[9]);
        gyro_cur[2] = (((short) data[10] << 8) | data[11]);

        gyro[0] += (long) gyro_cur[0];
        gyro[1] += (long) gyro_cur[1];
        gyro[2] += (long) gyro_cur[2];
    }

    // Normalize the packets
    gyro[0] = (long) (((long long) gyro[0] << 16) / MPU6050_GCONFIG_SENS / packet_count);
    gyro[1] = (long) (((long long) gyro[1] << 16) / MPU6050_GCONFIG_SENS / packet_count);
    gyro[2] = (long) (((long long) gyro[2] << 16) / MPU6050_GCONFIG_SENS / packet_count);
    accel[0] = (long) (((long long) accel[0] << 16) / MPU6050_ACONFIG_SENS / packet_count);
    accel[1] = (long) (((long long) accel[1] << 16) / MPU6050_ACONFIG_SENS / packet_count);
    accel[2] = (long) (((long long) accel[2] << 16) / MPU6050_ACONFIG_SENS / packet_count);

    // Don't remove gravity
    if (accel[2] > 0L)
        accel[2] -= 65536L;
    else
        accel[2] += 65536L;
}

/* Test the accelerometer biases
 */ 
int MPU6050::accel_self_test(long *bias_regular, long *bias_st, float *st_shift_var)
{
    int j, result = 0;
    float st_shift[3], st_shift_cust;

    get_accel_prod_shift(st_shift);

    for (j = 0; j < 3; j++) {
        st_shift_cust = labs(bias_regular[j] - bias_st[j]) / 65536.f;
        if (st_shift[j]) {
            st_shift_var[j] = st_shift_cust / st_shift[j] - 1.f;
            if (fabs(st_shift_var[j]) > MPU6050_ACONFIG_MAX_VAR)
                result |= 1 << j;
        }
        else if ((st_shift_cust < MPU6050_ACONFIG_MIN_G) ||
                 (st_shift_cust > MPU6050_ACONFIG_MAX_G))
            result |= 1 << j;
    }

    return result;
}

/* Get the values of the self test registers
 * and calculate the factory trims from the
 * values of these registers
 */ 
void MPU6050::get_accel_prod_shift(float *st_shift)
{ 
    unsigned char tmp[4], shift_code[3], i;

    // Get the self-test values
    i2Cdev.readBytes(devAddr, MPU6050_RA_SELF_TEST_X, 4, tmp);

    shift_code[0] = ((tmp[0] & 0xE0) >> 3) | ((tmp[3] & 0x30) >> 4);
    shift_code[1] = ((tmp[1] & 0xE0) >> 3) | ((tmp[3] & 0x0C) >> 2);
    shift_code[2] = ((tmp[2] & 0xE0) >> 3) | (tmp[3] & 0x03);

    for (i = 0; i < 3; i++) {
        if (shift_code[i] == 0)
            st_shift[i] = 0.f;
        else
            st_shift[i] = (4096.0f*0.34f)*(pow((0.92f/0.34f), ((shift_code[0] - 1.0f)/30.0f)));
    }
}

int MPU6050::gyro_self_test(long *bias_regular, long *bias_st, float *st_shift_var)
{
    int j, result = 0;
    unsigned char tmp[3];
    float st_shift, st_shift_cust;

    i2Cdev.readBytes(devAddr, MPU6050_RA_SELF_TEST_X, 3, tmp);

    tmp[0] &= 0x1F;
    tmp[1] &= 0x1F;
    tmp[2] &= 0x1F;

    for (j = 0; j < 3; j++) {
        st_shift_cust = labs(bias_regular[j] - bias_st[j]) / 65536.f;
        if (tmp[j]) {
            st_shift = 3275.f / MPU6050_GCONFIG_SENS;
            while (--tmp[j])
                st_shift *= 1.046f;
            st_shift_var[j] = st_shift_cust / st_shift - 1.f;
            if (fabs(st_shift_var[j]) > MPU6050_GCONFIG_MAX_VAR)
                result |= 1 << j;
        }
        else if ((st_shift_cust < MPU6050_GCONFIG_MIN_DPS) ||
                 (st_shift_cust < MPU6050_GCONFIG_MAX_DPS))
            result |= 1 << j;
    }

    return result;
}

// AUX_VDDIO register (InvenSense demo code calls this RA_*G_OFFS_TC)

// SMPLRT_DIV register

/** Get gyroscope output rate divider.
 * The sensor register output, FIFO output, DMP sampling, Motion detection, Zero
 * Motion detection, and Free Fall detection are all based on the Sample Rate.
 * The Sample Rate is generated by dividing the gyroscope output rate by
 * SMPLRT_DIV:
 *
 * Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV)
 *
 * where Gyroscope Output Rate = 8kHz when the DLPF is disabled (DLPF_CFG = 0 or
 * 7), and 1kHz when the DLPF is enabled (see Register 26).
 *
 * Note: The accelerometer output rate is 1kHz. This means that for a Sample
 * Rate greater than 1kHz, the same accelerometer sample may be output to the
 * FIFO, DMP, and sensor registers more than once.
 *
 * For a diagram of the gyroscope and accelerometer signal paths, see Section 8
 * of the MPU-6000/MPU-6050 Product Specification document.
 *
 * @return Current sample rate
 * @see MPU6050_RA_SMPLRT_DIV
 */
uint8_t MPU6050::getRate()
{
    i2Cdev.readByte(devAddr, MPU6050_RA_SMPLRT_DIV, buffer);
    return buffer[0];
}
/** Set gyroscope sample rate divider.
 * @param rate New sample rate divider
 * @see getRate()
 * @see MPU6050_RA_SMPLRT_DIV
 */
void MPU6050::setRate(uint8_t rate)
{
    i2Cdev.writeByte(devAddr, MPU6050_RA_SMPLRT_DIV, rate);
}

uint32_t MPU6050::getSampRate()
{
    uint8_t samp_div;
    uint8_t dlpf_mode;
    uint32_t samp_rate;
    
    // Get the sample rate divider and dlpf mode
    i2Cdev.readByte(devAddr, MPU6050_RA_SMPLRT_DIV,buffer);
    samp_div = buffer[0];
    
    i2Cdev.readBits(devAddr, MPU6050_RA_CONFIG, MPU6050_CFG_DLPF_CFG_BIT, MPU6050_CFG_DLPF_CFG_LENGTH, buffer);
    dlpf_mode = buffer[0];
    
    if (dlpf_mode == 0 || dlpf_mode == 7)
        samp_rate = 800000 / (1+samp_div);
    else 
        samp_rate = 100000 / (1+samp_div);
        
    return samp_rate;
}

// CONFIG register


/** Get digital low-pass filter configuration.
 * The DLPF_CFG parameter sets the digital low pass filter configuration. It
 * also determines the internal sampling rate used by the device as shown in
 * the table below.
 *
 * Note: The accelerometer output rate is 1kHz. This means that for a Sample
 * Rate greater than 1kHz, the same accelerometer sample may be output to the
 * FIFO, DMP, and sensor registers more than once.
 *
 * <pre>
 *          |   ACCELEROMETER    |           GYROSCOPE
 * DLPF_CFG | Bandwidth | Delay  | Bandwidth | Delay  | Sample Rate
 * ---------+-----------+--------+-----------+--------+-------------
 * 0        | 260Hz     | 0ms    | 256Hz     | 0.98ms | 8kHz
 * 1        | 184Hz     | 2.0ms  | 188Hz     | 1.9ms  | 1kHz
 * 2        | 94Hz      | 3.0ms  | 98Hz      | 2.8ms  | 1kHz
 * 3        | 44Hz      | 4.9ms  | 42Hz      | 4.8ms  | 1kHz
 * 4        | 21Hz      | 8.5ms  | 20Hz      | 8.3ms  | 1kHz
 * 5        | 10Hz      | 13.8ms | 10Hz      | 13.4ms | 1kHz
 * 6        | 5Hz       | 19.0ms | 5Hz       | 18.6ms | 1kHz
 * 7        |   -- Reserved --   |   -- Reserved --   | Reserved
 * </pre>
 *
 * @return DLFP configuration
 * @see MPU6050_RA_CONFIG
 * @see MPU6050_CFG_DLPF_CFG_BIT
 * @see MPU6050_CFG_DLPF_CFG_LENGTH
 */
uint8_t MPU6050::getDLPFMode()
{
    i2Cdev.readBits(devAddr, MPU6050_RA_CONFIG, MPU6050_CFG_DLPF_CFG_BIT, MPU6050_CFG_DLPF_CFG_LENGTH, buffer);
    return buffer[0];
}
/** Set digital low-pass filter configuration.
 * @param mode New DLFP configuration setting
 * @see getDLPFBandwidth()
 * @see MPU6050_DLPF_BW_256
 * @see MPU6050_RA_CONFIG
 * @see MPU6050_CFG_DLPF_CFG_BIT
 * @see MPU6050_CFG_DLPF_CFG_LENGTH
 */
void MPU6050::setDLPFMode(uint8_t mode)
{
    i2Cdev.writeBits(devAddr, MPU6050_RA_CONFIG, MPU6050_CFG_DLPF_CFG_BIT, MPU6050_CFG_DLPF_CFG_LENGTH, mode);
}

// GYRO_CONFIG register

/** Get self-test enabled setting for gyroscope X axis.
 * @return Self-test enabled value
 * @see MPU6050_RA_GYRO_CONFIG
 */
bool MPU6050::getGyroXSelfTest()
{
    i2Cdev.readBit(devAddr, MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_XA_ST_BIT, buffer);
    return buffer[0];
}
/** Get self-test enabled setting for gyroscope X axis.
 * @param enabled Self-test enabled value
 * @see MPU6050_RA_GYRO_CONFIG
 */
void MPU6050::setGyroXSelfTest(bool enabled)
{
    i2Cdev.writeBit(devAddr, MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_XA_ST_BIT, enabled);
}

/** Get self-test enabled setting for gyroscope Y axis.
 * @return Self-test enabled value
 * @see MPU6050_RA_GYRO_CONFIG
 */
bool MPU6050::getGyroYSelfTest()
{
    i2Cdev.readBit(devAddr, MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_YA_ST_BIT, buffer);
    return buffer[0];
}
/** Get self-test enabled setting for gyroscope Y axis.
 * @param enabled Self-test enabled value
 * @see MPU6050_RA_GYRO_CONFIG
 */
void MPU6050::setGyroYSelfTest(bool enabled)
{
    i2Cdev.writeBit(devAddr, MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_YA_ST_BIT, enabled);
}

/** Get self-test enabled setting for gyroscope Z axis.
 * @return Self-test enabled value
 * @see MPU6050_RA_GYRO_CONFIG
 */
bool MPU6050::getGyroZSelfTest()
{
    i2Cdev.readBit(devAddr, MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_ZA_ST_BIT, buffer);
    return buffer[0];
}
/** Get self-test enabled setting for gyroscope Y axis.
 * @param enabled Self-test enabled value
 * @see MPU6050_RA_GYRO_CONFIG
 */
void MPU6050::setGyroZSelfTest(bool enabled)
{
    i2Cdev.writeBit(devAddr, MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_ZA_ST_BIT, enabled);
}

/** Get full-scale gyroscope range.
 * The FS_SEL parameter allows setting the full-scale range of the gyro sensors,
 * as described in the table below.
 *
 * <pre>
 * 0 = +/- 250 degrees/sec
 * 1 = +/- 500 degrees/sec
 * 2 = +/- 1000 degrees/sec
 * 3 = +/- 2000 degrees/sec
 * </pre>
 *
 * @return Current full-scale gyroscope range setting
 * @see MPU6050_GYRO_FS_250
 * @see MPU6050_RA_GYRO_CONFIG
 * @see MPU6050_GCONFIG_FS_SEL_BIT
 * @see MPU6050_GCONFIG_FS_SEL_LENGTH
 */
uint8_t MPU6050::getFullScaleGyroRange()
{
    i2Cdev.readBits(devAddr, MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_FS_SEL_BIT, MPU6050_GCONFIG_FS_SEL_LENGTH, buffer);
    return buffer[0];
}
/** Set full-scale gyroscope range.
 * @param range New full-scale gyroscope range value
 * @see getFullScaleRange()
 * @see MPU6050_GYRO_FS_250
 * @see MPU6050_RA_GYRO_CONFIG
 * @see MPU6050_GCONFIG_FS_SEL_BIT
 * @see MPU6050_GCONFIG_FS_SEL_LENGTH
 */
void MPU6050::setFullScaleGyroRange(uint8_t range)
{
    i2Cdev.writeBits(devAddr, MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_FS_SEL_BIT, MPU6050_GCONFIG_FS_SEL_LENGTH, range);
}

// ACCEL_CONFIG register

/** Get self-test enabled setting for accelerometer X axis.
 * @return Self-test enabled value
 * @see MPU6050_RA_ACCEL_CONFIG
 */
bool MPU6050::getAccelXSelfTest()
{
    i2Cdev.readBit(devAddr, MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_XA_ST_BIT, buffer);
    return buffer[0];
}
/** Get self-test enabled setting for accelerometer X axis.
 * @param enabled Self-test enabled value
 * @see MPU6050_RA_ACCEL_CONFIG
 */
void MPU6050::setAccelXSelfTest(bool enabled)
{
    i2Cdev.writeBit(devAddr, MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_XA_ST_BIT, enabled);
}
/** Get self-test enabled value for accelerometer Y axis.
 * @return Self-test enabled value
 * @see MPU6050_RA_ACCEL_CONFIG
 */
bool MPU6050::getAccelYSelfTest()
{
    i2Cdev.readBit(devAddr, MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_YA_ST_BIT, buffer);
    return buffer[0];
}
/** Get self-test enabled value for accelerometer Y axis.
 * @param enabled Self-test enabled value
 * @see MPU6050_RA_ACCEL_CONFIG
 */
void MPU6050::setAccelYSelfTest(bool enabled)
{
    i2Cdev.writeBit(devAddr, MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_YA_ST_BIT, enabled);
}
/** Get self-test enabled value for accelerometer Z axis.
 * @return Self-test enabled value
 * @see MPU6050_RA_ACCEL_CONFIG
 */
bool MPU6050::getAccelZSelfTest()
{
    i2Cdev.readBit(devAddr, MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_ZA_ST_BIT, buffer);
    return buffer[0];
}
/** Set self-test enabled value for accelerometer Z axis.
 * @param enabled Self-test enabled value
 * @see MPU6050_RA_ACCEL_CONFIG
 */
void MPU6050::setAccelZSelfTest(bool enabled)
{
    i2Cdev.writeBit(devAddr, MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_ZA_ST_BIT, enabled);
}
/** Get full-scale accelerometer range.
 * The FS_SEL parameter allows setting the full-scale range of the accelerometer
 * sensors, as described in the table below.
 *
 * <pre>
 * 0 = +/- 2g
 * 1 = +/- 4g
 * 2 = +/- 8g
 * 3 = +/- 16g
 * </pre>
 *
 * @return Current full-scale accelerometer range setting
 * @see MPU6050_ACCEL_FS_2
 * @see MPU6050_RA_ACCEL_CONFIG
 * @see MPU6050_ACONFIG_AFS_SEL_BIT
 * @see MPU6050_ACONFIG_AFS_SEL_LENGTH
 */
uint8_t MPU6050::getFullScaleAccelRange()
{
    i2Cdev.readBits(devAddr, MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_AFS_SEL_BIT, MPU6050_ACONFIG_AFS_SEL_LENGTH, buffer);
    return buffer[0];
}
/** Set full-scale accelerometer range.
 * @param range New full-scale accelerometer range setting
 * @see getFullScaleAccelRange()
 */
void MPU6050::setFullScaleAccelRange(uint8_t range)
{
    i2Cdev.writeBits(devAddr, MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_AFS_SEL_BIT, MPU6050_ACONFIG_AFS_SEL_LENGTH, range);
}
/** Get the high-pass filter configuration.
 * The DHPF is a filter module in the path leading to motion detectors (Free
 * Fall, Motion threshold, and Zero Motion). The high pass filter output is not
 * available to the data registers (see Figure in Section 8 of the MPU-6000/
 * MPU-6050 Product Specification document).
 *
 * The high pass filter has three modes:
 *
 * <pre>
 *    Reset: The filter output settles to zero within one sample. This
 *           effectively disables the high pass filter. This mode may be toggled
 *           to quickly settle the filter.
 *
 *    On:    The high pass filter will pass signals above the cut off frequency.
 *
 *    Hold:  When triggered, the filter holds the present sample. The filter
 *           output will be the difference between the input sample and the held
 *           sample.
 * </pre>
 *
 * <pre>
 * ACCEL_HPF | Filter Mode | Cut-off Frequency
 * ----------+-------------+------------------
 * 0         | Reset       | None
 * 1         | On          | 5Hz
 * 2         | On          | 2.5Hz
 * 3         | On          | 1.25Hz
 * 4         | On          | 0.63Hz
 * 7         | Hold        | None
 * </pre>
 *
 * @return Current high-pass filter configuration
 * @see MPU6050_DHPF_RESET
 * @see MPU6050_RA_ACCEL_CONFIG
 */
uint8_t MPU6050::getDHPFMode()
{
    i2Cdev.readBits(devAddr, MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_ACCEL_HPF_BIT, MPU6050_ACONFIG_ACCEL_HPF_LENGTH, buffer);
    return buffer[0];
}
/** Set the high-pass filter configuration.
 * @param bandwidth New high-pass filter configuration
 * @see setDHPFMode()
 * @see MPU6050_DHPF_RESET
 * @see MPU6050_RA_ACCEL_CONFIG
 */
void MPU6050::setDHPFMode(uint8_t bandwidth)
{
    i2Cdev.writeBits(devAddr, MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_ACCEL_HPF_BIT, MPU6050_ACONFIG_ACCEL_HPF_LENGTH, bandwidth);
}

// FF_THR register

/** Get free-fall event acceleration threshold.
 * This register configures the detection threshold for Free Fall event
 * detection. The unit of FF_THR is 1LSB = 2mg. Free Fall is detected when the
 * absolute value of the accelerometer measurements for the three axes are each
 * less than the detection threshold. This condition increments the Free Fall
 * duration counter (Register 30). The Free Fall interrupt is triggered when the
 * Free Fall duration counter reaches the time specified in FF_DUR.
 *
 * For more details on the Free Fall detection interrupt, see Section 8.2 of the
 * MPU-6000/MPU-6050 Product Specification document as well as Registers 56 and
 * 58 of this document.
 *
 * @return Current free-fall acceleration threshold value (LSB = 2mg)
 * @see MPU6050_RA_FF_THR
 */
uint8_t MPU6050::getFreefallDetectionThreshold()
{
    i2Cdev.readByte(devAddr, MPU6050_RA_FF_THR, buffer);
    return buffer[0];
}
/** Get free-fall event acceleration threshold.
 * @param threshold New free-fall acceleration threshold value (LSB = 2mg)
 * @see getFreefallDetectionThreshold()
 * @see MPU6050_RA_FF_THR
 */
void MPU6050::setFreefallDetectionThreshold(uint8_t threshold)
{
    i2Cdev.writeByte(devAddr, MPU6050_RA_FF_THR, threshold);
}

// FF_DUR register

/** Get free-fall event duration threshold.
 * This register configures the duration counter threshold for Free Fall event
 * detection. The duration counter ticks at 1kHz, therefore FF_DUR has a unit
 * of 1 LSB = 1 ms.
 *
 * The Free Fall duration counter increments while the absolute value of the
 * accelerometer measurements are each less than the detection threshold
 * (Register 29). The Free Fall interrupt is triggered when the Free Fall
 * duration counter reaches the time specified in this register.
 *
 * For more details on the Free Fall detection interrupt, see Section 8.2 of
 * the MPU-6000/MPU-6050 Product Specification document as well as Registers 56
 * and 58 of this document.
 *
 * @return Current free-fall duration threshold value (LSB = 1ms)
 * @see MPU6050_RA_FF_DUR
 */
uint8_t MPU6050::getFreefallDetectionDuration()
{
    i2Cdev.readByte(devAddr, MPU6050_RA_FF_DUR, buffer);
    return buffer[0];
}
/** Get free-fall event duration threshold.
 * @param duration New free-fall duration threshold value (LSB = 1ms)
 * @see getFreefallDetectionDuration()
 * @see MPU6050_RA_FF_DUR
 */
void MPU6050::setFreefallDetectionDuration(uint8_t duration)
{
    i2Cdev.writeByte(devAddr, MPU6050_RA_FF_DUR, duration);
}

// MOT_THR register

/** Get motion detection event acceleration threshold.
 * This register configures the detection threshold for Motion interrupt
 * generation. The unit of MOT_THR is 1LSB = 2mg. Motion is detected when the
 * absolute value of any of the accelerometer measurements exceeds this Motion
 * detection threshold. This condition increments the Motion detection duration
 * counter (Register 32). The Motion detection interrupt is triggered when the
 * Motion Detection counter reaches the time count specified in MOT_DUR
 * (Register 32).
 *
 * The Motion interrupt will indicate the axis and polarity of detected motion
 * in MOT_DETECT_STATUS (Register 97).
 *
 * For more details on the Motion detection interrupt, see Section 8.3 of the
 * MPU-6000/MPU-6050 Product Specification document as well as Registers 56 and
 * 58 of this document.
 *
 * @return Current motion detection acceleration threshold value (LSB = 2mg)
 * @see MPU6050_RA_MOT_THR
 */
uint8_t MPU6050::getMotionDetectionThreshold()
{
    i2Cdev.readByte(devAddr, MPU6050_RA_MOT_THR, buffer);
    return buffer[0];
}
/** Set free-fall event acceleration threshold.
 * @param threshold New motion detection acceleration threshold value (LSB = 2mg)
 * @see getMotionDetectionThreshold()
 * @see MPU6050_RA_MOT_THR
 */
void MPU6050::setMotionDetectionThreshold(uint8_t threshold)
{
    i2Cdev.writeByte(devAddr, MPU6050_RA_MOT_THR, threshold);
}

// MOT_DUR register

/** Get motion detection event duration threshold.
 * This register configures the duration counter threshold for Motion interrupt
 * generation. The duration counter ticks at 1 kHz, therefore MOT_DUR has a unit
 * of 1LSB = 1ms. The Motion detection duration counter increments when the
 * absolute value of any of the accelerometer measurements exceeds the Motion
 * detection threshold (Register 31). The Motion detection interrupt is
 * triggered when the Motion detection counter reaches the time count specified
 * in this register.
 *
 * For more details on the Motion detection interrupt, see Section 8.3 of the
 * MPU-6000/MPU-6050 Product Specification document.
 *
 * @return Current motion detection duration threshold value (LSB = 1ms)
 * @see MPU6050_RA_MOT_DUR
 */
uint8_t MPU6050::getMotionDetectionDuration()
{
    i2Cdev.readByte(devAddr, MPU6050_RA_MOT_DUR, buffer);
    return buffer[0];
}
/** Set motion detection event duration threshold.
 * @param duration New motion detection duration threshold value (LSB = 1ms)
 * @see getMotionDetectionDuration()
 * @see MPU6050_RA_MOT_DUR
 */
void MPU6050::setMotionDetectionDuration(uint8_t duration)
{
    i2Cdev.writeByte(devAddr, MPU6050_RA_MOT_DUR, duration);
}

// ZRMOT_THR register

/** Get zero motion detection event acceleration threshold.
 * This register configures the detection threshold for Zero Motion interrupt
 * generation. The unit of ZRMOT_THR is 1LSB = 2mg. Zero Motion is detected when
 * the absolute value of the accelerometer measurements for the 3 axes are each
 * less than the detection threshold. This condition increments the Zero Motion
 * duration counter (Register 34). The Zero Motion interrupt is triggered when
 * the Zero Motion duration counter reaches the time count specified in
 * ZRMOT_DUR (Register 34).
 *
 * Unlike Free Fall or Motion detection, Zero Motion detection triggers an
 * interrupt both when Zero Motion is first detected and when Zero Motion is no
 * longer detected.
 *
 * When a zero motion event is detected, a Zero Motion Status will be indicated
 * in the MOT_DETECT_STATUS register (Register 97). When a motion-to-zero-motion
 * condition is detected, the status bit is set to 1. When a zero-motion-to-
 * motion condition is detected, the status bit is set to 0.
 *
 * For more details on the Zero Motion detection interrupt, see Section 8.4 of
 * the MPU-6000/MPU-6050 Product Specification document as well as Registers 56
 * and 58 of this document.
 *
 * @return Current zero motion detection acceleration threshold value (LSB = 2mg)
 * @see MPU6050_RA_ZRMOT_THR
 */
uint8_t MPU6050::getZeroMotionDetectionThreshold()
{
    i2Cdev.readByte(devAddr, MPU6050_RA_ZRMOT_THR, buffer);
    return buffer[0];
}
/** Set zero motion detection event acceleration threshold.
 * @param threshold New zero motion detection acceleration threshold value (LSB = 2mg)
 * @see getZeroMotionDetectionThreshold()
 * @see MPU6050_RA_ZRMOT_THR
 */
void MPU6050::setZeroMotionDetectionThreshold(uint8_t threshold)
{
    i2Cdev.writeByte(devAddr, MPU6050_RA_ZRMOT_THR, threshold);
}

// ZRMOT_DUR register

/** Get zero motion detection event duration threshold.
 * This register configures the duration counter threshold for Zero Motion
 * interrupt generation. The duration counter ticks at 16 Hz, therefore
 * ZRMOT_DUR has a unit of 1 LSB = 64 ms. The Zero Motion duration counter
 * increments while the absolute value of the accelerometer measurements are
 * each less than the detection threshold (Register 33). The Zero Motion
 * interrupt is triggered when the Zero Motion duration counter reaches the time
 * count specified in this register.
 *
 * For more details on the Zero Motion detection interrupt, see Section 8.4 of
 * the MPU-6000/MPU-6050 Product Specification document, as well as Registers 56
 * and 58 of this document.
 *
 * @return Current zero motion detection duration threshold value (LSB = 64ms)
 * @see MPU6050_RA_ZRMOT_DUR
 */
uint8_t MPU6050::getZeroMotionDetectionDuration()
{
    i2Cdev.readByte(devAddr, MPU6050_RA_ZRMOT_DUR, buffer);
    return buffer[0];
}
/** Set zero motion detection event duration threshold.
 * @param duration New zero motion detection duration threshold value (LSB = 1ms)
 * @see getZeroMotionDetectionDuration()
 * @see MPU6050_RA_ZRMOT_DUR
 */
void MPU6050::setZeroMotionDetectionDuration(uint8_t duration)
{
    i2Cdev.writeByte(devAddr, MPU6050_RA_ZRMOT_DUR, duration);
}

// FIFO_EN register


// I2C_MST_CTRL register

/** Get multi-master enabled value.
 * Multi-master capability allows multiple I2C masters to operate on the same
 * bus. In circuits where multi-master capability is required, set MULT_MST_EN
 * to 1. This will increase current drawn by approximately 30uA.
 *
 * In circuits where multi-master capability is required, the state of the I2C
 * bus must always be monitored by each separate I2C Master. Before an I2C
 * Master can assume arbitration of the bus, it must first confirm that no other
 * I2C Master has arbitration of the bus. When MULT_MST_EN is set to 1, the
 * MPU-60X0's bus arbitration detection logic is turned on, enabling it to
 * detect when the bus is available.
 *
 * @return Current multi-master enabled value
 * @see MPU6050_RA_I2C_MST_CTRL
 */
bool MPU6050::getMultiMasterEnabled()
{
    i2Cdev.readBit(devAddr, MPU6050_RA_I2C_MST_CTRL, MPU6050_MULT_MST_EN_BIT, buffer);
    return buffer[0];
}
/** Set multi-master enabled value.
 * @param enabled New multi-master enabled value
 * @see getMultiMasterEnabled()
 * @see MPU6050_RA_I2C_MST_CTRL
 */
void MPU6050::setMultiMasterEnabled(bool enabled)
{
    i2Cdev.writeBit(devAddr, MPU6050_RA_I2C_MST_CTRL, MPU6050_MULT_MST_EN_BIT, enabled);
}
/** Get wait-for-external-sensor-data enabled value.
 * When the WAIT_FOR_ES bit is set to 1, the Data Ready interrupt will be
 * delayed until External Sensor data from the Slave Devices are loaded into the
 * EXT_SENS_DATA registers. This is used to ensure that both the internal sensor
 * data (i.e. from gyro and accel) and external sensor data have been loaded to
 * their respective data registers (i.e. the data is synced) when the Data Ready
 * interrupt is triggered.
 *
 * @return Current wait-for-external-sensor-data enabled value
 * @see MPU6050_RA_I2C_MST_CTRL
 */
bool MPU6050::getWaitForExternalSensorEnabled()
{
    i2Cdev.readBit(devAddr, MPU6050_RA_I2C_MST_CTRL, MPU6050_WAIT_FOR_ES_BIT, buffer);
    return buffer[0];
}
/** Set wait-for-external-sensor-data enabled value.
 * @param enabled New wait-for-external-sensor-data enabled value
 * @see getWaitForExternalSensorEnabled()
 * @see MPU6050_RA_I2C_MST_CTRL
 */
void MPU6050::setWaitForExternalSensorEnabled(bool enabled)
{
    i2Cdev.writeBit(devAddr, MPU6050_RA_I2C_MST_CTRL, MPU6050_WAIT_FOR_ES_BIT, enabled);
}

/** Get I2C master clock speed.
 * I2C_MST_CLK is a 4 bit unsigned value which configures a divider on the
 * MPU-60X0 internal 8MHz clock. It sets the I2C master clock speed according to
 * the following table:
 *
 * <pre>
 * I2C_MST_CLK | I2C Master Clock Speed | 8MHz Clock Divider
 * ------------+------------------------+-------------------
 * 0           | 348kHz                 | 23
 * 1           | 333kHz                 | 24
 * 2           | 320kHz                 | 25
 * 3           | 308kHz                 | 26
 * 4           | 296kHz                 | 27
 * 5           | 286kHz                 | 28
 * 6           | 276kHz                 | 29
 * 7           | 267kHz                 | 30
 * 8           | 258kHz                 | 31
 * 9           | 500kHz                 | 16
 * 10          | 471kHz                 | 17
 * 11          | 444kHz                 | 18
 * 12          | 421kHz                 | 19
 * 13          | 400kHz                 | 20
 * 14          | 381kHz                 | 21
 * 15          | 364kHz                 | 22
 * </pre>
 *
 * @return Current I2C master clock speed
 * @see MPU6050_RA_I2C_MST_CTRL
 */
uint8_t MPU6050::getMasterClockSpeed()
{
    i2Cdev.readBits(devAddr, MPU6050_RA_I2C_MST_CTRL, MPU6050_I2C_MST_CLK_BIT, MPU6050_I2C_MST_CLK_LENGTH, buffer);
    return buffer[0];
}
/** Set I2C master clock speed.
 * @reparam speed Current I2C master clock speed
 * @see MPU6050_RA_I2C_MST_CTRL
 */
void MPU6050::setMasterClockSpeed(uint8_t speed)
{
    i2Cdev.writeBits(devAddr, MPU6050_RA_I2C_MST_CTRL, MPU6050_I2C_MST_CLK_BIT, MPU6050_I2C_MST_CLK_LENGTH, speed);
}

// INT_PIN_CFG register

// INT_ENABLE register


// INT_STATUS register


// ACCEL_*OUT_* registers

/** Get raw 9-axis motion sensor readings (accel/gyro/compass).
 * FUNCTION NOT FULLY IMPLEMENTED YET.
 * @param ax 16-bit signed integer container for accelerometer X-axis value
 * @param ay 16-bit signed integer container for accelerometer Y-axis value
 * @param az 16-bit signed integer container for accelerometer Z-axis value
 * @param gx 16-bit signed integer container for gyroscope X-axis value
 * @param gy 16-bit signed integer container for gyroscope Y-axis value
 * @param gz 16-bit signed integer container for gyroscope Z-axis value
 * @param mx 16-bit signed integer container for magnetometer X-axis value
 * @param my 16-bit signed integer container for magnetometer Y-axis value
 * @param mz 16-bit signed integer container for magnetometer Z-axis value
 * @see getMotion6()
 * @see getAcceleration()
 * @see getRotation()
 * @see MPU6050_RA_ACCEL_XOUT_H
 */
void MPU6050::getMotion9(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz, int16_t* mx, int16_t* my, int16_t* mz)
{
    getMotion6(ax, ay, az, gx, gy, gz);
    // TODO: magnetometer integration
}
/** Get raw 6-axis motion sensor readings (accel/gyro).
 * Retrieves all currently available motion sensor values.
 * @param ax 16-bit signed integer container for accelerometer X-axis value
 * @param ay 16-bit signed integer container for accelerometer Y-axis value
 * @param az 16-bit signed integer container for accelerometer Z-axis value
 * @param gx 16-bit signed integer container for gyroscope X-axis value
 * @param gy 16-bit signed integer container for gyroscope Y-axis value
 * @param gz 16-bit signed integer container for gyroscope Z-axis value
 * @see getAcceleration()
 * @see getRotation()
 * @see MPU6050_RA_ACCEL_XOUT_H
 */
void MPU6050::getMotion6(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz)
{

#ifdef useDebugSerial
    //debugSerial.printf("MPU6050::getMotion6 start\n\r");
#endif

    i2Cdev.readBytes(devAddr, MPU6050_RA_ACCEL_XOUT_H, 14, buffer);

    *ax = (((int16_t)buffer[0]) << 8) | buffer[1];
    *ay = (((int16_t)buffer[2]) << 8) | buffer[3];
    *az = (((int16_t)buffer[4]) << 8) | buffer[5];
    *gx = (((int16_t)buffer[8]) << 8) | buffer[9];
    *gy = (((int16_t)buffer[10]) << 8) | buffer[11];
    *gz = (((int16_t)buffer[12]) << 8) | buffer[13];
    
#ifdef useDebugSerial
    //debugSerial.printf("%d, %d, %d, %d, %d, %d\n\r", *ax, *ay, *az, *gx, *gy, *gz);
#endif

}
/** Get 3-axis accelerometer readings.
 * These registers store the most recent accelerometer measurements.
 * Accelerometer measurements are written to these registers at the Sample Rate
 * as defined in Register 25.
 *
 * The accelerometer measurement registers, along with the temperature
 * measurement registers, gyroscope measurement registers, and external sensor
 * data registers, are composed of two sets of registers: an internal register
 * set and a user-facing read register set.
 *
 * The data within the accelerometer sensors' internal register set is always
 * updated at the Sample Rate. Meanwhile, the user-facing read register set
 * duplicates the internal register set's data values whenever the serial
 * interface is idle. This guarantees that a burst read of sensor registers will
 * read measurements from the same sampling instant. Note that if burst reads
 * are not used, the user is responsible for ensuring a set of single byte reads
 * correspond to a single sampling instant by checking the Data Ready interrupt.
 *
 * Each 16-bit accelerometer measurement has a full scale defined in ACCEL_FS
 * (Register 28). For each full scale setting, the accelerometers' sensitivity
 * per LSB in ACCEL_xOUT is shown in the table below:
 *
 * <pre>
 * AFS_SEL | Full Scale Range | LSB Sensitivity
 * --------+------------------+----------------
 * 0       | +/- 2g           | 8192 LSB/mg
 * 1       | +/- 4g           | 4096 LSB/mg
 * 2       | +/- 8g           | 2048 LSB/mg
 * 3       | +/- 16g          | 1024 LSB/mg
 * </pre>
 *
 * @param x 16-bit signed integer container for X-axis acceleration
 * @param y 16-bit signed integer container for Y-axis acceleration
 * @param z 16-bit signed integer container for Z-axis acceleration
 * @see MPU6050_RA_GYRO_XOUT_H
 */
void MPU6050::getAcceleration(int16_t* x, int16_t* y, int16_t* z)
{
    i2Cdev.readBytes(devAddr, MPU6050_RA_ACCEL_XOUT_H, 6, buffer);
    *x = (((int16_t)buffer[0]) << 8) | buffer[1];
    *y = (((int16_t)buffer[2]) << 8) | buffer[3];
    *z = (((int16_t)buffer[4]) << 8) | buffer[5];
}
/** Get X-axis accelerometer reading.
 * @return X-axis acceleration measurement in 16-bit 2's complement format
 * @see getMotion6()
 * @see MPU6050_RA_ACCEL_XOUT_H
 */
int16_t MPU6050::getAccelerationX()
{
    i2Cdev.readBytes(devAddr, MPU6050_RA_ACCEL_XOUT_H, 2, buffer);
    return (((int16_t)buffer[0]) << 8) | buffer[1];
}
/** Get Y-axis accelerometer reading.
 * @return Y-axis acceleration measurement in 16-bit 2's complement format
 * @see getMotion6()
 * @see MPU6050_RA_ACCEL_YOUT_H
 */
int16_t MPU6050::getAccelerationY()
{
    i2Cdev.readBytes(devAddr, MPU6050_RA_ACCEL_YOUT_H, 2, buffer);
    return (((int16_t)buffer[0]) << 8) | buffer[1];
}
/** Get Z-axis accelerometer reading.
 * @return Z-axis acceleration measurement in 16-bit 2's complement format
 * @see getMotion6()
 * @see MPU6050_RA_ACCEL_ZOUT_H
 */
int16_t MPU6050::getAccelerationZ()
{
    i2Cdev.readBytes(devAddr, MPU6050_RA_ACCEL_ZOUT_H, 2, buffer);
    return (((int16_t)buffer[0]) << 8) | buffer[1];
}

// TEMP_OUT_* registers

/** Get current internal temperature.
 * @return Temperature reading in 16-bit 2's complement format
 * @see MPU6050_RA_TEMP_OUT_H
 */
int16_t MPU6050::getTemperature()
{
    i2Cdev.readBytes(devAddr, MPU6050_RA_TEMP_OUT_H, 2, buffer);
    return (((int16_t)buffer[0]) << 8) | buffer[1];
}

// GYRO_*OUT_* registers

/** Get 3-axis gyroscope readings.
 * These gyroscope measurement registers, along with the accelerometer
 * measurement registers, temperature measurement registers, and external sensor
 * data registers, are composed of two sets of registers: an internal register
 * set and a user-facing read register set.
 * The data within the gyroscope sensors' internal register set is always
 * updated at the Sample Rate. Meanwhile, the user-facing read register set
 * duplicates the internal register set's data values whenever the serial
 * interface is idle. This guarantees that a burst read of sensor registers will
 * read measurements from the same sampling instant. Note that if burst reads
 * are not used, the user is responsible for ensuring a set of single byte reads
 * correspond to a single sampling instant by checking the Data Ready interrupt.
 *
 * Each 16-bit gyroscope measurement has a full scale defined in FS_SEL
 * (Register 27). For each full scale setting, the gyroscopes' sensitivity per
 * LSB in GYRO_xOUT is shown in the table below:
 *
 * <pre>
 * FS_SEL | Full Scale Range   | LSB Sensitivity
 * -------+--------------------+----------------
 * 0      | +/- 250 degrees/s  | 131 LSB/deg/s
 * 1      | +/- 500 degrees/s  | 65.5 LSB/deg/s
 * 2      | +/- 1000 degrees/s | 32.8 LSB/deg/s
 * 3      | +/- 2000 degrees/s | 16.4 LSB/deg/s
 * </pre>
 *
 * @param x 16-bit signed integer container for X-axis rotation
 * @param y 16-bit signed integer container for Y-axis rotation
 * @param z 16-bit signed integer container for Z-axis rotation
 * @see getMotion6()
 * @see MPU6050_RA_GYRO_XOUT_H
 */
void MPU6050::getRotation(int16_t* x, int16_t* y, int16_t* z)
{
    i2Cdev.readBytes(devAddr, MPU6050_RA_GYRO_XOUT_H, 6, buffer);
    *x = (((int16_t)buffer[0]) << 8) | buffer[1];
    *y = (((int16_t)buffer[2]) << 8) | buffer[3];
    *z = (((int16_t)buffer[4]) << 8) | buffer[5];
}
/** Get X-axis gyroscope reading.
 * @return X-axis rotation measurement in 16-bit 2's complement format
 * @see getMotion6()
 * @see MPU6050_RA_GYRO_XOUT_H
 */
int16_t MPU6050::getRotationX()
{
    i2Cdev.readBytes(devAddr, MPU6050_RA_GYRO_XOUT_H, 2, buffer);
    return (((int16_t)buffer[0]) << 8) | buffer[1];
}
/** Get Y-axis gyroscope reading.
 * @return Y-axis rotation measurement in 16-bit 2's complement format
 * @see getMotion6()
 * @see MPU6050_RA_GYRO_YOUT_H
 */
int16_t MPU6050::getRotationY()
{
    i2Cdev.readBytes(devAddr, MPU6050_RA_GYRO_YOUT_H, 2, buffer);
    return (((int16_t)buffer[0]) << 8) | buffer[1];
}
/** Get Z-axis gyroscope reading.
 * @return Z-axis rotation measurement in 16-bit 2's complement format
 * @see getMotion6()
 * @see MPU6050_RA_GYRO_ZOUT_H
 */
int16_t MPU6050::getRotationZ()
{
    i2Cdev.readBytes(devAddr, MPU6050_RA_GYRO_ZOUT_H, 2, buffer);
    return (((int16_t)buffer[0]) << 8) | buffer[1];
}

// EXT_SENS_DATA_* registers

// MOT_DETECT_STATUS register

/** Get X-axis negative motion detection interrupt status.
 * @return Motion detection status
 * @see MPU6050_RA_MOT_DETECT_STATUS
 * @see MPU6050_MOTION_MOT_XNEG_BIT
 */
bool MPU6050::getXNegMotionDetected()
{
    i2Cdev.readBit(devAddr, MPU6050_RA_MOT_DETECT_STATUS, MPU6050_MOTION_MOT_XNEG_BIT, buffer);
    return buffer[0];
}
/** Get X-axis positive motion detection interrupt status.
 * @return Motion detection status
 * @see MPU6050_RA_MOT_DETECT_STATUS
 * @see MPU6050_MOTION_MOT_XPOS_BIT
 */
bool MPU6050::getXPosMotionDetected()
{
    i2Cdev.readBit(devAddr, MPU6050_RA_MOT_DETECT_STATUS, MPU6050_MOTION_MOT_XPOS_BIT, buffer);
    return buffer[0];
}
/** Get Y-axis negative motion detection interrupt status.
 * @return Motion detection status
 * @see MPU6050_RA_MOT_DETECT_STATUS
 * @see MPU6050_MOTION_MOT_YNEG_BIT
 */
bool MPU6050::getYNegMotionDetected()
{
    i2Cdev.readBit(devAddr, MPU6050_RA_MOT_DETECT_STATUS, MPU6050_MOTION_MOT_YNEG_BIT, buffer);
    return buffer[0];
}
/** Get Y-axis positive motion detection interrupt status.
 * @return Motion detection status
 * @see MPU6050_RA_MOT_DETECT_STATUS
 * @see MPU6050_MOTION_MOT_YPOS_BIT
 */
bool MPU6050::getYPosMotionDetected()
{
    i2Cdev.readBit(devAddr, MPU6050_RA_MOT_DETECT_STATUS, MPU6050_MOTION_MOT_YPOS_BIT, buffer);
    return buffer[0];
}
/** Get Z-axis negative motion detection interrupt status.
 * @return Motion detection status
 * @see MPU6050_RA_MOT_DETECT_STATUS
 * @see MPU6050_MOTION_MOT_ZNEG_BIT
 */
bool MPU6050::getZNegMotionDetected()
{
    i2Cdev.readBit(devAddr, MPU6050_RA_MOT_DETECT_STATUS, MPU6050_MOTION_MOT_ZNEG_BIT, buffer);
    return buffer[0];
}
/** Get Z-axis positive motion detection interrupt status.
 * @return Motion detection status
 * @see MPU6050_RA_MOT_DETECT_STATUS
 * @see MPU6050_MOTION_MOT_ZPOS_BIT
 */
bool MPU6050::getZPosMotionDetected()
{
    i2Cdev.readBit(devAddr, MPU6050_RA_MOT_DETECT_STATUS, MPU6050_MOTION_MOT_ZPOS_BIT, buffer);
    return buffer[0];
}
/** Get zero motion detection interrupt status.
 * @return Motion detection status
 * @see MPU6050_RA_MOT_DETECT_STATUS
 * @see MPU6050_MOTION_MOT_ZRMOT_BIT
 */
bool MPU6050::getZeroMotionDetected()
{
    i2Cdev.readBit(devAddr, MPU6050_RA_MOT_DETECT_STATUS, MPU6050_MOTION_MOT_ZRMOT_BIT, buffer);
    return buffer[0];
}

// SIGNAL_PATH_RESET register

/** Reset gyroscope signal path.
 * The reset will revert the signal path analog to digital converters and
 * filters to their power up configurations.
 * @see MPU6050_RA_SIGNAL_PATH_RESET
 * @see MPU6050_PATHRESET_GYRO_RESET_BIT
 */
void MPU6050::resetGyroscopePath()
{
    i2Cdev.writeBit(devAddr, MPU6050_RA_SIGNAL_PATH_RESET, MPU6050_PATHRESET_GYRO_RESET_BIT, true);
}
/** Reset accelerometer signal path.
 * The reset will revert the signal path analog to digital converters and
 * filters to their power up configurations.
 * @see MPU6050_RA_SIGNAL_PATH_RESET
 * @see MPU6050_PATHRESET_ACCEL_RESET_BIT
 */
void MPU6050::resetAccelerometerPath()
{
    i2Cdev.writeBit(devAddr, MPU6050_RA_SIGNAL_PATH_RESET, MPU6050_PATHRESET_ACCEL_RESET_BIT, true);
}
/** Reset temperature sensor signal path.
 * The reset will revert the signal path analog to digital converters and
 * filters to their power up configurations.
 * @see MPU6050_RA_SIGNAL_PATH_RESET
 * @see MPU6050_PATHRESET_TEMP_RESET_BIT
 */
void MPU6050::resetTemperaturePath()
{
    i2Cdev.writeBit(devAddr, MPU6050_RA_SIGNAL_PATH_RESET, MPU6050_PATHRESET_TEMP_RESET_BIT, true);
}

// MOT_DETECT_CTRL register

/** Get accelerometer power-on delay.
 * The accelerometer data path provides samples to the sensor registers, Motion
 * detection, Zero Motion detection, and Free Fall detection modules. The
 * signal path contains filters which must be flushed on wake-up with new
 * samples before the detection modules begin operations. The default wake-up
 * delay, of 4ms can be lengthened by up to 3ms. This additional delay is
 * specified in ACCEL_ON_DELAY in units of 1 LSB = 1 ms. The user may select
 * any value above zero unless instructed otherwise by InvenSense. Please refer
 * to Section 8 of the MPU-6000/MPU-6050 Product Specification document for
 * further information regarding the detection modules.
 * @return Current accelerometer power-on delay
 * @see MPU6050_RA_MOT_DETECT_CTRL
 * @see MPU6050_DETECT_ACCEL_ON_DELAY_BIT
 */
uint8_t MPU6050::getAccelerometerPowerOnDelay()
{
    i2Cdev.readBits(devAddr, MPU6050_RA_MOT_DETECT_CTRL, MPU6050_DETECT_ACCEL_ON_DELAY_BIT, MPU6050_DETECT_ACCEL_ON_DELAY_LENGTH, buffer);
    return buffer[0];
}
/** Set accelerometer power-on delay.
 * @param delay New accelerometer power-on delay (0-3)
 * @see getAccelerometerPowerOnDelay()
 * @see MPU6050_RA_MOT_DETECT_CTRL
 * @see MPU6050_DETECT_ACCEL_ON_DELAY_BIT
 */
void MPU6050::setAccelerometerPowerOnDelay(uint8_t delay)
{
    i2Cdev.writeBits(devAddr, MPU6050_RA_MOT_DETECT_CTRL, MPU6050_DETECT_ACCEL_ON_DELAY_BIT, MPU6050_DETECT_ACCEL_ON_DELAY_LENGTH, delay);
}
/** Get Free Fall detection counter decrement configuration.
 * Detection is registered by the Free Fall detection module after accelerometer
 * measurements meet their respective threshold conditions over a specified
 * number of samples. When the threshold conditions are met, the corresponding
 * detection counter increments by 1. The user may control the rate at which the
 * detection counter decrements when the threshold condition is not met by
 * configuring FF_COUNT. The decrement rate can be set according to the
 * following table:
 *
 * <pre>
 * FF_COUNT | Counter Decrement
 * ---------+------------------
 * 0        | Reset
 * 1        | 1
 * 2        | 2
 * 3        | 4
 * </pre>
 *
 * When FF_COUNT is configured to 0 (reset), any non-qualifying sample will
 * reset the counter to 0. For further information on Free Fall detection,
 * please refer to Registers 29 to 32.
 *
 * @return Current decrement configuration
 * @see MPU6050_RA_MOT_DETECT_CTRL
 * @see MPU6050_DETECT_FF_COUNT_BIT
 */
uint8_t MPU6050::getFreefallDetectionCounterDecrement()
{
    i2Cdev.readBits(devAddr, MPU6050_RA_MOT_DETECT_CTRL, MPU6050_DETECT_FF_COUNT_BIT, MPU6050_DETECT_FF_COUNT_LENGTH, buffer);
    return buffer[0];
}
/** Set Free Fall detection counter decrement configuration.
 * @param decrement New decrement configuration value
 * @see getFreefallDetectionCounterDecrement()
 * @see MPU6050_RA_MOT_DETECT_CTRL
 * @see MPU6050_DETECT_FF_COUNT_BIT
 */
void MPU6050::setFreefallDetectionCounterDecrement(uint8_t decrement)
{
    i2Cdev.writeBits(devAddr, MPU6050_RA_MOT_DETECT_CTRL, MPU6050_DETECT_FF_COUNT_BIT, MPU6050_DETECT_FF_COUNT_LENGTH, decrement);
}
/** Get Motion detection counter decrement configuration.
 * Detection is registered by the Motion detection module after accelerometer
 * measurements meet their respective threshold conditions over a specified
 * number of samples. When the threshold conditions are met, the corresponding
 * detection counter increments by 1. The user may control the rate at which the
 * detection counter decrements when the threshold condition is not met by
 * configuring MOT_COUNT. The decrement rate can be set according to the
 * following table:
 *
 * <pre>
 * MOT_COUNT | Counter Decrement
 * ----------+------------------
 * 0         | Reset
 * 1         | 1
 * 2         | 2
 * 3         | 4
 * </pre>
 *
 * When MOT_COUNT is configured to 0 (reset), any non-qualifying sample will
 * reset the counter to 0. For further information on Motion detection,
 * please refer to Registers 29 to 32.
 *
 */
uint8_t MPU6050::getMotionDetectionCounterDecrement()
{
    i2Cdev.readBits(devAddr, MPU6050_RA_MOT_DETECT_CTRL, MPU6050_DETECT_MOT_COUNT_BIT, MPU6050_DETECT_MOT_COUNT_LENGTH, buffer);
    return buffer[0];
}
/** Set Motion detection counter decrement configuration.
 * @param decrement New decrement configuration value
 * @see getMotionDetectionCounterDecrement()
 * @see MPU6050_RA_MOT_DETECT_CTRL
 * @see MPU6050_DETECT_MOT_COUNT_BIT
 */
void MPU6050::setMotionDetectionCounterDecrement(uint8_t decrement)
{
    i2Cdev.writeBits(devAddr, MPU6050_RA_MOT_DETECT_CTRL, MPU6050_DETECT_MOT_COUNT_BIT, MPU6050_DETECT_MOT_COUNT_LENGTH, decrement);
}

// USER_CTRL register

/** Get FIFO enabled status.
 * When this bit is set to 0, the FIFO buffer is disabled. The FIFO buffer
 * cannot be written to or read from while disabled. The FIFO buffer's state
 * does not change unless the MPU-60X0 is power cycled.
 * @return Current FIFO enabled status
 * @see MPU6050_RA_USER_CTRL
 * @see MPU6050_USERCTRL_FIFO_EN_BIT
 */
bool MPU6050::getFIFOEnabled()
{
    i2Cdev.readBit(devAddr, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_FIFO_EN_BIT, buffer);
    return buffer[0];
}
/** Set FIFO enabled status.
 * @param enabled New FIFO enabled status
 * @see getFIFOEnabled()
 * @see MPU6050_RA_USER_CTRL
 * @see MPU6050_USERCTRL_FIFO_EN_BIT
 */
void MPU6050::setFIFOEnabled(bool enabled)
{
    i2Cdev.writeBit(devAddr, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_FIFO_EN_BIT, enabled);
}
/** Get I2C Master Mode enabled status.
 * When this mode is enabled, the MPU-60X0 acts as the I2C Master to the
 * external sensor slave devices on the auxiliary I2C bus. When this bit is
 * cleared to 0, the auxiliary I2C bus lines (AUX_DA and AUX_CL) are logically
 * driven by the primary I2C bus (SDA and SCL). This is a precondition to
 * enabling Bypass Mode. For further information regarding Bypass Mode, please
 * refer to Register 55.
 * @return Current I2C Master Mode enabled status
 * @see MPU6050_RA_USER_CTRL
 * @see MPU6050_USERCTRL_I2C_MST_EN_BIT
 */
bool MPU6050::getI2CMasterModeEnabled()
{
    i2Cdev.readBit(devAddr, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_I2C_MST_EN_BIT, buffer);
    return buffer[0];
}
/** Set I2C Master Mode enabled status.
 * @param enabled New I2C Master Mode enabled status
 * @see getI2CMasterModeEnabled()
 * @see MPU6050_RA_USER_CTRL
 * @see MPU6050_USERCTRL_I2C_MST_EN_BIT
 */
void MPU6050::setI2CMasterModeEnabled(bool enabled)
{
    i2Cdev.writeBit(devAddr, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_I2C_MST_EN_BIT, enabled);
}
/** Switch from I2C to SPI mode (MPU-6000 only)
 * If this is set, the primary SPI interface will be enabled in place of the
 * disabled primary I2C interface.
 */
void MPU6050::switchSPIEnabled(bool enabled)
{
    i2Cdev.writeBit(devAddr, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_I2C_IF_DIS_BIT, enabled);
}
/** Reset the FIFO.
 * This bit resets the FIFO buffer when set to 1 while FIFO_EN equals 0. This
 * bit automatically clears to 0 after the reset has been triggered.
 * @see MPU6050_RA_USER_CTRL
 * @see MPU6050_USERCTRL_FIFO_RESET_BIT
 */
void MPU6050::resetFIFO()
{
    i2Cdev.writeBit(devAddr, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_FIFO_RESET_BIT, true);
}
/** Reset the I2C Master.
 * This bit resets the I2C Master when set to 1 while I2C_MST_EN equals 0.
 * This bit automatically clears to 0 after the reset has been triggered.
 * @see MPU6050_RA_USER_CTRL
 * @see MPU6050_USERCTRL_I2C_MST_RESET_BIT
 */
void MPU6050::resetI2CMaster()
{
    i2Cdev.writeBit(devAddr, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_I2C_MST_RESET_BIT, true);
}
/** Reset all sensor registers and signal paths.
 * When set to 1, this bit resets the signal paths for all sensors (gyroscopes,
 * accelerometers, and temperature sensor). This operation will also clear the
 * sensor registers. This bit automatically clears to 0 after the reset has been
 * triggered.
 *
 * When resetting only the signal path (and not the sensor registers), please
 * use Register 104, SIGNAL_PATH_RESET.
 *
 * @see MPU6050_RA_USER_CTRL
 * @see MPU6050_USERCTRL_SIG_COND_RESET_BIT
 */
void MPU6050::resetSensors()
{
    i2Cdev.writeBit(devAddr, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_SIG_COND_RESET_BIT, true);
}

// PWR_MGMT_1 register

/** Get the PWR_MGMT_1 register contents.
 * @see MPU6050_RA_PWR_MGMT_1
 */
uint8_t MPU6050::getPwrMgmt1Reg()
{
    i2Cdev.readBits(devAddr, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_BIT, MPU6050_PWR1_LENGTH, buffer);
    return buffer[0];
}

/** Trigger a full device reset.
 * A small delay of ~50ms may be desirable after triggering a reset.
 * @see MPU6050_RA_PWR_MGMT_1
 * @see MPU6050_PWR1_DEVICE_RESET_BIT
 */
void MPU6050::reset()
{
    i2Cdev.writeBit(devAddr, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_DEVICE_RESET_BIT, true);
}
/** Get sleep mode status.
 * Setting the SLEEP bit in the register puts the device into very low power
 * sleep mode. In this mode, only the serial interface and internal registers
 * remain active, allowing for a very low standby current. Clearing this bit
 * puts the device back into normal mode. To save power, the individual standby
 * selections for each of the gyros should be used if any gyro axis is not used
 * by the application.
 * @return Current sleep mode enabled status
 * @see MPU6050_RA_PWR_MGMT_1
 * @see MPU6050_PWR1_SLEEP_BIT
 */
bool MPU6050::getSleepEnabled()
{
    i2Cdev.readBit(devAddr, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT, buffer);
    return buffer[0];
}
/** Set sleep mode status.
 * @param enabled New sleep mode enabled status
 * @see getSleepEnabled()
 * @see MPU6050_RA_PWR_MGMT_1
 * @see MPU6050_PWR1_SLEEP_BIT
 */
void MPU6050::setSleepEnabled(bool enabled)
{
    i2Cdev.writeBit(devAddr, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT, enabled);
}
/** Get wake cycle enabled status.
 * When this bit is set to 1 and SLEEP is disabled, the MPU-60X0 will cycle
 * between sleep mode and waking up to take a single sample of data from active
 * sensors at a rate determined by LP_WAKE_CTRL (register 108).
 * @return Current sleep mode enabled status
 * @see MPU6050_RA_PWR_MGMT_1
 * @see MPU6050_PWR1_CYCLE_BIT
 */
bool MPU6050::getWakeCycleEnabled()
{
    i2Cdev.readBit(devAddr, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_CYCLE_BIT, buffer);
    return buffer[0];
}
/** Set wake cycle enabled status.
 * @param enabled New sleep mode enabled status
 * @see getWakeCycleEnabled()
 * @see MPU6050_RA_PWR_MGMT_1
 * @see MPU6050_PWR1_CYCLE_BIT
 */
void MPU6050::setWakeCycleEnabled(bool enabled)
{
    i2Cdev.writeBit(devAddr, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_CYCLE_BIT, enabled);
}
/** Get temperature sensor enabled status.
 * Control the usage of the internal temperature sensor.
 *
 * Note: this register stores the *disabled* value, but for consistency with the
 * rest of the code, the function is named and used with standard true/false
 * values to indicate whether the sensor is enabled or disabled, respectively.
 *
 * @return Current temperature sensor enabled status
 * @see MPU6050_RA_PWR_MGMT_1
 * @see MPU6050_PWR1_TEMP_DIS_BIT
 */
bool MPU6050::getTempSensorEnabled()
{
    i2Cdev.readBit(devAddr, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_TEMP_DIS_BIT, buffer);
    return buffer[0] == 0; // 1 is actually disabled here
}
/** Set temperature sensor enabled status.
 * Note: this register stores the *disabled* value, but for consistency with the
 * rest of the code, the function is named and used with standard true/false
 * values to indicate whether the sensor is enabled or disabled, respectively.
 *
 * @param enabled New temperature sensor enabled status
 * @see getTempSensorEnabled()
 * @see MPU6050_RA_PWR_MGMT_1
 * @see MPU6050_PWR1_TEMP_DIS_BIT
 */
void MPU6050::setTempSensorEnabled(bool enabled)
{
    // 1 is actually disabled here
    i2Cdev.writeBit(devAddr, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_TEMP_DIS_BIT, !enabled);
}
/** Get clock source setting.
 * @return Current clock source setting
 * @see MPU6050_RA_PWR_MGMT_1
 * @see MPU6050_PWR1_CLKSEL_BIT
 * @see MPU6050_PWR1_CLKSEL_LENGTH
 */
uint8_t MPU6050::getClockSource()
{
    i2Cdev.readBits(devAddr, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_CLKSEL_BIT, MPU6050_PWR1_CLKSEL_LENGTH, buffer);
    return buffer[0];
}
/** Set clock source setting.
 * An internal 8MHz oscillator, gyroscope based clock, or external sources can
 * be selected as the MPU-60X0 clock source. When the internal 8 MHz oscillator
 * or an external source is chosen as the clock source, the MPU-60X0 can operate
 * in low power modes with the gyroscopes disabled.
 *
 * Upon power up, the MPU-60X0 clock source defaults to the internal oscillator.
 * However, it is highly recommended that the device be configured to use one of
 * the gyroscopes (or an external clock source) as the clock reference for
 * improved stability. The clock source can be selected according to the following table:
 *
 * <pre>
 * CLK_SEL | Clock Source
 * --------+--------------------------------------
 * 0       | Internal oscillator
 * 1       | PLL with X Gyro reference
 * 2       | PLL with Y Gyro reference
 * 3       | PLL with Z Gyro reference
 * 4       | PLL with external 32.768kHz reference
 * 5       | PLL with external 19.2MHz reference
 * 6       | Reserved
 * 7       | Stops the clock and keeps the timing generator in reset
 * </pre>
 *
 * @param source New clock source setting
 * @see getClockSource()
 * @see MPU6050_RA_PWR_MGMT_1
 * @see MPU6050_PWR1_CLKSEL_BIT
 * @see MPU6050_PWR1_CLKSEL_LENGTH
 */
void MPU6050::setClockSource(uint8_t source)
{
    i2Cdev.writeBits(devAddr, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_CLKSEL_BIT, MPU6050_PWR1_CLKSEL_LENGTH, source);
}

// PWR_MGMT_2 register

/** Get wake frequency in Accel-Only Low Power Mode.
 * The MPU-60X0 can be put into Accerlerometer Only Low Power Mode by setting
 * PWRSEL to 1 in the Power Management 1 register (Register 107). In this mode,
 * the device will power off all devices except for the primary I2C interface,
 * waking only the accelerometer at fixed intervals to take a single
 * measurement. The frequency of wake-ups can be configured with LP_WAKE_CTRL
 * as shown below:
 *
 * <pre>
 * LP_WAKE_CTRL | Wake-up Frequency
 * -------------+------------------
 * 0            | 1.25 Hz
 * 1            | 2.5 Hz
 * 2            | 5 Hz
 * 3            | 10 Hz
 * <pre>
 *
 * For further information regarding the MPU-60X0's power modes, please refer to
 * Register 107.
 *


// FIFO_COUNT* registers

/** Get current FIFO buffer size.
 * This value indicates the number of bytes stored in the FIFO buffer. This
 * number is in turn the number of bytes that can be read from the FIFO buffer
 * and it is directly proportional to the number of samples available given the
 * set of sensor data bound to be stored in the FIFO (register 35 and 36).
 * @return Current FIFO buffer size
 */
uint16_t MPU6050::getFIFOCount()
{
    i2Cdev.readBytes(devAddr, MPU6050_RA_FIFO_COUNTH, 2, buffer);
    return (((uint16_t)buffer[0]) << 8) | buffer[1];
}

// FIFO_R_W register

/** Get byte from FIFO buffer.
 * This register is used to read and write data from the FIFO buffer. Data is
 * written to the FIFO in order of register number (from lowest to highest). If
 * all the FIFO enable flags (see below) are enabled and all External Sensor
 * Data registers (Registers 73 to 96) are associated with a Slave device, the
 * contents of registers 59 through 96 will be written in order at the Sample
 * Rate.
 *
 * The contents of the sensor data registers (Registers 59 to 96) are written
 * into the FIFO buffer when their corresponding FIFO enable flags are set to 1
 * in FIFO_EN (Register 35). An additional flag for the sensor data registers
 * associated with I2C Slave 3 can be found in I2C_MST_CTRL (Register 36).
 *
 * If the FIFO buffer has overflowed, the status bit FIFO_OFLOW_INT is
 * automatically set to 1. This bit is located in INT_STATUS (Register 58).
 * When the FIFO buffer has overflowed, the oldest data will be lost and new
 * data will be written to the FIFO.
 *
 * If the FIFO buffer is empty, reading this register will return the last byte
 * that was previously read from the FIFO until new data is available. The user
 * should check FIFO_COUNT to ensure that the FIFO buffer is not read when
 * empty.
 *
 * @return Byte from FIFO buffer
 */
uint8_t MPU6050::getFIFOByte()
{
    i2Cdev.readByte(devAddr, MPU6050_RA_FIFO_R_W, buffer);
    return buffer[0];
}
void MPU6050::getFIFOBytes(uint8_t *data, uint8_t length)
{
    i2Cdev.readBytes(devAddr, MPU6050_RA_FIFO_R_W, length, data);
}
/** Write byte to FIFO buffer.
 * @see getFIFOByte()
 * @see MPU6050_RA_FIFO_R_W
 */
void MPU6050::setFIFOByte(uint8_t data)
{
    i2Cdev.writeByte(devAddr, MPU6050_RA_FIFO_R_W, data);
}

// WHO_AM_I register

/** Get Device ID.
 * This register is used to verify the identity of the device (0b110100, 0x34).
 * @return Device ID (6 bits only! should be 0x34)
 * @see MPU6050_RA_WHO_AM_I
 * @see MPU6050_WHO_AM_I_BIT
 * @see MPU6050_WHO_AM_I_LENGTH
 */
uint8_t MPU6050::getDeviceID()
{
    i2Cdev.readBits(devAddr, MPU6050_RA_WHO_AM_I, MPU6050_WHO_AM_I_BIT, MPU6050_WHO_AM_I_LENGTH, buffer);
    return buffer[0];
}
/** Set Device ID.
 * Write a new ID into the WHO_AM_I register (no idea why this should ever be
 * necessary though).
 * @param id New device ID to set.
 * @see getDeviceID()
 * @see MPU6050_RA_WHO_AM_I
 * @see MPU6050_WHO_AM_I_BIT
 * @see MPU6050_WHO_AM_I_LENGTH
 */
void MPU6050::setDeviceID(uint8_t id)
{
    i2Cdev.writeBits(devAddr, MPU6050_RA_WHO_AM_I, MPU6050_WHO_AM_I_BIT, MPU6050_WHO_AM_I_LENGTH, id);
}

// ======== UNDOCUMENTED/DMP REGISTERS/METHODS ========

/* Gets the sensor data and
 * uses functions to convert
 * it into acceleration and
 * yaw, pitch, and roll data
 */

/* Compute the quaternion
 * ax, ay, and az will get normalized
 * gx, gy, gz won't change
 * q[0], q[1], q[2], q[3] will get populated
 */

// XG_OFFS_TC register

uint8_t MPU6050::getOTPBankValid()
{
    i2Cdev.readBit(devAddr, MPU6050_RA_XG_OFFS_TC, MPU6050_TC_OTP_BNK_VLD_BIT, buffer);
    return buffer[0];
}
void MPU6050::setOTPBankValid(bool enabled)
{
    i2Cdev.writeBit(devAddr, MPU6050_RA_XG_OFFS_TC, MPU6050_TC_OTP_BNK_VLD_BIT, enabled);
}
int8_t MPU6050::getXGyroOffset()
{
    i2Cdev.readBits(devAddr, MPU6050_RA_XG_OFFS_TC, MPU6050_TC_OFFSET_BIT, MPU6050_TC_OFFSET_LENGTH, buffer);
    return buffer[0];
}
void MPU6050::setXGyroOffset(int8_t offset)
{
    i2Cdev.writeBits(devAddr, MPU6050_RA_XG_OFFS_TC, MPU6050_TC_OFFSET_BIT, MPU6050_TC_OFFSET_LENGTH, offset);
}

// YG_OFFS_TC register

int8_t MPU6050::getYGyroOffset()
{
    i2Cdev.readBits(devAddr, MPU6050_RA_YG_OFFS_TC, MPU6050_TC_OFFSET_BIT, MPU6050_TC_OFFSET_LENGTH, buffer);
    return buffer[0];
}
void MPU6050::setYGyroOffset(int8_t offset)
{
    i2Cdev.writeBits(devAddr, MPU6050_RA_YG_OFFS_TC, MPU6050_TC_OFFSET_BIT, MPU6050_TC_OFFSET_LENGTH, offset);
}

// ZG_OFFS_TC register

int8_t MPU6050::getZGyroOffset()
{
    i2Cdev.readBits(devAddr, MPU6050_RA_ZG_OFFS_TC, MPU6050_TC_OFFSET_BIT, MPU6050_TC_OFFSET_LENGTH, buffer);
    return buffer[0];
}
void MPU6050::setZGyroOffset(int8_t offset)
{
    i2Cdev.writeBits(devAddr, MPU6050_RA_ZG_OFFS_TC, MPU6050_TC_OFFSET_BIT, MPU6050_TC_OFFSET_LENGTH, offset);
}

// X_FINE_GAIN register

int8_t MPU6050::getXFineGain()
{
    i2Cdev.readByte(devAddr, MPU6050_RA_X_FINE_GAIN, buffer);
    return buffer[0];
}
void MPU6050::setXFineGain(int8_t gain)
{
    i2Cdev.writeByte(devAddr, MPU6050_RA_X_FINE_GAIN, gain);
}

// Y_FINE_GAIN register

int8_t MPU6050::getYFineGain()
{
    i2Cdev.readByte(devAddr, MPU6050_RA_Y_FINE_GAIN, buffer);
    return buffer[0];
}
void MPU6050::setYFineGain(int8_t gain)
{
    i2Cdev.writeByte(devAddr, MPU6050_RA_Y_FINE_GAIN, gain);
}

// Z_FINE_GAIN register

int8_t MPU6050::getZFineGain()
{
    i2Cdev.readByte(devAddr, MPU6050_RA_Z_FINE_GAIN, buffer);
    return buffer[0];
}
void MPU6050::setZFineGain(int8_t gain)
{
    i2Cdev.writeByte(devAddr, MPU6050_RA_Z_FINE_GAIN, gain);
}

// XA_OFFS_* registers

int16_t MPU6050::getXAccelOffset()
{
    i2Cdev.readBytes(devAddr, MPU6050_RA_XA_OFFS_H, 2, buffer);
    return (((int16_t)buffer[0]) << 8) | buffer[1];
}
void MPU6050::setXAccelOffset(int16_t offset)
{
    i2Cdev.writeWord(devAddr, MPU6050_RA_XA_OFFS_H, offset);
}

// YA_OFFS_* register

int16_t MPU6050::getYAccelOffset()
{
    i2Cdev.readBytes(devAddr, MPU6050_RA_YA_OFFS_H, 2, buffer);
    return (((int16_t)buffer[0]) << 8) | buffer[1];
}
void MPU6050::setYAccelOffset(int16_t offset)
{
    i2Cdev.writeWord(devAddr, MPU6050_RA_YA_OFFS_H, offset);
}

// ZA_OFFS_* register

int16_t MPU6050::getZAccelOffset()
{
    i2Cdev.readBytes(devAddr, MPU6050_RA_ZA_OFFS_H, 2, buffer);
    return (((int16_t)buffer[0]) << 8) | buffer[1];
}
void MPU6050::setZAccelOffset(int16_t offset)
{
    i2Cdev.writeWord(devAddr, MPU6050_RA_ZA_OFFS_H, offset);
}

// XG_OFFS_USR* registers

int16_t MPU6050::getXGyroOffsetUser()
{
    i2Cdev.readBytes(devAddr, MPU6050_RA_XG_OFFS_USRH, 2, buffer);
    return (((int16_t)buffer[0]) << 8) | buffer[1];
}
void MPU6050::setXGyroOffsetUser(int16_t offset)
{
    i2Cdev.writeWord(devAddr, MPU6050_RA_XG_OFFS_USRH, offset);
}

// YG_OFFS_USR* register

int16_t MPU6050::getYGyroOffsetUser()
{
    i2Cdev.readBytes(devAddr, MPU6050_RA_YG_OFFS_USRH, 2, buffer);
    return (((int16_t)buffer[0]) << 8) | buffer[1];
}
void MPU6050::setYGyroOffsetUser(int16_t offset)
{
    i2Cdev.writeWord(devAddr, MPU6050_RA_YG_OFFS_USRH, offset);
}

// ZG_OFFS_USR* register

int16_t MPU6050::getZGyroOffsetUser()
{
    i2Cdev.readBytes(devAddr, MPU6050_RA_ZG_OFFS_USRH, 2, buffer);
    return (((int16_t)buffer[0]) << 8) | buffer[1];
}
void MPU6050::setZGyroOffsetUser(int16_t offset)
{
    i2Cdev.writeWord(devAddr, MPU6050_RA_ZG_OFFS_USRH, offset);
}

