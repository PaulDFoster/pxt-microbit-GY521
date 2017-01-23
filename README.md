for PXT/microbit

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

The microbit_GY521 is built using the I2CDev code for the MPU-6050 (GY-521).
To allow this code to work on the Microbit significant parts of the functionality have been removed from the I2CDev source.

Example code for the package and for using the package with a balancing robot is provided at the bottom of this file.

A small subset of functions are made available:

    initialise()  - required to prepare to MPU6050 instantiation
    calibrate_Sensors()  - calculates sensor calibrations
    testConnection()  - validates MPU6050 can be found on I2C bus
    computeX() - calculates a complementary filter value for X axis in degrees
    computeY() - calculates a complementary filter value for Y axis in degrees
    setXGyroOffset(value: number ) - sets axis 
    setYGyroOffset(value: number)
    setZGyroOffset(value: number) 
    setZAccelOffset(value: number)   
    setYAccelOffset(value: number)   
    setXAccelOffset(value: number)   
    getZGyroOffset()   - reads axis offset. Note MPU6050 doesn't store set values without specific additional actions
    getYGyroOffset()    
    getXGyroOffset()   
    getYAccelOffset()  
    getXAccelOffset()  
    getZAccelOffset()  
    getMotion6()  - Reads device accelerometer and gyro registers
    readAccelX()  - returned the axis raw value
    readAccelY()  
    readAccelZ()  
    readGyroX()   
    readGyroY()   
    readGyroZ()   


Specifically, the follow functions are not available, other functions are retained but not exposed:

         AUX_VDDIO register
        uint8_t getAuxVDDIOLevel();
        void setAuxVDDIOLevel(uint8_t level);
        
         CONFIG register
        uint8_t getExternalFrameSync();
        void setExternalFrameSync(uint8_t sync);

         FIFO_EN register
        bool getTempFIFOEnabled();
        void setTempFIFOEnabled(bool enabled);
        bool getXGyroFIFOEnabled();
        void setXGyroFIFOEnabled(bool enabled);
        bool getYGyroFIFOEnabled();
        void setYGyroFIFOEnabled(bool enabled);
        bool getZGyroFIFOEnabled();
        void setZGyroFIFOEnabled(bool enabled);
        bool getAccelFIFOEnabled();
        void setAccelFIFOEnabled(bool enabled);
        bool getSlave2FIFOEnabled();
        void setSlave2FIFOEnabled(bool enabled);
        bool getSlave1FIFOEnabled();
        void setSlave1FIFOEnabled(bool enabled);
        bool getSlave0FIFOEnabled();
        void setSlave0FIFOEnabled(bool enabled);

         I2C_MST_CTRL register
        bool getSlave3FIFOEnabled();
        void setSlave3FIFOEnabled(bool enabled);
        bool getSlaveReadWriteTransitionEnabled();
        void setSlaveReadWriteTransitionEnabled(bool enabled);

                 I2C_SLV* registers (Slave 0-3)
        uint8_t getSlaveAddress(uint8_t num);
        void setSlaveAddress(uint8_t num, uint8_t address);
        uint8_t getSlaveRegister(uint8_t num);
        void setSlaveRegister(uint8_t num, uint8_t reg);
        bool getSlaveEnabled(uint8_t num);
        void setSlaveEnabled(uint8_t num, bool enabled);
        bool getSlaveWordByteSwap(uint8_t num);
        void setSlaveWordByteSwap(uint8_t num, bool enabled);
        bool getSlaveWriteMode(uint8_t num);
        void setSlaveWriteMode(uint8_t num, bool mode);
        bool getSlaveWordGroupOffset(uint8_t num);
        void setSlaveWordGroupOffset(uint8_t num, bool enabled);
        uint8_t getSlaveDataLength(uint8_t num);
        void setSlaveDataLength(uint8_t num, uint8_t length);

         I2C_SLV* registers (Slave 4)
        uint8_t getSlave4Address();
        void setSlave4Address(uint8_t address);
        uint8_t getSlave4Register();
        void setSlave4Register(uint8_t reg);
        void setSlave4OutputByte(uint8_t data);
        bool getSlave4Enabled();
        void setSlave4Enabled(bool enabled);
        bool getSlave4InterruptEnabled();
        void setSlave4InterruptEnabled(bool enabled);
        bool getSlave4WriteMode();
        void setSlave4WriteMode(bool mode);
        uint8_t getSlave4MasterDelay();
        void setSlave4MasterDelay(uint8_t delay);
        uint8_t getSlate4InputByte();

         I2C_MST_STATUS register
        bool getPassthroughStatus();
        bool getSlave4IsDone();
        bool getLostArbitration();
        bool getSlave4Nack();
        bool getSlave3Nack();
        bool getSlave2Nack();
        bool getSlave1Nack();
        bool getSlave0Nack();

         INT_PIN_CFG register
        bool getInterruptMode();
        void setInterruptMode(bool mode);
        bool getInterruptDrive();
        void setInterruptDrive(bool drive);
        bool getInterruptLatch();
        void setInterruptLatch(bool latch);
        bool getInterruptLatchClear();
        void setInterruptLatchClear(bool clear);
        bool getFSyncInterruptLevel();
        void setFSyncInterruptLevel(bool level);
        bool getFSyncInterruptEnabled();
        void setFSyncInterruptEnabled(bool enabled);
        bool getI2CBypassEnabled();
        void setI2CBypassEnabled(bool enabled);
        bool getClockOutputEnabled();
        void setClockOutputEnabled(bool enabled);

         INT_ENABLE register
        uint8_t getIntEnabled();
        void setIntEnabled(uint8_t enabled);
        bool getIntFreefallEnabled();
        void setIntFreefallEnabled(bool enabled);
        bool getIntMotionEnabled();
        void setIntMotionEnabled(bool enabled);
        bool getIntZeroMotionEnabled();
        void setIntZeroMotionEnabled(bool enabled);
        bool getIntFIFOBufferOverflowEnabled();
        void setIntFIFOBufferOverflowEnabled(bool enabled);
        bool getIntI2CMasterEnabled();
        void setIntI2CMasterEnabled(bool enabled);
        bool getIntDataReadyEnabled();
        void setIntDataReadyEnabled(bool enabled);

         INT_STATUS register
        uint8_t getIntStatus();
        bool getIntFreefallStatus();
        bool getIntMotionStatus();
        bool getIntZeroMotionStatus();
        bool getIntFIFOBufferOverflowStatus();
        bool getIntI2CMasterStatus();
        bool getIntDataReadyStatus();

         I2C_SLV*_DO register
        void setSlaveOutputByte(uint8_t num, uint8_t data);

         I2C_MST_DELAY_CTRL register
        bool getExternalShadowDelayEnabled();
        void setExternalShadowDelayEnabled(bool enabled);
        bool getSlaveDelayEnabled(uint8_t num);
        void setSlaveDelayEnabled(uint8_t num, bool enabled);

         PWR_MGMT_2 register
        uint8_t getWakeFrequency();
        void setWakeFrequency(uint8_t frequency);
        bool getStandbyXAccelEnabled();
        void setStandbyXAccelEnabled(bool enabled);
        bool getStandbyYAccelEnabled();
        void setStandbyYAccelEnabled(bool enabled);
        bool getStandbyZAccelEnabled();
        void setStandbyZAccelEnabled(bool enabled);
        bool getStandbyXGyroEnabled();
        void setStandbyXGyroEnabled(bool enabled);
        bool getStandbyYGyroEnabled();
        void setStandbyYGyroEnabled(bool enabled);
        bool getStandbyZGyroEnabled();
        void setStandbyZGyroEnabled(bool enabled);

                 INT_ENABLE register (DMP functions)
        bool getIntPLLReadyEnabled();
        void setIntPLLReadyEnabled(bool enabled);
        bool getIntDMPEnabled();
        void setIntDMPEnabled(bool enabled);
        
         DMP_INT_STATUS
        bool getDMPInt5Status();
        bool getDMPInt4Status();
        bool getDMPInt3Status();
        bool getDMPInt2Status();
        bool getDMPInt1Status();
        bool getDMPInt0Status();

         INT_STATUS register (DMP functions)
        bool getIntPLLReadyStatus();
        bool getIntDMPStatus();
        
         USER_CTRL register (DMP functions)
        bool getDMPEnabled();
        void setDMPEnabled(bool enabled);
        void resetDMP();
        
         BANK_SEL register
        void setMemoryBank(uint8_t bank, bool prefetchEnabled=false, bool userBank=false);
        
         MEM_START_ADDR register
        void setMemoryStartAddress(uint8_t address);
        
         MEM_R_W register
        uint8_t readMemoryByte();
        void writeMemoryByte(uint8_t data);
        void readMemoryBlock(uint8_t *data, uint16_t dataSize, uint8_t bank=0, uint8_t address=0);
        bool writeMemoryBlock(const uint8_t *data, uint16_t dataSize, uint8_t bank=0, uint8_t address=0, bool verify=true, bool useProgMem=false);
        bool writeProgMemoryBlock(const uint8_t *data, uint16_t dataSize, uint8_t bank=0, uint8_t address=0, bool verify=true);

        bool writeDMPConfigurationSet(const uint8_t *data, uint16_t dataSize, bool useProgMem=false);
        bool writeProgDMPConfigurationSet(const uint8_t *data, uint16_t dataSize);

         DMP_CFG_1 register
        uint8_t getDMPConfig1();
        void setDMPConfig1(uint8_t config);

         DMP_CFG_2 register
        uint8_t getDMPConfig2();
        void setDMPConfig2(uint8_t config);


***************************************************************************************************
This code demonstrates tests of the GY521 code package
***************************************************************************************************
microbit_GY521.initialise();

if (microbit_GY521.testConnection())
    serial.writeLine("GY521 connected");
else
    serial.writeLine("GY521 not found");

// supply your own gyro offsets here, scaled for min sensitivity
// These don't appear to get set, as the get values differ
microbit_GY521.setXGyroOffset(-96);
let xgo = microbit_GY521.getXGyroOffset();
serial.writeString("xgo -96 ");
serial.writeNumber(xgo);
serial.writeLine(" ");
microbit_GY521.setYGyroOffset(54); //54
let ygo = microbit_GY521.getYGyroOffset();
serial.writeString("ygo 54 ");
serial.writeNumber(ygo);
serial.writeLine(" ");
microbit_GY521.setZGyroOffset(87); //128
let zgo = microbit_GY521.getZGyroOffset();
serial.writeString("zgo 87 ");
serial.writeNumber(zgo);
serial.writeLine(" ");
microbit_GY521.setZAccelOffset(1393); // 1688 factory default for my test chip
let zao = microbit_GY521.getZAccelOffset();
serial.writeString("zao 1393 ");
serial.writeNumber(zao);
serial.writeLine(" ");

//Test raw reads
microbit_GY521.getMotion6();
serial.writeNumber(microbit_GY521.readGyroX());
serial.writeString(" ");
serial.writeNumber(microbit_GY521.readGyroY());
serial.writeString(" ");
serial.writeNumber(microbit_GY521.readGyroZ());
serial.writeString(" ");
serial.writeNumber(microbit_GY521.readAccelX());
serial.writeString(" ");
serial.writeNumber(microbit_GY521.readAccelY());
serial.writeString(" ");
serial.writeNumber(microbit_GY521.readAccelZ());
serial.writeLine(" ");

***************************************************************************************************
The code below demonstrates using the GY521 package to drive a self-balancing robot
***************************************************************************************************

microbit_GY521.initialise();

microbit_GY521.calibrate_Sensors(); // required to increase accuracy of readings

let y: number = 0
let x: number = 0
let speed: number = 0
let bigSwitch = false;
let releaseSwitch = false;

while (1)
{
    input.onButtonPressed(Button.A, () => { releaseSwitch = true; });
    input.onButtonPressed(Button.B, () => { releaseSwitch = false; });
    
    x = microbit_GY521.computeX()
    y = microbit_GY521.computeY()
    serial.writeNumber(x);
    serial.writeString(" ");
    serial.writeNumber(y);
    serial.writeString(" ");

    // Bug in Angle code is causing int overflow
    // and of device falls over, stop the motors. This simplifies having no off switch
    if (y > 140 && y < 230) {
        bigSwitch = true;
    }
    else {
        bigSwitch = false;
    }

    if (releaseSwitch) bigSwitch = true;

    speed = PID(x)
    serial.writeNumber(speed);
    serial.writeString(" ");

    speed = PID(y)
    serial.writeNumber(speed);
    serial.writeLine(" ");

    motorStop();

  /*  
    if(bigSwitch){
        motorController(speed);
    }
    else {
        serial.writeLine(" Angle overflow, motors stopped");
        motorStop();
    }
  */
}

// PID state setup
let lastTime = 0;
let SampleTime = 10;
let output = 0;

// ************************************
// Set point 190, Kp 70, Ki 275 Kd 0. Falls over
//           190, Kp 60, Ki 275 kd 0, 190 top far, over reaching
//           188, Kp 60, Ki 275 Kd 0, 188 to fat, still over reaching
//           186, Kp 60, Ki 275, kd0, 186 seemed almost too less
//           186, Kp 70, Ki 275, kd0 feels very close, under reaching
//           187, Kp 70, Ki 275, kd 0 less stable then 186
//           186, Kp 80, ki 275, kd 0 shorter oscillation of wheels to and fro
//           186, Kp 80, Ki 300, kd 0 almost
//           186, kp 90, Ki 0, kd 0 quite neat
//           186, kp 100 ki 0 kd 0 feels like set point should be greater
//           187, kp 100, ki 0 kd 0 most stable for Kp
//           187, kp 110, ki 0 kd 0 ditto
//           188, kp 110, ki 0 kd 0 not as good as 187
//           187, kp 120, ki, 0, kd 0 hmmm good
//           187, kp 130, ki 0, kd 0 needs greater angle
//           188, kp 130, ki 0, kd 0 ditto
//           187, kp 120, ki 0, kd 0 hmmm
//           187, kp 120, ki 0, kd 0
//           188, kp 120, ki 240, kd 0
//           190, kp 110, ki 280 aggitated, 260 better
//           190, kp 110, ki 220 interesting
//           190, kp 110, ki 230 interesting
//           190, kp 110, ki 235, kd 2 almost
//           190, kp 110, ki 235, kd 3?

let setPoint = 190;
let Kp = 110; 
let Ki = 235;                                                                                                                                                                                                                                     
let Kd = 0;
let ITerm = 0;
let outMax = 255;
let outMin = -255;
let lastInput = 0;

function PID(input:number) : number
{

    let now = game.currentTime();
    let timeChange = (now - lastTime);

    if (timeChange >= SampleTime)
    {

        let error = setPoint - input;
        ITerm += (Ki * error);
        if (ITerm > outMax)
            ITerm = outMax;
        else if (ITerm < outMin)
            ITerm = outMin;
        let dInput = (input - lastInput);
        /*Compute PID Output*/
        output = Kp * error + ITerm - Kd * dInput;

        if (output > outMax)
            output = outMax;
        else if (output < outMin)
            output = outMin;

        lastInput = input;
        lastTime = now;

   }
    return output;
}

function motorStop() {
        pins.digitalWritePin(DigitalPin.P13, 0)
        pins.digitalWritePin(DigitalPin.P14, 0)
        pins.digitalWritePin(DigitalPin.P15, 0)
        pins.digitalWritePin(DigitalPin.P16, 0)   
}

function motorController(speed: number)
{
    if (speed > 0) {
        pins.digitalWritePin(DigitalPin.P13, 1)
        pins.digitalWritePin(DigitalPin.P14, 0)
        pins.digitalWritePin(DigitalPin.P15, 1)
        pins.digitalWritePin(DigitalPin.P16, 0)

    }
    if (speed < 0) {
        pins.digitalWritePin(DigitalPin.P13, 0)
        pins.digitalWritePin(DigitalPin.P14, 1)
        pins.digitalWritePin(DigitalPin.P15, 0)
        pins.digitalWritePin(DigitalPin.P16, 1)
    }
 
    pins.servoWritePin(AnalogPin.P0, speed)
    pins.servoWritePin(AnalogPin.P1, speed)
 
}