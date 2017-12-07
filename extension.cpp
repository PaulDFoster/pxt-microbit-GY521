#include "pxt.h"
#include "smallMPU6050.h"

using namespace pxt;

namespace microbit_GY521 {

MPU6050 mpu;

int16_t ax=0, ay=0, az=0;
int16_t gx=0, gy=0, gz=0;

#define Gyr_Gain 0.00763358 
float dt=0.02;

float AccelX;
float AccelY;
float GyroY;
float mixY;

    // ================================================================
    // ===                CALIBRATION_ROUTINE                       ===
    // ================================================================
    // Simple calibration - just average first few readings to subtract
    // from the later data
    //%
    bool calibrate_Sensors(int *offSets) {
    mpu.initialize();

    // Validate parameters are passed correctly
    /*
    if(offSets[0]!=-2694) return false;
    if(offSets[1]!=593) return false;
    if(offSets[2]!=485) return false;
    if(offSets[3]!=75) return false;
    if(offSets[4]!=15) return false;
    if(offSets[5]!=21) return false;
    */

    mpu.setXAccelOffset(offSets[0]);
    mpu.setYAccelOffset(offSets[1]);
    mpu.setZAccelOffset(offSets[2]);
    mpu.setXGyroOffset(offSets[3]);
    mpu.setYGyroOffset(offSets[4]);
    mpu.setZGyroOffset(offSets[5]);

    // Discard the first reading (don't know if this is needed or
    // not, however, it won't hurt.)
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    return true;
    }

    //%
    bool testConnection(){
    return mpu.testConnection();
    }

    // Expose the raw accel and gyro data
    //%
    void getMotion6(){
        mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    }
    //%
    int readAccelX(){
        return (int)ax;
    }
    //%
    int readAccelY(){
        return (int)ay;
    }
    //%
    int readAccelZ(){
        return (int)az;
    }
    //% 
    int readGyroX(){
        return (int)gx;
    }
    //% 
    int readGyroY(){
        return (int)gy;
    }
    //% 
    int readGyroZ(){
        return (int)gz;
    }

    // Complementary filters used by the self balancing robot solution
    //%
    int computeY()
    {
          mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
          
          GyroY = Gyr_Gain * (gy)*-1;
    
          AccelY = (atan2(ay, az) * 180 / PI);
          AccelX = (atan2(ax, az) * 180 / PI);
          
          float K = 0.98;
          float A = K / (K + dt);
    
          mixY = A *(mixY+GyroY*dt) + (1-A)*AccelX; 
    
          mpu.resetFIFO();

          return mixY;
    }

    //%
    void setXGyroOffset(int value){
        mpu.setXGyroOffset(value);
    }
    //%
    int getXGyroOffset(){
        return mpu.getXGyroOffset();
    }

    //%
    void setYGyroOffset(int value){
        mpu.setYGyroOffset(value);
    }
    //%
    int getYGyroOffset(){
        return mpu.getYGyroOffset();
    }

    //%
    void setZGyroOffset(int value){
        mpu.setZGyroOffset(value);
    }

    //%
    int getZGyroOffset(){
        return mpu.getZGyroOffset();
    }

    //%
    void setZAccelOffset(int value){
        mpu.setZAccelOffset(value);
    }

    //%
    int getYAccelOffset(){
        return mpu.getYAccelOffset();
    }

    //%
    int getXAccelOffset(){
        return mpu.getXAccelOffset();
    }

    //%
    int getZAccelOffset(){
        return mpu.getZAccelOffset();
    }
}
