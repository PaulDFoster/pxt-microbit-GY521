#include "pxt.h"
#include "MPU6050.h"
using namespace pxt;
namespace microbit_GY521 {
    //%
    int extfun(int x, int y) {
        return x + y;
    }

    MPU6050 mpu;
    float unfiltered_gyro_angle_y;
    float last_gyro_y_angle;
    float last_y_angle;
    float GYRO_FACTOR;
    uint8_t READ_FS_SEL = 0; // hard coding the range value of the MPU
    unsigned long last_read_time;
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
    const float RADIANS_TO_DEGREES = 57.2958; //180/3.14159
    const float base_y_gyro = 0;
    // Apply the complementary filter to figure out the change in angle - choice of alpha is
    // estimated now.  Alpha depends on the sampling rate...
    const float alpha = 0.96;

    //%
    int GetIntStatus(){
        return mpu.getIntStatus();
    }

    //%
    float ComputeY(){
        mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
        unsigned long t_now = system_timer_get_period();
        float dt =(t_now - last_read_time)/1000.0;
        float gyro_y = (gy - base_y_gyro) / GYRO_FACTOR;
        float accel_y = ay; // - base_y_accel;
        float accel_angle_y = atan(-1*ax/sqrt(pow(accel_y,2) + pow(az,2)))*RADIANS_TO_DEGREES;
        float gyro_angle_y = gyro_y*dt + last_y_angle;
        float unfiltered_gyro_angle_y = gyro_y*dt + last_gyro_y_angle;
        float angle_y = alpha*gyro_angle_y + (1.0 - alpha)*accel_angle_y;

        last_y_angle = angle_y;
        last_gyro_y_angle = unfiltered_gyro_angle_y;
        last_read_time = t_now;

        return angle_y + 180;
    }

    //%
    void Initialise(){
        mpu.initialize();
        GYRO_FACTOR = 131.0 / pow (2 , READ_FS_SEL);
        ax = 0;
        ay = 0;
        az = 0;
        gx = 0;
        gy = 0;
        gz = 0;
    }

    //%
    void SetXGyroOffset(int value){
        mpu.setXGyroOffset(value);
    }

    //%
    void SetYGyroOffset(int value){
        mpu.setYGyroOffset(value);
    }

    //%
    void SetZGyroOffset(int value){
        mpu.setZGyroOffset(value);
    }

    }
