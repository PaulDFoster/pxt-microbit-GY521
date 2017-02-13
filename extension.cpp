#include "pxt.h"
#include "MPU6050.h"

using namespace pxt;

namespace microbit_GY521 {

MPU6050 mpu;

float unfiltered_gyro_angle_y;
float last_gyro_y_angle;
float last_y_angle;
float unfiltered_gyro_angle_x;
float last_gyro_x_angle;
float last_x_angle;

uint8_t READ_FS_SEL = 0; // hard coding the range value of the MPU
float GYRO_FACTOR = 0; //131.0 / pow(2, READ_FS_SEL);
unsigned long last_y_read_time;
unsigned long last_x_read_time;

int16_t ax=0, ay=0, az=0;
int16_t gx=0, gy=0, gz=0;

// Apply the complementary filter to figure out the change in angle - choice of alpha is
// estimated now.  Alpha depends on the sampling rate...
const float alpha = 0.98;
const float RADIANS_TO_DEGREES = 57.2958; //180/3.14159

//  Use the following global variables 
//  to calibrate the gyroscope sensor and accelerometer readings
float    base_x_gyro = 0;
float    base_y_gyro = 0;
float    base_z_gyro = 0;
float    base_x_accel = 0;
float    base_y_accel = 0;
float    base_z_accel = 0;

// ================================================================
// ===                CALIBRATION_ROUTINE                       ===
// ================================================================
// Simple calibration - just average first few readings to subtract
// from the later data
//%
void calibrate_Sensors() {
  int       num_readings = 10;
  GYRO_FACTOR = 131.0 / pow (2 , READ_FS_SEL);
  mpu.initialize();

  
  // Discard the first reading (don't know if this is needed or
  // not, however, it won't hurt.)
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  
  // Read and average the raw values
  for (int i = 0; i < num_readings; i++) {
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    base_x_gyro += gx;
    base_y_gyro += gy;
    base_z_gyro += gz;
    base_x_accel += ax;
    base_y_accel += ay;
    base_y_accel += az;
  }
  
  base_x_gyro /= num_readings;
  base_y_gyro /= num_readings;
  base_z_gyro /= num_readings;
  base_x_accel /= num_readings;
  base_y_accel /= num_readings;
  base_z_accel /= num_readings;
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
    unsigned long t_now = system_timer_current_time();
    
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    float dt = (t_now - last_y_read_time) / 1000.0; // Convert to seconds
    float gyro_y = (gy - base_y_gyro) / GYRO_FACTOR;

    float accel_y = ay - base_y_accel;
    float accel_angle_y = atan(-1 * ax / sqrt(pow(accel_y, 2) + pow(az, 2))) * RADIANS_TO_DEGREES;
    
    // R*dAngle/dt is the error of the accelerometer on distance R
    // R * (accel_angle_y - last_y_angle)/dt 
    // 200mm * (1/0.1) = 2000 error in accelerometer reading?
    float accelError = -1.28; //1.28 = 0.128 * (1 / 0.1);

  //  accel_angle_y = accel_angle_y - accelError;

    float gyro_angle_y = gyro_y * dt + last_y_angle;
    // Filtered angle
    float angle_y = alpha * gyro_angle_y + (1.0 - alpha) * accel_angle_y;

    last_y_angle = angle_y;
    last_y_read_time = t_now;

    mpu.resetFIFO();
    

    return (int)(((angle_y) + 180));
    }


//%
int computeX()
{
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    unsigned long t_now = system_timer_current_time();
    
    float gyro_x = (gx - base_x_gyro) / GYRO_FACTOR;

    float accel_x = ax; // - base_y_accel;
    
    float accel_angle_x = atan(ay/sqrt(pow(accel_x,2) + pow(az,2)))*RADIANS_TO_DEGREES;

    // Compute the filtered gyro angles
    float dt = (t_now - last_x_read_time) / 1000.0;
    float gyro_angle_x = gyro_x*dt + last_x_angle;
    // Compute the drifting gyro angles
    float unfiltered_gyro_angle_x = gyro_x*dt + last_gyro_x_angle;

    float angle_x = alpha*gyro_angle_x + (1.0 - alpha)*accel_angle_x;

    last_x_angle = angle_x;
    last_gyro_x_angle = unfiltered_gyro_angle_x;
    last_x_read_time = t_now;

    return (int)((angle_x + 180));
    }

/*
void initialise(){
    mpu.initialize();
}
*/

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
