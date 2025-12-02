/*
    NOTE acceleration (accelerometer) is in m/s^2, rotation (gyro) is in rad/s

    TODO/TOTRY:
    add low pass filtering during calibration 

*/



#ifndef IMU_H
#define IMU_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <math.h>

struct roll_pitch {     // to hold roll and pitch vals
  float roll;
  float pitch;
};

struct raw_imu_vals {       // to hold raw imu vals

    float ax, ay, az, wx, wy;

};

class IMU {

    private: 
    Adafruit_MPU6050 mpu;

    float prev_roll = 0.0;
    float prev_pitch = 0.0;

    // offsets
    raw_imu_vals raw_offsets;


    public: 

    // constructor 
    IMU(Adafruit_MPU6050 _mpu);


    // METHODS

    // setup IMU
    void SetupIMU();

    // set raw offsets
    void SetRawOffsets(float _ax,float _ay,float _az,float _wx,float _wy);

    //get and print raw offsets
    void GetRawOffsets();

    // read raw IMU data
    raw_imu_vals ReadRawIMU();

    // read raw imu data and subtract offsets 
    raw_imu_vals ReadCalibIMU();

    // calibrate raw IMU offsets 
    void CalibRawOffsets();

    // complementary filter method 
    roll_pitch CompFilter(raw_imu_vals _raw_calib_imu_vals, float _alpha, float _delta_t);

    // more advanced comp filter with accelerometer reconditioning
    roll_pitch CompFilterv2(raw_imu_vals _v, float _alpha, float _delta_t);

};




#endif