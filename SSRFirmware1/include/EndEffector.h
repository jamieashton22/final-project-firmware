
// HEADER FILE FOR ENDEFFECTOR CLASS

#ifndef ENDEFFECTOR_H
#define ENDEFFECTOR_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <math.h>
#include <Servo.h>
#include "Receiver.h"

typedef struct roll_pitch {     // data type to store roll and pitch angles

    float roll;
    float pitch;

};

typedef struct IMU_values {     // data type to store raw IMU values 

    float acceleration_x, acceleration_y, acceleration_z;
    float velocity_x, velocity_y, velocity_z;              
    // don't need temperature     

};

class EndEffector{

    private:

        int roll_pin;
        int pitch_pin;
        Servo roll_servo;
        Servo pitch_servo;
        Adafruit_MPU6050 imu;
        float prev_roll;
        float prev_pitch;

    public:

        EndEffector(int _roll_pin, int _pitch_pin);
        void EndEffectorSetup();
        roll_pitch CompFilter(IMU_values _imu_reading, float _alpha, float _delta_t);
        IMU_values getRawIMU();

};

#endif