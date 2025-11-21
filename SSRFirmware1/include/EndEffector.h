
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

typedef struct servo_positions {

    float roll_position;
    float pitch_position;

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
    
        int roll_feedback_pin;
        int pitch_feedback_pin;

        float angle_max;
        float angle_min;

        float roll_v_min;    // minimum voltage from calibration
        float roll_v_max;     // max voltage from calibration
        float pitch_v_min;
        float pitch_v_max;



    public:

        EndEffector(int _roll_pin, int _pitch_pin, int _roll_feedback_pin, int _pitch_feedback_pin);
        void EndEffectorSetup();

        roll_pitch CompFilter(IMU_values _imu_reading, float _alpha, float _delta_t);
        IMU_values getRawIMU();
        String GetIMUMessage(float _alpha,float _dt);

        void CalibrateFeedback();   // to calibrate the analogue feedback from servos
        servo_positions ReadServos();

        void writeEEServos(int _roll_write, int _pitch_write);

};

#endif