
// HEADER FILE FOR MAIN ROBOTIC ARM

#ifndef ROBOTARM_H
#define ROBOTARM_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <math.h>
#include <Servo.h>
#include "Receiver.h"

// typedef struct main_joint_angles {      // struct to hold joint angles

//     int q0;
//     int q1;
//     int q2;

// };

class RobotArm{

    private:

        int base_pin;
        int shoulder_pin;
        int elbow_pin;
        Servo base;
        Servo shoulder;
        Servo elbow;

    public:

        RobotArm(int _base_pin, int _shoulder_pin, int _elbow_pin);
        void ArmSetup();
        void UpdatePosition(int _q0, int _q1, int _q2); 
        void UpdateSinglePosition(char _j, int _q);
        void UpdatePositionGeneral(char _j, int _q, int _q0, int _q1, int _q2);


};




#endif