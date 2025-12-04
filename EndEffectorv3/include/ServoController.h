#ifndef SERVOCONTROLLER_H

#define SERVOCONTROLLER_H

#include <Arduino.h>
#include <Servo.h>

class ServoController { 

    private:

        // passed in to constructor
        int signal_pin;
        int feedback_pin;       
        float servo_tolerance;  // allowable range of real servo val to written
        float tilt_threshold;   // don't correct tilts smaller than this 
        float update_rate;      // min time to allow servos to reach pos.

        float servo_range = 180.0; 
        unsigned long servoPrevTime = 0.0;
        unsigned long servoCurrTime = 0.0;
        int servo_true = 0.0; // to hold actual servo vals

        float servo_write = 90.0;

        float dt_sec = 0.0; // dt for controller
        float accum_error; // for I term
        float prev_error;

        // temp hardcoded Vmin and Vmax for servo analogue feedback
        float v_max = 3.07;
        float v_min = 0.22;

        //flags
        bool servo_reached = false; // flag to store if servo reached position
        bool servo_time_passed = false;



    public: 

    // necessary values & objects
    Servo servo;

    // constructor

    ServoController(int _signal_pin, int _feedback_pin, float _servo_tolerance, float _tilt_threshold, float _update_rate);
    
    // methods

    //setup/attach the servos and write to start position
    void setupServo();

    // calibrate the servos analogue feedback 
    float calibrateServo(int _pos);

    // read the analog feedback to find true servo position 
    int readServo();

    // calculate if servo time passed
    bool servoTimePassed();

    // check if servo reached written pos.- pass true position from readServo in
    bool servoReachedPosition(int _true_pos);

    /* UPDATE SERVO ------------------------------------------
    this ones important

    flow in main will be something like

    if (servoTimePassed() && servoReachedPosition()){
        read IMU
        pass imu angle into update function and 
        execute update function
    }

    and update function will:
    reset servo timer
    apply deadband if statement
    apply the control to the correction (PID)
    create new roll_write
    constrain new roll_write
    write new roll_write 

    */

    void updateServo(float _correction);

    //P controller
    float PController(float _setpoint, float _measurement, float _kp);

    //PI controller
    float PIController(float _setpoint, float _measurement, float _kp, float _ki);

    //PD controller
    float PDController(float _setpoint, float _measurement, float _kp, float _kd);

    //PID controller
    float PIDController(float _setpoint, float _measurement, float _kp, float _ki, float _kd);

};





#endif