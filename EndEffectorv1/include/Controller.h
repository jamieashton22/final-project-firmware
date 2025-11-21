
#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <Arduino.h>

class Controller {

    private: 
        
        float kp, ki, kd;
        float setpoint;

        
    public:

        
        Controller(float _kp, float _ki, float _kd, float _setpoint);
        float PController(float _measurement);
        float PIController(float _measurement);

        float accum_error;
        float prev_error;
};

#endif