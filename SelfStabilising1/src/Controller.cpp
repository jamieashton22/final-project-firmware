#include "Controller.h"


Controller::Controller(float _kp, float _ki, float _kd, float _setpoint){

    kp = _kp;
    ki = _ki;
    kd = _kd;
    setpoint = _setpoint;
    accum_error = 0;
    prev_error = 0;

}

float Controller::PController(float _measurement){

    float error = setpoint - _measurement;
    float output = (90 + (kp * error));
  
  return(output);

}

float Controller::PIController(float _measurement){

    float error = setpoint - _measurement;
    accum_error += error;

    if(abs(error) > 0.01){
        if((prev_error * error) < 0){
            accum_error = 0;                    // clegg integrator 
        }
    }


    float max_accum_error = 20/ki;          //tune  
    if(accum_error > max_accum_error){
        accum_error = max_accum_error;                  // setting maximum integral term value 
    }
    else if (accum_error < (- max_accum_error))
    {
        accum_error = - max_accum_error;
    }
    
    float output = (90 + (kp * error) + (ki * accum_error));

    if(output > 180){
        output = 180;           // clamping output
    }
    else if (output < 0){
        output = 0;
    }

    prev_error = error;
    return(output);

}

float Controller::LEDController(float _measurement){

    float error = setpoint - _measurement;

    // 255 to be maximum aka default, when error is zero
    //take abs of error 
    float output = (255 - (10 * kp * (abs(error))));
    return(output);

}