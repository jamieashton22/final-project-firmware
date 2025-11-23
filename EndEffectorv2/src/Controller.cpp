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
    
    
    float servoCommand = 90.0 + (kp * error);
    
    servoCommand = constrain(servoCommand, 0.0, 180.0);
    
    return servoCommand;

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
    
    float output = ((2/3)*(90 + (kp * error) + (ki * accum_error)));

    if(output > 120){
        output = 120;           // clamping output
    }
    else if (output < 20){
        output = 20;
    }

    prev_error = error;
    return(output);

}

float Controller::PDController(float _measurement, float _dt){
    
    float error = setpoint - _measurement;
    
    // Derivative term (rate of change of error)
    float d_error = (error - prev_error) / _dt;  // You need dt passed in!
    
    // Control output
    float servoCommand = 90.0 + (kp * error) + (kd * d_error);
    
    // Clamp
    servoCommand = constrain(servoCommand, 0.0, 180.0);
    
    prev_error = error;
    return servoCommand;
}