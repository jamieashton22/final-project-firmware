#include "ServoController.h"

ServoController::ServoController(int _signal_pin, int _feedback_pin, float _servo_tolerance, float _tilt_threshold, float _update_rate){

    signal_pin = _signal_pin;
    feedback_pin = _feedback_pin;
    servo_tolerance = _servo_tolerance;     // initialise private members
    tilt_threshold = _tilt_threshold;
    update_rate = _update_rate;
    accum_error = 0.0;
    prev_error = 0.0;

}

void ServoController::setupServo(){

    pinMode(feedback_pin, INPUT);

    servo.attach(signal_pin);
    servo.write(90);
    servoPrevTime = millis();   // <- MUST HAVE THIS
    delay(1000); // wait a second to let it reach position

}

float ServoController::calibrateServo(int _pos){

  double v_calib = 0.0;

  servo.write(_pos); // write servo to position

  delay(1000);        // wait a second to allow servo to reach position

  v_calib = ((analogRead(feedback_pin)) * 5.0)/1023.0;

  return(v_calib);

}

int ServoController::readServo(){

  float v_meas = ((analogRead(feedback_pin))*5.0/1023.0 );
  float normalised_pos = (v_meas - v_min)/(v_max- v_min);
  normalised_pos = constrain(normalised_pos, 0.0, 1.0);

  servo_true = int(normalised_pos * servo_range);

  return(servo_true);


}

bool ServoController::servoTimePassed(){

    servoCurrTime = millis();

    dt_sec = (servoCurrTime - servoPrevTime) / 1000.0;  // calculate dt for controller here

    if(servoCurrTime - servoPrevTime >= update_rate) { //if sufficient time passed
        servo_time_passed = true;
    }
    else{
        servo_time_passed = false;
    }
    return(servo_time_passed);

}

bool ServoController::servoReachedPosition(int _true_pos){  //pass true position into the function from read

    if(abs(_true_pos - round(servo_write)) <= servo_tolerance){
        servo_reached = true;
    }
    else{
        servo_reached = false;
    }
    return(servo_reached);
}

    
    // void ServoController::updateServo(float _imu_reading, float _dt){

    //     // reset servo timer    
    //     servoPrevTime = servoCurrTime;

    //     // apply deadband
    //     if(abs(_imu_reading) > tilt_threshold) {

    //         // CONTROL GOES HERE
    //         // just with a hardcoded P for now, change this when controller written
    //         float correction = 0.4 * _imu_reading;
    //         servo_write = servo_true + correction;
    //         servo_write = constrain(servo_write, 30, 150);  // safe limits
    //         servo.write(round(servo_write));


    //     }

    //     servo_reached = false;


    // }

void ServoController::updateServo(float _correction) {

        //reset servo timer
        servoPrevTime = servoCurrTime;

        servo_write = servo_true - _correction;
        servo_write = constrain(servo_write, 30, 150);  // safe limits
        servo.write(round(servo_write));

        servo_reached = false;

}

    // P controller

    // pass in a float error SETPOINT - _imu_reading 
    // return a correction 
    // pass in an imu reading, return a correction

float ServoController::PController(float _setpoint, float _measurement, float _kp){

        // calculate error = r - y
        float error = _setpoint - _measurement;

        if(abs(error) > tilt_threshold){    // deadband - don't adjust for neglegible tilts 

            // P controller
            float correction = _kp * error;


            return(correction);
        }

        else{
            return 0.0;
        }

    }

float ServoController::PIController(float _setpoint, float _measurement, float _kp, float _ki){

        // calculate error r-y
        float error = _setpoint - _measurement;

        if(abs(error) > tilt_threshold) {   // deadband - dont adjust for neglegible tilts

            accum_error += error * dt_sec;

            // CLEGG INTEGRATOR - reset accumulated error when platform hits level


            if((prev_error * error) < 0){       // when change in error sign (i.e platform has hit level)

                accum_error = 0; 

            }


            
            float max_accum_error = 20/_ki; // TUNE - set a maximum error contribution
            if(accum_error > max_accum_error) {
                accum_error = max_accum_error;
            }
            else if (accum_error <(-max_accum_error)){
                accum_error = -max_accum_error;
            }


            // PI control 

            float correction = (_kp * error) + (_ki * accum_error);

            prev_error = error;

            return(correction);

        }

        else{

            prev_error = error;

            return 0.0;
        }


    }

float ServoController::PDController(float _setpoint, float _measurement, float _kp, float _kd){

    // calculate error r-y
    float error = _setpoint - _measurement;

    //deadband - dont adjust for neglegible titls
    if(abs(error) < tilt_threshold){

        float derivative = (error - prev_error) / dt_sec;

        float correction = (_kp * error) + (_kd * derivative);

        return correction;

    }

    else{
        return 0.0;
    }


}

float ServoController::PIDController(float _setpoint, float _measurement, float _kp, float _ki, float _kd){

    // calculate error = r - y
    float error = _setpoint - _measurement;

    //deadband - dont adjust for neglegible titls
    if(abs(error) < tilt_threshold){

        float derivative = (error - prev_error) / dt_sec;

        accum_error += error * dt_sec;

        // CLEGG INTEGRATOR - reset accumulated error when platform hits level

        if((prev_error * error) < 0){       // when change in error sign (i.e platform has hit level)

                accum_error = 0; 

            }


        float max_accum_error = 20/_ki; // TUNE - set a maximum error contribution
        if(accum_error > max_accum_error) {
                accum_error = max_accum_error;
            }
        else if (accum_error <(-max_accum_error)){
                accum_error = -max_accum_error;
            }


        float correction = (_kp * error) + (_ki * accum_error) + (_kd * derivative);  
        return(correction);      

    }

    else{
        return 0.0;
    }


}
