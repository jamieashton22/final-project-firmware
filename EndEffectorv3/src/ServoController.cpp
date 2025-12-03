#include "ServoController.h"

ServoController::ServoController(int _signal_pin, int _feedback_pin, float _servo_tolerance, float _tilt_threshold, float _update_rate){

    signal_pin = _signal_pin;
    feedback_pin = _feedback_pin;
    servo_tolerance = _servo_tolerance;     // initialise private members
    tilt_threshold = _tilt_threshold;
    update_rate = _update_rate;


}

void ServoController::setupServo(){

    pinMode(feedback_pin, INPUT);

    servo.attach(signal_pin);
    servo.write(90);
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

  int angle_read = int(normalised_pos * servo_range);

  return(angle_read);


}

bool ServoController::servoTimePassed(){

    servoCurrTime = millis();
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

    
    void ServoController::updateServo(float _imu_reading){

        // reset servo timer
        servoPrevTime = servoCurrTime;

        // apply deadband
        if(abs(_imu_reading) > 2.0) {

            // CONTROL GOES HERE
            // just with a hardcoded P for now, change this when controller written
            float correction = 0.4 * _imu_reading;
            servo_write = servo_true + correction;
            servo_write = constrain(servo_write, 30, 150);  // safe limits
            servo.write(round(servo_write));


        }

        servo_reached = false;


    }