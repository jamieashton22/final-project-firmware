/* End Effector v3  02/12/25
finally have some functionality 

TODO:
- add core functionality        -- DONE
- test just pitch   -- working 
- get working with both axis and film   -- DONE 
- write/add PID controller
- add and use improved IMU class - DONE
- move servo functionality into seperate class -- DONE 
- try without the timing (adapt EEv1)


NOTE: BOTH AXIS
- first, try with one 'servo time passed' flag and one big if statement
- second, try with 2 servo time passed and 2 if statements - yeah that works 
*/

#include <Arduino.h>
#include "IMU.h"
#include "ServoController.h"

#define ROLL_SETPOINT 0
#define PITCH_SETPOINT 0

#define PITCH_KP 0.4
#define PITCH_KI 0
#define PITCH_KD 0
#define PITCH_ALPHA 0.7

#define ROLL_KP 0.4
#define ROLL_KI 0
#define ROLL_KD 0
#define ROLL_ALPHA 0.7

// roll servo controller parameters
#define ROLL_SIGNAL_PIN 2
#define ROLL_FEEDBACK_PIN A0
#define ROLL_SERVO_TOLERANCE 3.0
#define ROLL_TILT_THRESHOLD 5.0
#define ROLL_UPDATE_RATE 20.0

// pitch servo controller parameters
#define PITCH_SIGNAL_PIN 3
#define PITCH_FEEDBACK_PIN A1
#define PITCH_SERVO_TOLERANCE 3.0
#define PITCH_TILT_THRESHOLD 5.0
#define PITCH_UPDATE_RATE 20.0

// IMU parameters
#define COMP_FILTER_ALPHA 0.96

unsigned long imu_prev_time = 0.0;
float dt_imu = 0.05;

// objects
Adafruit_MPU6050 mpu;
ServoController RollServo(ROLL_SIGNAL_PIN, ROLL_FEEDBACK_PIN, ROLL_SERVO_TOLERANCE, ROLL_TILT_THRESHOLD, ROLL_UPDATE_RATE);
ServoController PitchServo(PITCH_SIGNAL_PIN, PITCH_FEEDBACK_PIN, PITCH_SERVO_TOLERANCE, PITCH_TILT_THRESHOLD, PITCH_UPDATE_RATE);
IMU imu(mpu);

void setup() {

    Serial.begin(9600);

    imu.SetupIMU();
    Serial.println("Setting up servos");
    RollServo.setupServo();
    PitchServo.setupServo();
    // Serial.println("8 second delay");
    // delay(8000);
    imu_prev_time = micros();
}

void loop() {

    // calculate dt for IMU timing 
    unsigned long curr_time = micros();
    dt_imu = (curr_time - imu_prev_time) / 1000000.0; // convert to seconds 
    imu_prev_time = curr_time; //update prevtime 
    if(dt_imu > 0.05) {

        dt_imu = 0.05;  // cap dt 50 ms

    }

    // calculate dt for each servo

    bool roll_time_flag = RollServo.servoTimePassed(); // returns bool
    bool pitch_time_flag = PitchServo.servoTimePassed(); //returns bool


    // read servo position

    int true_roll_pos = RollServo.readServo();
    int true_pitch_pos = PitchServo.readServo();

    // check servos reached position 

    bool roll_pos_flag = RollServo.servoReachedPosition(true_roll_pos);
    bool pitch_pos_flag = PitchServo.servoReachedPosition(true_pitch_pos);

    // Roll Update
    
    if(roll_time_flag && roll_pos_flag) {

        // read imu and comp filter
        raw_imu_vals imu_reading_raw = imu.ReadCalibIMU();
        roll_pitch imu_reading_filt = imu.CompFilter(imu_reading_raw, COMP_FILTER_ALPHA, dt_imu);

        RollServo.updateServo(-(imu_reading_filt.roll) * 180.0/PI);


    }

    // Pitch Update

    if(pitch_time_flag && pitch_pos_flag) {

        // read imu and comp filter
        raw_imu_vals imu_reading_raw = imu.ReadCalibIMU();
        roll_pitch imu_reading_filt = imu.CompFilter(imu_reading_raw, COMP_FILTER_ALPHA, dt_imu);

        // pass imu reading into controller

        // for P controller
        float correction = PitchServo.PController(PITCH_SETPOINT,-(imu_reading_filt.pitch) * 180.0/PI, PITCH_KP);   // feed negative measurement into pitch controller as inverted

        // for PI controller
        // float correction = PitchServo.PIController(PITCH_SETPOINT, -(imu_reading_filt.pitch) * 180.0/PI, PITCH_KP, PITCH_KI); // feed negative measurement into pitch controller as inverted

        // for PD controller
        // float correction = PitchServo.PDController(PITCH_SETPOINT, -(imu_reading_filt.pitch) * 180.0/PI, PITCH_KP, PITCH_KD, PITCH_ALPHA); // feed negative measurement into pitch controller as inverted

        // for PID controller
        // float correction = PitchServo.PIDController(PITCH_SETPOINT, -(imu_reading_filt.pitch) * 180.0/PI, PITCH_KP,PITCH_KI, PITCH_KD); // feed negative measurement into pitch controller as inverted

        // pass controller correction into updateServo method
        PitchServo.updateServo(correction);


    }

    delay(10);

}