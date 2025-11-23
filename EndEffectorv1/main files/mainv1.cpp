
/* 

  // FRI 21 NOV THIS NEEDS SOME REAL REWORK - ESPECIALLY CONCERNING SWEEPING ETC
    e.g reading voltage before doing a sweep - servo takes time to reach its specified value 
        2/3 logic is completely wrong ( most likely)
        just go to bed and try again tomorrow ....

  Code for just end-effector on its own - to be integrated into overall system when working

  NOTE: 0-270 maps to 0-180 with servo.write so multiply by 2/3
        rpm too high, somewhat unstable on servo, use for loops with delays when writing 

  TODO: 
        fix calibration - determine pitch and roll setpoints from calibration
        adjust everything for the 2/3 situation - honestly no clue with this one
        add delayed movement - DONE (SORT OF)
        Modify P controller - DONE
        Modify PI controller - DONE
        Write PID controller - 
        Add safe startup position
        Move end-effector function to end-effector class
        Read servo values function - BUT TO DO THAT we need a servo calibration function on startup???


  so sequence goes
  setup: initialise MPU, attach servo pins, init prev_roll and prev_pitch, init time w micros()
  loop: calculate loop timing, read and filter IMU, put that into PID to get new angles, write new angles to servo w delays (incremental/sweep) 


  TESTING W HARDWARE:
  make sure IMU giving pitch and roll correctly
  start with one servo 
  then incorporate other 

  NEED TO GO OVER ALL THE 270 -> 180 LOGIC WHEN SOBER 
        
*/

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <math.h>
#include <Servo.h>
#include "Controller.h"

#define COMP_ALPHA 0.94
#define PITCH_PIN 2
#define ROLL_PIN 3
#define HARD_PITCH_SETPOINT -3 // temporary hard coded setpoint
#define HARD_ROLL_SETPOINT -3

#define KP 1
#define KI 0          // initially no controller
#define KD 0
#define SETPOINT 0

#define PITCHFEEDBACKPIN A0
#define ROLLFEEDBACKPIN A1

Adafruit_MPU6050 mpu;
Servo PitchServo;
Servo RollServo;

float prev_roll = 0.0;
float prev_pitch = 0.0;

float calibration_x_sum = 0.0;
float calibration_y_sum = 0.0;
float calibration_z_sum = 0.0;

float initial_roll = 0.0;
float initial_pitch = 0.0;

float pitch_setpoint = 0.0;
float roll_setpoint = 0.0;

unsigned long prevTime = 0;   // to get loop timing 
float dt = 0.0;

// FOR READING SERVOS

float minV = 0.0;
float maxV = 0.0;

typedef struct roll_pitch {   // struct for comp filter return type
  float roll;
  float pitch;
};

roll_pitch CompFilter(float _ax,float _ay, float _az, float _wx, float _wy, float alpha, float delta_t);

Controller PitchController(KP, KI, KD, SETPOINT);
Controller RollController(KP, KI, KD, SETPOINT);

float ReadServo(byte _analogPin);
void CalibrateServo(Servo _servo, byte _analogPin);


void setup() {

  pinMode(PITCHFEEDBACKPIN, INPUT);
  pinMode(ROLLFEEDBACKPIN, INPUT);  // declare the voltage out pins as inputs

  Serial.begin(9600);

      Serial.println("Initializing MPU6050...");
    if (!mpu.begin()) {
        Serial.println("MPU6050 not detected. Check wiring!");
        while (1);  // Halt execution if sensor is not found
    }
    Serial.println("MPU6050 successfully initialized!");

    mpu.setAccelerometerRange(MPU6050_RANGE_8_G); // set accelerometer sensitivity
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);      // set gyro sensitivity
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ); // apply a low pass filter

    PitchServo.attach(PITCH_PIN);
    RollServo.attach(ROLL_PIN);

    prev_roll = initial_roll;     // hardcode for now
    prev_pitch = initial_pitch;

    CalibrateServo(PitchServo, PITCHFEEDBACKPIN);
    CalibrateServo(RollServo, ROLLFEEDBACKPIN);


    prevTime = micros();  // initialise prev time to not be zero

}

void loop() {

// // ======= MAIN CODE =========

  unsigned long currTime = micros();          // current time in µs
  dt = (currTime - prevTime) / 1000000.0;     // convert to seconds
  prevTime = currTime;
  if (dt > 0.1) dt = 0.1;   // cap initial dt         // TWEAK

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);    // reciever data for accelertion, angular velocity & temp

  roll_pitch comp_angles = CompFilter(a.acceleration.x,a.acceleration.y, a.acceleration.z, g.gyro.x, g.gyro.y, COMP_ALPHA, dt);

  float newPitchAngle = (2.0/3.0) * PitchController.PController(comp_angles.pitch * 180.0/PI);    // CHECK THE 2/3 LOGIC HERE 
  float newRollAngle = (2.0/3.0) * RollController.PController(comp_angles.roll *180/PI);        // convert to deg 

  float curr_pitch_position = ReadServo(PITCHFEEDBACKPIN);
  float curr_roll_position = ReadServo(ROLLFEEDBACKPIN);

  // PitchServo.write(newPitchAngle);      // if no delay
  // RollServo.write(newRollAngle);

  //if delays

  // writing for pitch w delay

  if(curr_pitch_position < newPitchAngle){

    for(int i = int(curr_pitch_position); i < newPitchAngle; i++){
      PitchServo.write(i);
      delay(10);
    }

  }

  if(curr_pitch_position > newPitchAngle){

    for(int i = int(curr_pitch_position); i > newPitchAngle; i--){
      PitchServo.write(i);
      delay(10);
    }

  }

  // writing for roll w delay 

  if(curr_roll_position < newRollAngle){

    for(int i = int(curr_roll_position); i < newRollAngle; i++){
      RollServo.write(i);
      delay(10);
    }

  }

  if(curr_roll_position > newRollAngle){

    for(int i = int(curr_roll_position); i > newRollAngle; i--){
      RollServo.write(i);
      delay(10);
    }

  }
  

  delay(10);    // TWEAK


}

roll_pitch CompFilter(float _ax,float _ay, float _az, float _wx, float _wy, float alpha, float delta_t){

  //first convert ax and ay to angles
  // roll is about x axis, pitch is about y
  float roll_acc = (atan2(_ay,_az));
  float pitch_acc = atan2(-1*_ax, sqrt(pow(_ay,2) + pow(_az,2)));

  // need to store previous angle x and previous angle y and previous angle z
  float roll_gyro = prev_roll + (_wx * delta_t);
  float pitch_gyro = prev_pitch + (_wy * delta_t);

  float comp_roll = (alpha * roll_gyro) + ((1-alpha)*roll_acc);
  float comp_pitch = (alpha * pitch_gyro) + ((1-alpha)* pitch_acc);

  roll_pitch _roll_pitch;
  _roll_pitch.roll = comp_roll;
  _roll_pitch.pitch = comp_pitch;

  prev_roll = comp_roll;
  prev_pitch = comp_pitch;   // update the old roll & pitch with filtered values 

  return _roll_pitch;

}

void CalibrateServo(Servo _servo, byte _analogPin){

  _servo.write(0);
  minV = analogRead(_analogPin);

  for(int i = 0; i < 180; i++){
    _servo.write(i);
    delay(10);
  }

  maxV = analogRead(_analogPin);

}

float ReadServo(byte _analogPin){

  float curr_voltage = analogRead(_analogPin);

  float position = (curr_voltage-minV)/(maxV - minV);
  
  return(position * 180);

}
