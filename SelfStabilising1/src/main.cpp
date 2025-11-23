

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <math.h>
#include <Servo.h>
#include "Controller.h"

#define COMP_ALPHA 0.94
#define PITCH_PIN 10
#define ROLL_PIN 9
#define HARD_PITCH_SETPOINT -3 // temporary hard coded setpoint
#define HARD_ROLL_SETPOINT                

#define KP 1
#define KI 0
#define KD 0
#define SETPOINT 0 

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

typedef struct roll_pitch {   // struct for comp filter return type
  float roll;
  float pitch;
};

roll_pitch CompFilter(float _ax,float _ay, float _az, float _wx, float _wy, float alpha, float delta_t);

Controller PitchController(KP, KI, KD, SETPOINT);
Controller RollController(KP, KI, KD, SETPOINT);

void CalibrateIMU();

void setup() {

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

    CalibrateIMU();                 // this is currently not working, setpoint is always 0 
    prev_roll = initial_roll;       // use hardcoded setpoint for now
    prev_pitch = initial_pitch;

    pitch_setpoint = initial_pitch * PI/180;
    roll_setpoint = initial_roll * PI/180;

    prevTime = micros();  // initialise prev time to not be zero

}

void loop() {

// // ======= MAIN CODE =========

  unsigned long currTime = micros();          // current time in µs
  dt = (currTime - prevTime) / 1000000.0;     // convert to seconds
  prevTime = currTime;
  if (dt > 0.05) dt = 0.05;   // cap initial dt
  
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);    // reciever data for accelertion, angular velocity & temp

  roll_pitch comp_angles = CompFilter(a.acceleration.x,a.acceleration.y, a.acceleration.z, g.gyro.x, g.gyro.y, COMP_ALPHA, dt);

  // Serial.print(comp_angles.roll * 180.0 / PI);
  // Serial.print(", ");
  // Serial.println(comp_angles.pitch * 180.0 / PI);

  float newPitchAngle = PitchController.PController(comp_angles.pitch * 180.0/PI);
  float newRollAngle = RollController.PController(comp_angles.roll *180/PI);
  PitchServo.write(newPitchAngle);
  RollServo.write(newRollAngle);

// delay(10);

// // ======= SERVO TESTING CODE =========

// PitchServo.write(0); //min
// delay(1000);
// PitchServo.write(90); //min
// delay(1000);
// PitchServo.write(180); //min
// delay(5000);



// PitchServo.write(90); // max

// // PitchServo.write(0);
// // delay(3000);

// for(int i = 0; i<180; i++){
//   PitchServo.write(i);        // test positive movement
// }

// delay(10000);

// // ======= SETPOINT TESTING CODE =========

  // Serial.print("Pitch setpoint: ");
  // Serial.print(pitch_setpoint);
  // Serial.print(", ROll setpoint: ");
  // Serial.println(roll_setpoint);
  // delay(1000);


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

// need function that gives us our initial values or just run the function once at setpoint 

void CalibrateIMU(){

    delay(3000);   //delay to let sensors warm up

    calibration_x_sum = 0;
    calibration_y_sum = 0;
    calibration_z_sum = 0;

    for(int i = 0; i< 50; i++){

      sensors_event_t a, g, temp;
      mpu.getEvent(&a, &g, &temp);

      calibration_x_sum += a.acceleration.x;
      calibration_y_sum += a.acceleration.y;
      calibration_z_sum += a.acceleration.z;      // 50 samples

      delay(10);

    }

    float a_x_avg = calibration_x_sum/50;
    float a_y_avg = calibration_y_sum/50;
    float a_z_avg = calibration_z_sum/50;

    initial_roll = (atan2(a_y_avg,a_z_avg));
    initial_pitch = atan2(-1*a_x_avg, sqrt(pow(a_y_avg,2) + pow(a_z_avg,2)));  // use accelerometer to get rough intial position

    
}