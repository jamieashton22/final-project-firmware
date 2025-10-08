

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <math.h>
#include <Servo.h>

#define COMP_ALPHA 0.94
#define PITCH_PIN 9

Adafruit_MPU6050 mpu;
Servo PitchServo;

float prev_roll = 0.0;
float prev_pitch = 0.0;

float initial_roll = 0.0;
float initial_pitch = 0.0;


unsigned long prevTime = 0;   // to get loop timing 
float dt = 0.0;

typedef struct roll_pitch {   // struct for comp filter return type
  float roll;
  float pitch;
};

roll_pitch CompFilter(float _ax,float _ay, float _az, float _wx, float _wy, float alpha, float delta_t);

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

    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);    // reciever data for accelertion, angular velocity & temp

    initial_roll = (atan2(a.acceleration.y,a.acceleration.z));
    initial_pitch = atan2(-1*a.acceleration.x, sqrt(pow(a.acceleration.y,2) + pow(a.acceleration.z,2)));  // use accelerometer to get rough intial position
    
    prevTime = micros();  // initialise prev time to not be zero
}

void loop() {

// ======= MAIN CODE =========

//   unsigned long currTime = micros();          // current time in µs
//   dt = (currTime - prevTime) / 1000000.0;     // convert to seconds
//   prevTime = currTime;
//   if (dt > 0.05) dt = 0.05;   // cap initial dt
  
//   sensors_event_t a, g, temp;
//   mpu.getEvent(&a, &g, &temp);    // reciever data for accelertion, angular velocity & temp

//   roll_pitch comp_angles = CompFilter(a.acceleration.x,a.acceleration.y, a.acceleration.z, g.gyro.x, g.gyro.y, COMP_ALPHA, dt);

// Serial.print(comp_angles.roll * 180.0 / PI);
// Serial.print(", ");
// Serial.println(comp_angles.pitch * 180.0 / PI);

// delay(100);

// ======= SERVO TESTING CODE =========

// PitchServo.write(0); //min

PitchServo.write(90); // max

// PitchServo.write(0);
// delay(3000);

// for(int i = 0; i<180; i++){
//   PitchServo.write(i);        // test positive movement
// }

// delay(10000);

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
