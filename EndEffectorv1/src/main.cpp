#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <math.h>
#include <Servo.h>
#include "Controller.h"

#define COMP_ALPHA 0.94  // Reduced from 0.94
#define ROLL_PIN 3
#define HARD_ROLL_SETPOINT 0

#define KP 0.5 // Reduced from 1.0
#define KI 0          
#define KD 0
#define SETPOINT 0.0

#define ROLLFEEDBACKPIN A1

Adafruit_MPU6050 mpu;
Servo RollServo;

float prev_roll = 0.0;
float prev_pitch = 0.0;

float initial_roll = 0.0;
float initial_pitch = 0.0;

float pitch_setpoint = 0.0;
float roll_setpoint = 0.0;

unsigned long prevTime = 0;
float dt = 0.0;

float minV = 0.0;
float maxV = 0.0;

typedef struct roll_pitch {
  float roll;
  float pitch;
};

roll_pitch CompFilter(float _ax, float _ay, float _az, float _wx, float _wy, float alpha, float delta_t);
Controller RollController(KP, KI, KD, SETPOINT);


void setup() {
  pinMode(ROLLFEEDBACKPIN, INPUT);
  Serial.begin(9600);
  
  // Wait for serial to be ready
  while(!Serial) delay(10);
  
  Serial.println("Initializing MPU6050...");
  if (!mpu.begin()) {
    Serial.println("MPU6050 not detected. Check wiring!");
    while (1);
  }
  Serial.println("MPU6050 successfully initialized!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  RollServo.attach(ROLL_PIN);
  
  // Move servo to neutral position
  Serial.println("Moving servo to neutral position (90)...");
  RollServo.write(90);
  delay(2000);  // Give servo time to reach position
  
  prev_roll = initial_roll;
  prev_pitch = initial_pitch;
  
  prevTime = micros();
  
  Serial.println("Setup complete! Starting main loop...\n");
}


 void loop() {

    // Calculate dt for this specific update
    unsigned long currTime = micros();
    dt = (currTime - prevTime) / 1000000.0;
    prevTime = currTime;
    if (dt > 0.05) dt = 0.05;  // Cap at 50ms
    
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    roll_pitch comp_angles = CompFilter(
      a.acceleration.x, 
      a.acceleration.y, 
      a.acceleration.z, 
      g.gyro.x, 
      g.gyro.y, 
      COMP_ALPHA, 
      dt
    );

    float roll_deg = comp_angles.roll * 180.0/PI;
    float newRollAngle = RollController.PDController(roll_deg, dt);
    
    RollServo.write(newRollAngle);
    
    // Debug output
    Serial.print("Roll: "); Serial.print(roll_deg);
    Serial.print(" | Command: "); Serial.print(newRollAngle);
    Serial.print(" | dt: "); Serial.println(dt, 4);

    delay(10);
  }



roll_pitch CompFilter(float _ax, float _ay, float _az, float _wx, float _wy, float alpha, float delta_t) {
  
  // Accelerometer angles
  float roll_acc = atan2(_ay, _az);
  float pitch_acc = atan2(-_ax, sqrt(pow(_ay, 2) + pow(_az, 2)));

  // Gyro integration WITH CALIBRATION OFFSETS APPLIED
  float roll_gyro = prev_roll + (_wx * delta_t);
  float pitch_gyro = prev_pitch + (_wy * delta_t);

  // Complementary filter
  float comp_roll = (alpha * roll_gyro) + ((1 - alpha) * roll_acc);
  float comp_pitch = (alpha * pitch_gyro) + ((1 - alpha) * pitch_acc);

  roll_pitch _roll_pitch;
  _roll_pitch.roll = comp_roll;
  _roll_pitch.pitch = comp_pitch;

  prev_roll = comp_roll;
  prev_pitch = comp_pitch;

  return _roll_pitch;
}