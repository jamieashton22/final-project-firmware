#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <math.h>
#include <Servo.h>
#include "Controller.h"

#define COMP_ALPHA 0.94
#define ROLL_PIN 3
#define PITCH_PIN 2

#define ROLLSETPOINT 90
#define PITCHSETPOINT 90

Adafruit_MPU6050 mpu;
Servo RollServo;
Servo PitchServo;

float prev_roll = 0.0;
float prev_pitch = 0.0;

float initial_roll = 0.0;
float initial_pitch = 0.0;

unsigned long prevTime = 0;
float dt = 0.0;

typedef struct roll_pitch {
  float roll;
  float pitch;
};

roll_pitch CompFilter(float _ax, float _ay, float _az, float _wx, float _wy, float alpha, float delta_t);

void setup() {

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
  PitchServo.attach(PITCH_PIN);

  prevTime = micros();
  
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
    float currentServoReading = RollServo.read();
    float newRollAngle = currentServoReading + (2.0 * roll_deg);

    float pitch_deg = comp_angles.pitch * 180.0/PI;
    float newPitchAngle = PITCHSETPOINT + (pitch_deg);

    Serial.print("roll deg: ");
    Serial.print(roll_deg);
    Serial.print(" pitch deg: ");
    Serial.print(pitch_deg);
    Serial.print(" new roll angle: ");
    Serial.println((constrain(round(newRollAngle), 60, 120)));
    

    RollServo.write(constrain(round(newRollAngle), 60, 120));
    PitchServo.write(round(newPitchAngle));

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