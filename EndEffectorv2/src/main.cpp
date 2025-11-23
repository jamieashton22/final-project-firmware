#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <math.h>
#include <Servo.h>
#include "Controller.h"

#define COMP_ALPHA 0.94
#define ROLL_PIN 3
#define HARD_ROLL_SETPOINT 0

#define KP 0.5
#define KI 0          
#define KD 0
#define SETPOINT 0.0

#define MAX_SLEW_RATE 30.0  // Maximum degrees per second the servo can move

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

// Slew rate variables
float currentServoPosition = 90.0;
unsigned long lastServoUpdate = 0;

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
  currentServoPosition = 90.0;  // Initialize slew position
  delay(2000);  // Give servo time to reach position
  
  prev_roll = initial_roll;
  prev_pitch = initial_pitch;
  
  prevTime = micros();
  lastServoUpdate = millis();  // Initialize servo update time
  
  Serial.println("Setup complete! Starting main loop...\n");
  Serial.print("Slew rate limit: ");
  Serial.print(MAX_SLEW_RATE);
  Serial.println(" degrees/second\n");
}


void loop() {
  // Calculate dt for IMU
  unsigned long currTime = micros();
  dt = (currTime - prevTime) / 1000000.0;
  prevTime = currTime;
  if (dt > 0.05) dt = 0.05;  // Cap at 50ms
  
  // Read IMU
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
  
  // Calculate desired target position from controller
  float targetServoAngle = RollController.PDController(roll_deg, dt);
  
  // ========== SLEW RATE LIMITING ==========
  unsigned long now = millis();
  float dt_servo = (now - lastServoUpdate) / 1000.0;  // Time since last servo update in seconds
  lastServoUpdate = now;
  
  // Calculate maximum allowed change based on slew rate
  float maxChange = MAX_SLEW_RATE * dt_servo;
  
  // Calculate error between target and current position
  float positionError = targetServoAngle - currentServoPosition;
  
  // Apply slew rate limit
  if (abs(positionError) > maxChange) {
    // Move at maximum slew rate in the direction of target
    currentServoPosition += (positionError > 0 ? maxChange : -maxChange);
  } else {
    // Close enough - just go to target
    currentServoPosition = targetServoAngle;
  }
  
  // Constrain to valid servo range
  currentServoPosition = constrain(currentServoPosition, 0.0, 180.0);
  
  // Write the slew-limited position to servo
  RollServo.write((int)currentServoPosition);
  // ========================================
  
  // Debug output
  Serial.print("Roll: "); 
  Serial.print(roll_deg, 2);
  Serial.print(" | Target: "); 
  Serial.print(targetServoAngle, 2);
  Serial.print(" | Actual: "); 
  Serial.print(currentServoPosition, 2);
  Serial.print(" | dt: "); 
  Serial.println(dt, 4);

  delay(10);  // Fast loop for smooth slewing
}


roll_pitch CompFilter(float _ax, float _ay, float _az, float _wx, float _wy, float alpha, float delta_t) {
  
  // Accelerometer angles
  float roll_acc = atan2(_ay, _az);
  float pitch_acc = atan2(-_ax, sqrt(pow(_ay, 2) + pow(_az, 2)));

  // Gyro integration
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