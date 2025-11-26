#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <math.h>
#include <Servo.h>

#define PITCH_PIN 2
#define ROLL_PIN 3
#define PITCH_FBACK A0
#define ROLL_FBACK A1

#define COMP_ALPHA 0.96

#define V_MAX 3.03
#define V_MIN 0.22
#define SERVO_RANGE 180.0

// Platform control parameters
#define PLATFORM_SETPOINT 0.0     // Target angle: 0° (level)
#define PLATFORM_KP 0.8           // Proportional gain for platform control

// Servo settling parameters
#define SERVO_SETTLED_THRESHOLD 3.0   // Degrees - within this = settled
#define SERVO_SETTLED_TIME_MS 100     // Must be settled for this long

// Loop timing
#define SERVO_CHECK_MS 20         // Check servo position every 20ms
#define PLATFORM_UPDATE_MS 50     // Update platform control every 50ms

struct roll_pitch {
  float roll;
  float pitch;
};

// Function declarations
float calibrateServo(Servo &_servo, int _pos, int _feedbackpin);
float readServo(int _feedbackpin);  // Changed: returns float, no servo ref needed
roll_pitch CompFilter(float _ax, float _ay, float _az, float _wx, float _wy, float alpha, float delta_t);
roll_pitch ReadIMU();
bool isServoSettled(float current_pos, float target_pos);

// Global variables
float prev_roll = 0.0;
float prev_pitch = 0.0;
unsigned long prevTime = 0;
float dt = 0.0;

// Servo state
float targetServoPosition = 90.0;      // Commanded position
float currentServoPosition = 90.0;     // Actual position from feedback
float filteredServoPosition = 90.0;    // Filtered position

// Settling detection
unsigned long servoSettledStartTime = 0;
bool servoIsSettled = false;

// Timing
unsigned long lastServoCheck = 0;
unsigned long lastPlatformUpdate = 0;

// Gyro calibration offsets
float gyro_x_offset = 0.0;
float gyro_y_offset = 0.0;

Adafruit_MPU6050 mpu;
Servo rollServo;

void setup(){
  rollServo.attach(ROLL_PIN);
  pinMode(ROLL_FBACK, INPUT);
  
  Serial.begin(9600);
  while(!Serial) delay(10);
  
  Serial.println("\n========================================");
  Serial.println("Cascaded Control Self-Stabilizer");
  Serial.println("========================================\n");
  
  // Initialize MPU6050
  Serial.println("Initializing MPU6050...");
  if (!mpu.begin()) {
    Serial.println("ERROR: MPU6050 not detected!");
    while (1);
  }
  Serial.println("✓ MPU6050 initialized");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  // Calibrate gyro
  Serial.println("\nCalibrating gyro - keep still!");
  delay(2000);
  
  const int samples = 200;
  float gx_sum = 0, gy_sum = 0;
  
  for(int i = 0; i < samples; i++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    gx_sum += g.gyro.x;
    gy_sum += g.gyro.y;
    delay(5);
  }
  
  gyro_x_offset = gx_sum / samples;
  gyro_y_offset = gy_sum / samples;
  
  Serial.print("✓ Gyro calibrated (offsets: ");
  Serial.print(gyro_x_offset, 4);
  Serial.print(", ");
  Serial.print(gyro_y_offset, 4);
  Serial.println(")");

  // Move servo to center and initialize positions
  Serial.println("\nMoving to center position...");
  rollServo.write(90);
  delay(2000);
  
  currentServoPosition = readServo(ROLL_FBACK);
  filteredServoPosition = currentServoPosition;
  targetServoPosition = currentServoPosition;
  
  Serial.print("✓ Servo at: ");
  Serial.print(currentServoPosition, 1);
  Serial.println("°");

  // Initialize IMU filter
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  float initial_roll = atan2(a.acceleration.y, a.acceleration.z);
  prev_roll = initial_roll;
  
  prevTime = micros();
  lastServoCheck = millis();
  lastPlatformUpdate = millis();
  
  Serial.println("\n✓ Setup complete - Starting control loop\n");
  delay(1000);
}

void loop() {
  unsigned long currentMillis = millis();
  
  // ===== SERVO POSITION MONITORING (50Hz) =====
  if (currentMillis - lastServoCheck >= SERVO_CHECK_MS) {
    lastServoCheck = currentMillis;
    
    // Read actual servo position
    float rawPosition = readServo(ROLL_FBACK);
    
    // Apply low-pass filter to reduce noise
    filteredServoPosition = 0.7 * filteredServoPosition + 0.3 * rawPosition;
    currentServoPosition = filteredServoPosition;
    
    // Check if servo has settled
    servoIsSettled = isServoSettled(currentServoPosition, targetServoPosition);
    
    // Debug output
    Serial.print("Servo - Target: ");
    Serial.print(targetServoPosition, 1);
    Serial.print("° | Actual: ");
    Serial.print(currentServoPosition, 1);
    Serial.print("° | Error: ");
    Serial.print(targetServoPosition - currentServoPosition, 1);
    Serial.print("° | Settled: ");
    Serial.println(servoIsSettled ? "YES" : "NO");
  }
  
  // ===== PLATFORM CONTROL UPDATE (20Hz) =====
  // Only update when servo has reached position and settled
  if (currentMillis - lastPlatformUpdate >= PLATFORM_UPDATE_MS && servoIsSettled) {
    lastPlatformUpdate = currentMillis;
    
    // Calculate dt for complementary filter
    unsigned long currTime = micros();
    dt = (currTime - prevTime) / 1000000.0;
    prevTime = currTime;
    if(dt > 0.1) dt = 0.1;
    
    // Read IMU (safe - servo is stationary!)
    roll_pitch imu_reading = ReadIMU();
    float roll_deg = imu_reading.roll * 180.0 / PI;
    
    // Calculate platform error
    float platform_error = PLATFORM_SETPOINT - roll_deg;
    
    // Simple proportional control for now
    float control_output = PLATFORM_KP * platform_error;
    
    // Calculate new servo target
    // 90° is center, positive error = tilt right = move servo left
    targetServoPosition = 90.0 - control_output;
    
    // Safety limits
    targetServoPosition = constrain(targetServoPosition, 45.0, 135.0);
    
    // Write to servo
    rollServo.write((int)round(targetServoPosition));
    
    // Mark as not settled (target changed)
    servoIsSettled = false;
    servoSettledStartTime = 0;
    
    // Debug output
    Serial.print("\n>>> PLATFORM UPDATE - Roll: ");
    Serial.print(roll_deg, 2);
    Serial.print("° | Error: ");
    Serial.print(platform_error, 2);
    Serial.print("° | New Target: ");
    Serial.print(targetServoPosition, 1);
    Serial.println("°\n");
  }
  
  // Small delay to prevent CPU hammering
  delay(5);
}

// ===== SERVO FEEDBACK READING =====
float readServo(int _feedbackpin) {
  // Read voltage
  float v_meas = (analogRead(_feedbackpin) * 5.0) / 1023.0;
  
  // Normalize to 0.0 - 1.0
  float normalised_pos = (v_meas - V_MIN) / (V_MAX - V_MIN);
  normalised_pos = constrain(normalised_pos, 0.0, 1.0);  // FIXED: 0 to 1, not 0 to 180
  
  // Convert to angle
  float angle_read = normalised_pos * SERVO_RANGE;
  
  return angle_read;
}

// ===== SETTLING DETECTION =====
bool isServoSettled(float current_pos, float target_pos) {
  float error = abs(target_pos - current_pos);
  
  if (error < SERVO_SETTLED_THRESHOLD) {
    // Within threshold
    if (servoSettledStartTime == 0) {
      // Just entered threshold - start timer
      servoSettledStartTime = millis();
      return false;
    } else if (millis() - servoSettledStartTime >= SERVO_SETTLED_TIME_MS) {
      // Been in threshold long enough
      return true;
    } else {
      // In threshold but not long enough
      return false;
    }
  } else {
    // Outside threshold - reset timer
    servoSettledStartTime = 0;
    return false;
  }
}

// ===== IMU READING =====
roll_pitch ReadIMU() {
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

  return comp_angles;
}

// ===== COMPLEMENTARY FILTER =====
roll_pitch CompFilter(float _ax, float _ay, float _az, float _wx, float _wy, float alpha, float delta_t) {
  
  // Accelerometer angles
  float roll_acc = atan2(_ay, _az);
  float pitch_acc = atan2(-_ax, sqrt(pow(_ay, 2) + pow(_az, 2)));

  // Gyro integration with calibration
  float roll_gyro = prev_roll + ((_wx - gyro_x_offset) * delta_t);
  float pitch_gyro = prev_pitch + ((_wy - gyro_y_offset) * delta_t);

  // Complementary filter
  float comp_roll = (alpha * roll_gyro) + ((1 - alpha) * roll_acc);
  float comp_pitch = (alpha * pitch_gyro) + ((1 - alpha) * pitch_acc);

  roll_pitch result;
  result.roll = comp_roll;
  result.pitch = comp_pitch;

  prev_roll = comp_roll;
  prev_pitch = comp_pitch;

  return result;
}

// ===== CALIBRATION FUNCTION =====
float calibrateServo(Servo &_servo, int _pos, int _feedbackpin) {
  _servo.write(_pos);
  delay(1000);
  
  float v_calib = (analogRead(_feedbackpin) * 5.0) / 1023.0;
  return v_calib;
}