
/* 

  EndEffectorv2 will have a function to calibrate the servo for analogue feedback and a function to read the actual servos position,
      this can then be implemented with the rest to find out if the cascaded control works

  To Do: reusable calibration function to find servo v min and vmax
        get values of Vmin and Vmax
        function to read servo values

    Calibration routine: first simple to give a single voltage reading


  Calibration Results:
  for servo 1: Vmin = 0.22V, Vmax = 3.03V, V90 = 1.63V
  for servo 2: Vmin = 0.23V, Vmax = 3.03V, V90 = 1.63V
  identical so can use same Vmin and Vmax

*/


#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <math.h>
#include <Servo.h>

#define PITCH_PIN 3
#define ROLL_PIN 2
#define PITCH_FBACK A0
#define ROLL_FBACK A1

#define COMP_ALPHA 0.96

#define V_MAX 3.03
#define V_MIN 0.22
#define SERVO_RANGE 180.0

#define ROLLSETPOINT 80
#define PITCHSETPOINT 80

// ===================== PARAMETERS =========================
#define SERVO_TOLERANCE_DEG 20   // servo must be within ±10°
#define SERVO_STABLE_TIME  80       // ms the servo must remain stable
#define SERVO_UPDATE_RATE  50       // ms between servo writes (~20Hz)
#define SERVO_DEADBAND     2        // minimum difference to update servo
// ==========================================================

unsigned long lastServoUpdate = 0;
unsigned long stableStart = 0;
bool servoStable = false;
struct roll_pitch {
  float roll;
  float pitch;
};

float calibrateServo(Servo &_servo, int _pos, int _feedbackpin);
int readServo(Servo &_servo, int _feedbackpin);

roll_pitch CompFilter(float _ax, float _ay, float _az, float _wx, float _wy, float alpha, float delta_t);
roll_pitch ReadIMU();

// FOR CALIBRATION OF VMIN AND VMAX
float reading_raw_adc = 0.0; // temp for debug

float prev_roll = 0.0;
float prev_pitch = 0.0;

float initial_roll = 0.0;
float initial_pitch = 0.0;

unsigned long prevTime = 0;
float dt = 0.0;

bool servo_reached = false;   // flag whether servos reached target 

float roll_deg = 0.0;
float pitch_deg = 0.0;
int roll_write = 0;
int pitch_write = 0;

int roll_servo_true = 0;

Adafruit_MPU6050 mpu;
Servo rollServo;
Servo pitchServo;

// ===================== FILTERED ADC READING ===============
int readServoFiltered(int pin) {
  long sum = 0;
  for (int i = 0; i < 10; i++) {
    sum += analogRead(pin);
    delayMicroseconds(300);
  }
  return sum / 10;
}

// ===================== MAP FEEDBACK → ANGLE ===============
int readServoAngle(int feedbackPin) {
  int raw = readServoFiltered(feedbackPin);
  float v_meas = raw * 5.0 / 1023.0;

  float norm = (v_meas - V_MIN) / (V_MAX - V_MIN);
  norm = constrain(norm, 0.0, 1.0);

  return int(norm * SERVO_RANGE);
}

// ===================== SERVO REACHED DETECTOR =============
bool servoHasReached(int feedbackAngle, int commandedAngle) {
  int error = abs(feedbackAngle - commandedAngle);

  if (error <= SERVO_TOLERANCE_DEG) {
    if (!servoStable) {
      servoStable = true;
      stableStart = millis();
    }
    if (millis() - stableStart > SERVO_STABLE_TIME) {
      return true;
    }
  } else {
    servoStable = false;
  }
  
  return false;
}

// ===================== RATE-LIMITED SERVO UPDATER =========
void writeServoStabilized(Servo& s, int& lastWrite, int newWrite) {

  if (abs(newWrite - lastWrite) < SERVO_DEADBAND)
    return;

  if (millis() - lastServoUpdate < SERVO_UPDATE_RATE)
    return;

  newWrite = constrain(newWrite, 0, 180);

  s.write(newWrite);
  lastWrite = newWrite;

  lastServoUpdate = millis();
}

void setup(){

  rollServo.attach(ROLL_PIN);
  pinMode(ROLL_FBACK, INPUT);
  
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

  roll_servo_true = 90;
  rollServo.write(90);
  roll_write = 90;

  prevTime = micros();


}


void loop() {

  // TIMING
  unsigned long currTime = micros();
  dt = (currTime - prevTime) / 1000000.0;
  prevTime = currTime;
  if (dt > 0.05) dt = 0.05; // cap


  // 1. READ SERVO FEEDBACK
  int rollActual = readServoAngle(ROLL_FBACK);

  Serial.print("Servo true: ");
  Serial.print(rollActual);
  Serial.print("   Last write = ");
  Serial.println(roll_write);


  // 2. CHECK IF SERVO HAS REACHED POSITION
  bool reached = servoHasReached(rollActual, roll_write);


  // 3. ONLY WHEN REACHED → read IMU & compute correction
  if (reached) {

    roll_pitch angles = ReadIMU();   // your IMU function

    float roll_deg  = angles.roll  * 180.0 / PI;

    int newRollWrite = 90 - roll_deg;    // your logic

    // 4. WRITE SERVO SAFELY (rate limited, deadband)
    writeServoStabilized(rollServo, roll_write, newRollWrite);
  }
}


roll_pitch ReadIMU(){

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

    return (comp_angles);     // returns radians

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
