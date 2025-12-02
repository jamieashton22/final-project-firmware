
/* End Effector v3  02/12/25
finally have some functionality 

TODO:
- add core functionality
- test just pitch 
- get working with both axis and film
- write/add PID controller
- add and use improved IMU class
- move servo functionality into seperate class 
- try without the timing (adapt EEv1)


NOTE: BOTH AXIS 
- first, try with one 'servo time passed' flag 
*/

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <math.h>
#include <Servo.h>

#define PITCH_PIN 3
#define ROLL_PIN 2
#define PITCH_FBACK A1
#define ROLL_FBACK A0

#define COMP_ALPHA 0.96     // TUNE 

#define SERVO_RANGE 180.0

#define SERVO_TOLERANCE 3.0 
#define TILT_THRESHOLD 10.0 // TUNE   // don't correct tilts smaller than this 

#define SERVO_UPDATE_RATE 10.0

struct roll_pitch {
  float roll;
  float pitch;
};

float calibrateServo(Servo &_servo, int _pos, int _feedbackpin);
int readServo(Servo &_servo, int _feedbackpin);

roll_pitch CompFilter(float _ax, float _ay, float _az, float _wx, float _wy, float alpha, float delta_t);
roll_pitch ReadIMU();

// FOR CALIBRATION OF VMIN AND VMAX FOR SERVO ANALOGUE FEEDBACK
float v_max = 3.07;
float v_min = 0.22; // temp hardcode

float prev_roll = 0.0;
float prev_pitch = 0.0;

roll_pitch initial_roll_pitch;  // to hold roll pitch offset

unsigned long imuPrevTime = 0.0;
unsigned long servoPrevTime = 0.0;
float dt_imu = 0.05; 

bool roll_servo_reached = false; //flag to store if servo has reached position
bool pitch_servo_reached = false;
bool servo_time_passed = false; //flag to store if servo update time reached 
// bool pitch_servo_time_passed = false;

float roll_deg = 0.0;
float pitch_deg = 0.0;
float roll_write = 90.0;
float pitch_write = 90.0;

int roll_servo_true = 0;  // to hold actual servo value 
int pitch_servo_true = 0;

Adafruit_MPU6050 mpu;
Servo rollServo;
Servo pitchServo;

void setup() {
  rollServo.attach(ROLL_PIN);
  pinMode(ROLL_FBACK, INPUT);
  pitchServo.attach(PITCH_PIN);
  pinMode(PITCH_FBACK, INPUT);
  
  Serial.begin(9600);

  // Wait for serial to be ready
  while(!Serial) delay(10);

  // calibrating servo feedback

  // Serial.println("Calibrating servo feedback");
  // v_max = calibrateServo(rollServo, 180, ROLL_FBACK);
  // delay(100);
  // Serial.println("v max = ");
  // Serial.println(v_max);
  // v_min = calibrateServo(rollServo, 0, ROLL_FBACK);
  // delay(100);
  // Serial.println("v min = ");
  // Serial.println(v_min);
  // delay(100);
  
  // set up IMU 

  Serial.println("Initializing MPU6050...");
  if (!mpu.begin()) {
    Serial.println("MPU6050 not detected. Check wiring!");
    while (1);
  }
  Serial.println("MPU6050 successfully initialized!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  // move servos to center position and wait 
  rollServo.write(90);
  pitchServo.write(90);
  delay(1000);
  Serial.println("Moving servo to centre position, true value = ");
  Serial.println(readServo(rollServo, ROLL_FBACK));

  // calibrate IMU

  Serial.println("Calibrating IMU");
  initial_roll_pitch = ReadIMU();
  delay(1000);
  Serial.println(" Initial roll: ");
  Serial.println((initial_roll_pitch.roll) * 180.0/PI);
  Serial.println("Initial pitch: ");
  Serial.println((initial_roll_pitch.pitch) * 180.0/PI);

  imuPrevTime = micros();

}

void loop() {

  // calculate dt for IMU timing 
  unsigned long currTime = micros();
  dt_imu = (currTime - imuPrevTime) / 1000000.0; // convert to seconds 
  imuPrevTime = currTime; //update prevtime 
  if(dt_imu > 0.05) {

    dt_imu = 0.05;  // cap dt 50 ms

  }


  // calculate dt for servo timing 

  unsigned long currTimeServo = millis();
  if(currTimeServo - servoPrevTime >= SERVO_UPDATE_RATE) { // if sufficient time passed to allow servos to reach position 
    servo_time_passed = true;
  }
  
  else{
    servo_time_passed = false;
  }

  // read servo position
  roll_servo_true = readServo(rollServo, ROLL_FBACK);
  pitch_servo_true = readServo(pitchServo, PITCH_FBACK);

  // check if servos reached position

  if(abs(roll_servo_true - round(roll_write)) <= SERVO_TOLERANCE){

    roll_servo_reached = true;

  }
  else{
    roll_servo_reached = false;
  }

  if(abs(pitch_servo_true - round(pitch_write)) <= SERVO_TOLERANCE){

    pitch_servo_reached = true;

  }
  else{
    pitch_servo_reached = false;
  }


  // only update position when servo has been given sufficient time to move and servo has reached its position
  // for roll servo
  if(roll_servo_reached && servo_time_passed && pitch_servo_reached) {
  // if(servo_time_passed && pitch_servo_reached) {    // DEBUG TEMP FOR TESTING JUST PITCH AXIS

    // reset servos timer 
    servoPrevTime = currTimeServo;

    // safe to read IMU
    roll_pitch curr_imu_reading = ReadIMU();
    roll_deg = curr_imu_reading.roll * 180.0/PI;
    pitch_deg = curr_imu_reading.pitch * 180.0/PI;

    // apply deadband
    if(abs(roll_deg) > 2.0) {

      // incremental control (just P control for now)
      float roll_correction = 0.4 * roll_deg;
      
      roll_write = roll_servo_true - roll_correction;
      roll_write = constrain(roll_write, 30, 150);  // safe limits
      rollServo.write(round(roll_write));

    }

    if(abs(pitch_deg) > 2.0) {

      float pitch_correction = 0.3 * pitch_deg;

      pitch_write = pitch_servo_true + pitch_correction;
      pitch_write = constrain(pitch_write, 30, 150);
      pitchServo.write(round(pitch_write));

    }

  pitch_servo_reached = false; 
  roll_servo_reached = false; // reset flag 

  }

  delay(10);

}

float calibrateServo(Servo &_servo, int _pos, int _feedbackpin){

  double v_calib = 0.0;

  _servo.write(_pos); // write servo to position

  delay(1000);        // wait a second to allow servo to reach position

  v_calib = ((analogRead(_feedbackpin)) * 5.0)/1023.0;

  return(v_calib);

}

int readServo(Servo &_servo, int _feedbackpin){   // use of & prevents copies, pass by reference 

  float v_meas = ((analogRead(_feedbackpin))*5.0/1023.0 );
  float normalised_pos = (v_meas - v_min)/(v_max- v_min);
  normalised_pos = constrain(normalised_pos, 0.0, 1.0);

  int angle_read = int(normalised_pos * SERVO_RANGE);

  return(angle_read);


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
      dt_imu
    );

    return (comp_angles);     // returns radians

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