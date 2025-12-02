
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


  THIS ISNT WORKING - 
  - calibrate servos in setup - done
  - calibrate IMU in setup


  BOOM - STARTING TO GET THERE 

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

#define COMP_ALPHA 0.96

#define SERVO_RANGE 180.0

#define ROLLSETPOINT 80
#define PITCHSETPOINT 80

#define SERVO_TOLERANCE 5.0
#define STABILITY_THRESHOLD 3.0;  // ignore small tilts

#define SERVO_UPDATE_RATE 100 // 100 ms between control updates

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
float v_max = 3.07;
float v_min = 0.22; // temp hardcode

float prev_roll = 0.0;
float prev_pitch = 0.0;

roll_pitch initial_roll_pitch;

float initial_roll = 0.0;
float initial_pitch = 0.0;

unsigned long imuPrevTime = 0.0;
unsigned long servoPrevTime = 0.0;
float dt_imu = 0.05;

bool servo_reached = false;   // flag whether servos reached target 
bool servo_time_passed = false;

float roll_deg = 0.0;
float pitch_deg = 0.0;
float roll_write = 90.0;
int pitch_write = 0;

int roll_servo_true = 0;

Adafruit_MPU6050 mpu;
Servo rollServo;
Servo pitchServo;

void setup(){

  rollServo.attach(ROLL_PIN);
  pinMode(ROLL_FBACK, INPUT);
  
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

// // ======   MAIN CODE =====================================================

  // calculate dt for IMU timing 
  unsigned long currTime = micros();
  dt_imu = (currTime - imuPrevTime)/ 1000000.0;
  imuPrevTime = currTime; 
  if(dt_imu > 0.05){
    dt_imu = 0.05;          // cap at 50ms -- TWEAK
  }

  // roll_pitch curr_imu_reading = ReadIMU();

  // calculate dt for servo timing

  unsigned long currTimeServo = millis();
  if(currTimeServo - servoPrevTime >= SERVO_UPDATE_RATE){

    servoPrevTime = currTimeServo;
    servo_time_passed = true;

  }
  else { servo_time_passed = false;}

  roll_servo_true = readServo(rollServo, ROLL_FBACK);
  Serial.println(" roll_servo_true : ");
  Serial.println(roll_servo_true);

  Serial.println("roll_write is ");
  Serial.println(roll_write);
  
  if(abs(roll_servo_true - round(roll_write)) <= SERVO_TOLERANCE){  // if servo has reached commanded position within certain threshold (tweak)
    
    servo_reached = true;
  
  }

  else{
    servo_reached = false; 
  }

  // only update if servos have reached, and enough time passed

  if(servo_reached == true && servo_time_passed == true){ // MAIN IF STATEMENT

    roll_pitch curr_imu_reading = ReadIMU();
    roll_deg = curr_imu_reading.roll * 180.0/PI;
    pitch_deg = curr_imu_reading.pitch * 180.0/PI;

    if(abs(roll_deg) > 3.0){ // deadband

      // write new angle 
      roll_write = 90.0 - (roll_deg);     // TWEAK
      roll_write = constrain(roll_write, 0, 180);
      rollServo.write(round(roll_write)); 
      
    }


    servoPrevTime = currTimeServo;

    // for debug: 
   
    Serial.print("Corrected - Roll: "); Serial.print(roll_deg);
    Serial.print("° → Cmd: "); Serial.print(roll_write);
    Serial.print(" | Pitch: "); Serial.print(pitch_deg);
    Serial.print("° → Cmd: "); Serial.println(pitch_write);

  }

  delay(10);

// // ===222222=============== TEST CODES ============================================

//   // // FOR CALIBRATION OF VMIN AND VMAX

//   // float v_for_calib = 0.0;
//   // v_for_calib = calibrateServo(servo1, 90, FEEDBACK_1);
//   // Serial.println("Raw Reading = ");
//   // Serial.println(reading_raw_adc);
//   // Serial.println("V min = ");
//   // Serial.println(v_for_calib);


  // // FOR TESTING READSERVO 
  // rollServo.write(90);   // write to 90 to test
  // delay(1000);        // delay -allow to reach
  // int curr_pos = readServo(rollServo, ROLL_FBACK);
  // Serial.println("Wrote servo to 90, actual angle = ");
  // Serial.println(curr_pos);

  // rollServo.write(120);   // write to 120 to test
  // delay(1000);        // delay -allow to reach
  // curr_pos = readServo(rollServo, ROLL_FBACK);
  // Serial.println("Wrote servo to 120, actual angle = ");
  // Serial.println(curr_pos);

  // rollServo.write(45);   // write to 90 to test
  // delay(1000);        // delay -allow to reach
  // curr_pos = readServo(rollServo, ROLL_FBACK);
  // Serial.println("Wrote servo to 45, actual angle = ");
  // Serial.println(curr_pos);


  // // FOR TESTING IMU
  // roll_pitch current_imu_reading = ReadIMU();
  // delay(100);
  // Serial.println(" loop roll: ");
  // Serial.println((current_imu_reading.roll) * 180.0/PI);
  // Serial.println("loop pitch: ");
  // Serial.println((current_imu_reading.pitch) * 180.0/PI);
  


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
