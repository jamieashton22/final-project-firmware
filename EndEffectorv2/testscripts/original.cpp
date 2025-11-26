
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

// ======   MAIN CODE =====================================================

  // calculate dt for specific loop 
  unsigned long currTime = micros();
  dt = (currTime - prevTime)/ 1000000.0;
  prevTime = currTime; 
  if(dt > 0.05){
    dt = 0.05;          // cap at 50ms -- TWEAK
  }

  // read servo
  // if servo_true = servo_written set servo reached to true, else delay or else break?

  roll_servo_true = readServo(rollServo, ROLL_FBACK);

  Serial.println("roll_servo_true: ");
  Serial.println(roll_servo_true);
  Serial.println("roll write at beginning of loop : ");
  Serial.println(roll_write);


  if(abs(roll_servo_true - roll_write) <= 2){  // if servo has reached commanded position within certain threshold (tweak)
    
      servo_reached = true;
  
   }

  else{
    servo_reached = false; 
  }

  if(servo_reached == true){

    roll_pitch curr_IMU_reading = ReadIMU();

    roll_deg = curr_IMU_reading.roll * 180.0/PI;
    pitch_deg = curr_IMU_reading.pitch * 180.0/PI;

    roll_write = 90 - (roll_deg);     // TWEAK
    roll_write = constrain(roll_write, 0, 180);
    rollServo.write(round(roll_write));               // write the new angle

    servo_reached = false;              // set flag to false

  }

// rollServo.write(90);
// delay(1000);
// rollServo.write(0);
// delay(1000);

// ===222222=============== TEST CODES ============================================

  // // FOR CALIBRATION OF VMIN AND VMAX

  // float v_for_calib = 0.0;
  // v_for_calib = calibrateServo(servo1, 90, FEEDBACK_1);
  // Serial.println("Raw Reading = ");
  // Serial.println(reading_raw_adc);
  // Serial.println("V min = ");
  // Serial.println(v_for_calib);


  // // FOR TESTING READSERVO 
  // servo1.write(90);   // write to 90 to test
  // delay(1000);        // delay -allow to reach
  // int curr_pos = readServo(servo1, FEEDBACK_1);
  // Serial.println("Wrote servo to 90, actual angle = ");
  // Serial.println(curr_pos);

  // servo1.write(120);   // write to 120 to test
  // delay(1000);        // delay -allow to reach
  // curr_pos = readServo(servo1, FEEDBACK_1);
  // Serial.println("Wrote servo to 120, actual angle = ");
  // Serial.println(curr_pos);

  // servo1.write(45);   // write to 90 to test
  // delay(1000);        // delay -allow to reach
  // curr_pos = readServo(servo1, FEEDBACK_1);
  // Serial.println("Wrote servo to 45, actual angle = ");
  // Serial.println(curr_pos);
  


}

float calibrateServo(Servo &_servo, int _pos, int _feedbackpin){

  double v_calib = 0.0;

  _servo.write(_pos); // write servo to position

  delay(1000);        // wait a second to allow servo to reach position

  reading_raw_adc = analogRead(_feedbackpin);
  v_calib = ((analogRead(_feedbackpin)) * 5.0)/1023.0;

  return(v_calib);

}

int readServo(Servo &_servo, int _feedbackpin){   // use of & prevents copies, pass by reference 

  float v_meas = ((analogRead(_feedbackpin))*5.0/1023.0 );
  float normalised_pos = (v_meas - V_MIN)/(V_MAX - V_MIN);
  normalised_pos = constrain(normalised_pos, 0.0, 1.1);

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
