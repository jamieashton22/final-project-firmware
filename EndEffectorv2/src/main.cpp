
/* 

  EndEffectorv2 will have a function to calibrate the servo for analogue feedback and a function to read the actual servos position,
      this can then be implemented with the rest to find out if the cascaded control works

  To Do: reusable calibration function to find servo v min and vmax
        get values of Vmin and Vmax
        function to read servo values

    Calibration routine: first simple to give a single voltage reading


  Calibration Results:
  for servo 1: Vmin = 0.22V, Vmax = 3.03V, V90 = 1.63V

*/


#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <math.h>
#include <Servo.h>

#define SERVO_1_PIN 2
#define FEEDBACK_1 A0

// FOR CALIBRATION OF VMIN AND VMAX
float reading_raw_adc = 0.0; // temp for debug

#define V_MAX 3.03
#define V_MIN 0.22
#define SERVO_RANGE 180.0


Servo servo1;

float calibrateServo(Servo &_servo, int _pos, int _feedbackpin);
int readServo(Servo &_servo, int _feedbackpin);


void setup(){

  servo1.attach(SERVO_1_PIN);

  pinMode(FEEDBACK_1, INPUT);

  Serial.begin(9600);


}


void loop() {


  // // FOR CALIBRATION OF VMIN AND VMAX

  // float v_for_calib = 0.0;
  // v_for_calib = calibrateServo(servo1, 90, FEEDBACK_1);
  // Serial.println("Raw Reading = ");
  // Serial.println(reading_raw_adc);
  // Serial.println("V min = ");
  // Serial.println(v_for_calib);


  // FOR TESTING READSERVO 
  servo1.write(90);   // write to 90 to test
  delay(1000);        // delay -allow to reach
  int curr_pos = readServo(servo1, FEEDBACK_1);
  Serial.println("Wrote servo to 90, actual angle = ");
  Serial.println(curr_pos);

  servo1.write(120);   // write to 120 to test
  delay(1000);        // delay -allow to reach
  curr_pos = readServo(servo1, FEEDBACK_1);
  Serial.println("Wrote servo to 120, actual angle = ");
  Serial.println(curr_pos);

  servo1.write(45);   // write to 90 to test
  delay(1000);        // delay -allow to reach
  curr_pos = readServo(servo1, FEEDBACK_1);
  Serial.println("Wrote servo to 45, actual angle = ");
  Serial.println(curr_pos);
  


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
  normalised_pos = constrain(normalised_pos, 0.0, 180.0);

  int angle_read = int(normalised_pos * SERVO_RANGE);

  return(angle_read);


}
