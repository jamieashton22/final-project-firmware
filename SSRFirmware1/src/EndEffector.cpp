#include "EndEffector.h"

EndEffector::EndEffector(int _roll_pin, int _pitch_pin, int _roll_feedback_pin, int _pitch_feedback_pin){

    roll_pin = _roll_pin;
    pitch_pin = _pitch_pin;
    prev_roll = 0.0;
    prev_pitch = 0.0;
    roll_feedback_pin = _roll_feedback_pin;
    pitch_feedback_pin = _pitch_feedback_pin;

}

void EndEffector::EndEffectorSetup(){       // setup function

    // setup servos

    roll_servo.attach(roll_pin);
    pitch_servo.attach(pitch_pin);

    // setup IMU

    Serial.println("Initializing MPU6050...");
    if (!imu.begin()) {
        Serial.println("MPU6050 not detected. Check wiring!");
        while (1);  // Halt execution if sensor is not found
    }
    Serial.println("MPU6050 successfully initialized!");

    imu.setAccelerometerRange(MPU6050_RANGE_8_G); // set accelerometer sensitivity
    imu.setGyroRange(MPU6050_RANGE_500_DEG);      // set gyro sensitivity
    imu.setFilterBandwidth(MPU6050_BAND_21_HZ); // apply a low pass filter

    // initialise first pitch and roll readings (approximation)
    sensors_event_t a, g, temp;        
    imu.getEvent(&a, &g, &temp);        
    prev_roll = atan2(a.acceleration.y, a.acceleration.z);
    prev_pitch = atan2(-a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z));


}   


roll_pitch EndEffector::CompFilter(IMU_values _imu_reading, float _alpha, float _delta_t){

       //first convert ax and ay to angles
  // roll is about x axis, pitch is about y
  float roll_acc = (atan2(_imu_reading.acceleration_y,_imu_reading.acceleration_z));
  float pitch_acc = atan2(-1*(_imu_reading.acceleration_x), sqrt(pow(_imu_reading.acceleration_y,2) + pow(_imu_reading.acceleration_z,2)));

  // need to store previous angle x and previous angle y and previous angle z
  float roll_gyro = prev_roll + (_imu_reading.velocity_x * _delta_t);
  float pitch_gyro = prev_pitch + (_imu_reading.velocity_y * _delta_t);

  float comp_roll = (_alpha * roll_gyro) + ((1-_alpha)*roll_acc);
  float comp_pitch = (_alpha * pitch_gyro) + ((1-_alpha)* pitch_acc);

  roll_pitch _roll_pitch;
  _roll_pitch.roll = comp_roll * (180/PI);
  _roll_pitch.pitch = comp_pitch * (180/PI);

  prev_roll = comp_roll;
  prev_pitch = comp_pitch;   // update the old roll & pitch with filtered values 

  return _roll_pitch;
 

}


IMU_values EndEffector::getRawIMU(){

    sensors_event_t a, g, temp;
    imu.getEvent(&a, &g, &temp);    // recieve data for accelertion, angular velocity

    IMU_values IMU_reading;
    IMU_reading.acceleration_x = a.acceleration.x;
    IMU_reading.acceleration_y = a.acceleration.y;
    IMU_reading.acceleration_z = a.acceleration.z;
    IMU_reading.velocity_x = g.gyro.x;
    IMU_reading.velocity_y = g.gyro.y;
    IMU_reading.velocity_z = g.gyro.z;

    return(IMU_reading);

}

String EndEffector::GetIMUMessage(float _alpha,float _dt){

    //get raw 
    IMU_values raw_IMU_reading = getRawIMU();
        // filter it
    roll_pitch fused_IMU_reading = CompFilter(raw_IMU_reading,_alpha, _dt);
    String imuMessage = String(fused_IMU_reading.roll, 3) + "," + String(fused_IMU_reading.pitch, 3);   // convert to string for sending

    return(imuMessage);


}

void EndEffector::CalibrateFeedback(){

    // obtain max values
    roll_servo.write(angle_max);
    pitch_servo.write(angle_max);
    delay(100);
    roll_v_max = analogRead(roll_feedback_pin);
    pitch_v_max = analogRead(pitch_feedback_pin);
    

    // obtain min values

    roll_servo.write(angle_min);
    pitch_servo.write(angle_min);
    delay(100);
    roll_v_min = analogRead(roll_feedback_pin);
    pitch_v_min = analogRead(pitch_feedback_pin);


}

servo_positions EndEffector::ReadServos(){

    servo_positions current_positions;
    float roll_voltage = analogRead(roll_feedback_pin);
    float pitch_voltage = analogRead(pitch_feedback_pin);

    // normalise the read voltage and convert to an angle
    current_positions.roll_position = ((roll_voltage - roll_v_min)/(roll_v_max - roll_v_min))*(angle_max - angle_min);  
    current_positions.pitch_position = (pitch_voltage - pitch_v_min)/(pitch_v_max - pitch_v_min)*(angle_max-angle_min);

    return(current_positions);

}

void EndEffector::writeEEServos(int _roll_write, int _pitch_write){

    roll_servo.write(_roll_write);
    pitch_servo.write(_pitch_write);

    // maybe delay 

}

