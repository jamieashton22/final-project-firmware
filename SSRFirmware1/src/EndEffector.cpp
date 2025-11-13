#include "EndEffector.h"

EndEffector::EndEffector(int _roll_pin, int _pitch_pin){

    roll_pin = _roll_pin;
    pitch_pin = _pitch_pin;
    prev_roll = 0.0;
    prev_pitch = 0.0;

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

// roll_pitch EndEffector::CompFilter(float _ax,float _ay, float _az, float _wx, float _wy, float alpha, float delta_t){   // return sensor fusion values


//    //first convert ax and ay to angles
//   // roll is about x axis, pitch is about y
//   float roll_acc = (atan2(_ay,_az));
//   float pitch_acc = atan2(-1*_ax, sqrt(pow(_ay,2) + pow(_az,2)));

//   // need to store previous angle x and previous angle y and previous angle z
//   float roll_gyro = prev_roll + (_wx * delta_t);
//   float pitch_gyro = prev_pitch + (_wy * delta_t);

//   float comp_roll = (alpha * roll_gyro) + ((1-alpha)*roll_acc);
//   float comp_pitch = (alpha * pitch_gyro) + ((1-alpha)* pitch_acc);

//   roll_pitch _roll_pitch;
//   _roll_pitch.roll = comp_roll;
//   _roll_pitch.pitch = comp_pitch;

//   prev_roll = comp_roll;
//   prev_pitch = comp_pitch;   // update the old roll & pitch with filtered values 

//   return _roll_pitch;


// }

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
  _roll_pitch.roll = comp_roll;
  _roll_pitch.pitch = comp_pitch;

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