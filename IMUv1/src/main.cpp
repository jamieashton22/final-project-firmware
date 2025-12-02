/* CODE just for the IMU for now to get it working, write a class and test it

FOR TESTING:
1 - Can read raw IMU vals
2 - can read compfiltered IMU vals
3 - tweak and test accel range, gyro range and bandwidth in setup method 
4 - Can get raw offsets before calibration (should be all zero)
5 - Can get raw offsets after calibration 
6 - TEST improved comp filter

NOTE 
// dt needs to be in seconds when passed into the compfilter
// filter output is currently in radians, convert before passing/printing
// look into ACCELEROMETER RECONDITIONING

*/

#include <Arduino.h>
#include "IMU.h"

Adafruit_MPU6050 mpu6050;
IMU imu(mpu6050);

#define COMP_FILTER_ALPHA 0.98

unsigned long imuPrevTime = 0;
float dt_imu = 0.05;


void setup() {

    Serial.begin(9600);
    Serial.println("Starting program");
    imu.SetupIMU();
    imu.GetRawOffsets();

    imuPrevTime = micros();

}



void loop() {

        // calculate dt for IMU timing 
    unsigned long currTime = micros();
    dt_imu = (currTime - imuPrevTime)/ 1000000.0;
    imuPrevTime = currTime; 
    if(dt_imu > 0.05){
        dt_imu = 0.05;          // cap at 50ms -- TWEAK
    }

    // // TEST RAW VALUES

    // raw_imu_vals currval = imu.ReadRawIMU();
    // Serial.println("Raw IMU vals :");
    // Serial.print("Accelerometer: ax: ");
    // Serial.print(currval.ax);
    // Serial.print("ay: ");
    // Serial.print(currval.ay);
    // Serial.print("az: ");
    // Serial.println(currval.az);
    // Serial.print("Gyro: wx: ");
    // Serial.print(currval.wx);
    // Serial.print("wy: ");
    // Serial.println(currval.wy);
    // delay(100);

    // // TEST CALIBRATED IMU VALS

    // raw_imu_vals calcurrval = imu.ReadCalibIMU();
    // Serial.println("Calibrates IMU vals :");
    // Serial.print("Accelerometer: ax: ");
    // Serial.print(calcurrval.ax);
    // Serial.print("ay: ");
    // Serial.print(calcurrval.ay);
    // Serial.print("az: ");
    // Serial.println(calcurrval.az);
    // Serial.print("Gyro: wx: ");
    // Serial.print(calcurrval.wx);
    // Serial.print("wy: ");
    // Serial.println(calcurrval.wy);
    // delay(100);

    // // for plotting 
    // raw_imu_vals calcurrval = imu.ReadRawIMU();
    // Serial.print(calcurrval.ax);
    // Serial.print(" ");
    // Serial.print(calcurrval.ay);
    // Serial.print(" ");
    // Serial.println(calcurrval.az);

    // for data 
    raw_imu_vals calcurrval = imu.ReadCalibIMU();
    Serial.print(calcurrval.ax);
    Serial.print(",");
    Serial.print(calcurrval.ay);
    Serial.print(",");
    Serial.print(calcurrval.az);
    Serial.print(",");
    Serial.print(calcurrval.wx);
    Serial.print(",");
    Serial.println(calcurrval.wy);
    delay(100);

    // Test comp filter

    // raw_imu_vals curr_imu_vals = imu.ReadCalibIMU();
    // roll_pitch  curr_roll_pitch = imu.CompFilter(curr_imu_vals, COMP_FILTER_ALPHA, dt_imu);

    // Serial.print("Roll: ");
    // Serial.print(curr_roll_pitch.roll * 180.0/PI);
    // Serial.print("Pitch: ");
    // Serial.println(curr_roll_pitch.pitch * 180.0/PI);

    // delay(50);


}