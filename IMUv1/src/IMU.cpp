#include "IMU.h"


IMU::IMU(Adafruit_MPU6050 _mpu){     

    mpu = _mpu;

}

void IMU::SetupIMU(){

    Serial.println("Initializing MPU6050...");
    if (!mpu.begin()) {
        Serial.println("MPU6050 not detected. Check wiring!");
        while (1);
    }
    Serial.println("MPU6050 successfully initialized!");

    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);     // check this and test 

    // zero offsets 
    SetRawOffsets(0.0,0.0,0.0,0.0,0.0);

    Serial.println(" 8 second delay to let IMU warm up");
    delay(8000);

    // calibrate IMU
    CalibRawOffsets();
    raw_imu_vals initial_vals = ReadCalibIMU();
    
    prev_roll = atan2(initial_vals.ay, initial_vals.az);
    prev_pitch = atan2(- initial_vals.ax, sqrt((initial_vals.ay * initial_vals.ay) + (initial_vals.az * initial_vals.az)));

    
}

// change the raw data offsets 
void IMU::SetRawOffsets(float _ax,float _ay,float _az,float _wx,float _wy) {

    raw_offsets.ax = _ax;
    raw_offsets.ay = _ay;
    raw_offsets.az = _az;
    raw_offsets.wy = _wy;
    raw_offsets.wx = _wx;

}

// get and print the raw offsets 

void IMU::GetRawOffsets(){

    Serial.print("Accel offsets: ");
    Serial.print(raw_offsets.ax); Serial.print(", ");
    Serial.print(raw_offsets.ay); Serial.print(", ");
    Serial.println(raw_offsets.az);

    Serial.print("Gyro offsets: ");
    Serial.print(raw_offsets.wx); Serial.print(", ");
    Serial.println(raw_offsets.wy);

}

// read raw IMU vals
raw_imu_vals IMU::ReadRawIMU(){

    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    raw_imu_vals rawVals = {
        a.acceleration.x,
        a.acceleration.y,
        a.acceleration.z,
        g.gyro.x,
        g.gyro.y
    };

    return(rawVals);

}


// read the IMU and subtract the calibration offsets
raw_imu_vals IMU::ReadCalibIMU(){

    raw_imu_vals calib_imu_vals = ReadRawIMU();

    calib_imu_vals.ax = calib_imu_vals.ax - raw_offsets.ax;
    calib_imu_vals.ay = calib_imu_vals.ay - raw_offsets.ay;
    calib_imu_vals.az = calib_imu_vals.az - raw_offsets.az;
    calib_imu_vals.wx = calib_imu_vals.wx - raw_offsets.wx;
    calib_imu_vals.wy = calib_imu_vals.wy - raw_offsets.wy;         // subtract all offsets 

    return(calib_imu_vals);


}

// calibrate the raw IMU offsets 
void IMU::CalibRawOffsets() {

    // some setup 
    Serial.println(" Calibrating raw IMU offsets - keep level and stable ");

    // take 1000 samples 

    const int total_samples = 1000;
    const int used_samples = 900;

    float sum_ax = 0.0, sum_ay = 0.0, sum_az = 0.0; // hold sums
    float sum_wx = 0.0, sum_wy = 0.0;

    // want to discard first 100 samples
    for(int i = 0; i < total_samples; i++) {


        if (i > 100){

            raw_imu_vals raw_vals = ReadRawIMU();

            sum_ax += raw_vals.ax;
            sum_ay += raw_vals.ay;
            sum_az += raw_vals.az;
            sum_wx += raw_vals.wx;
            sum_wy += raw_vals.wy;


        }

        delay(5); // small delay avoid flooding 

    }

    // call set offsets, pass them all in

    SetRawOffsets((sum_ax/used_samples),(sum_ay/used_samples),((sum_az/used_samples) - 9.81 ),(sum_wx/used_samples),(sum_wy/used_samples));

    Serial.println("Raw Calibration Complete");

}


roll_pitch IMU::CompFilter(raw_imu_vals _raw_calib_imu_vals, float _alpha, float _delta_t){

    // accelerometer angles
    float roll_acc = atan2(_raw_calib_imu_vals.ay, _raw_calib_imu_vals.az);
    // float pitch_acc = atan2(-_raw_calib_imu_vals.ax, sqrt(pow(_raw_calib_imu_vals.ay,2) + pow(_raw_calib_imu_vals.az,2)));
    float pitch_acc = atan2(-_raw_calib_imu_vals.ax, sqrt((_raw_calib_imu_vals.ay * _raw_calib_imu_vals.ay ) + (_raw_calib_imu_vals.az * _raw_calib_imu_vals.az)));


    // gyro integration
    float roll_gyro = prev_roll + (_raw_calib_imu_vals.wx * _delta_t);
    float pitch_gyro = prev_pitch + (_raw_calib_imu_vals.wy * _delta_t);

    // comp filter
    float comp_roll = (_alpha * roll_gyro) + ((1 - _alpha) * roll_acc);
    float comp_pitch = (_alpha * pitch_gyro) + ((1 - _alpha) * pitch_acc);

    roll_pitch _roll_pitch = {

        comp_roll,
        comp_pitch

    };

    prev_roll = comp_roll;
    prev_pitch = comp_pitch;

    return(_roll_pitch);
}


    
// comp filter with accelerometer reconditioning

roll_pitch IMU::CompFilterv2(raw_imu_vals _v, float _alpha, float _delta_t){

    // get accel based angles
    float roll_acc = atan2(_v.ay, _v.az);
    float pitch_acc = atan2(-_v.ax, sqrt(_v.ay*_v.ay + _v.az * _v.az));

    // gyro integration
    float roll_gyro = prev_roll + _v.wx * _delta_t;
    float pitch_gyro = prev_pitch + _v.wy * _delta_t;

    // accel reconditioning 

    // get accel magnitude
    float acc_mag_raw = sqrt(_v.ax * _v.ax + _v.ay*_v.ay + _v.az*_v.az);

    // smooth using EMA
    static float acc_mag_filt = 9.81f; // start near gravity g 
    acc_mag_filt = 0.1f * acc_mag_raw + 0.9f * acc_mag_filt;

    // thresholds -- TWEAK
    const float ENTER_LO = 8.3f;   // must be VERY close to 1g to re-enter
    const float ENTER_HI = 10.6f;
    const float EXIT_LO  = 7.8f;   // more tolerant window to exit
    const float EXIT_HI  = 11.5f;

    // gyro angular rate threshold
    const float GYRO_THR = 0.04f; 

    // accel valid flaf
    static bool accel_valid = true; 
    
    if(accel_valid) {

        // stop trusting accel if ...
        if (acc_mag_filt <EXIT_LO || acc_mag_filt > EXIT_HI || fabs(_v.wx) > GYRO_THR || fabs(_v.wy) > GYRO_THR){


            accel_valid = false;
        }
        }

    else {  

            // start trusting accel if 

            if (acc_mag_filt > ENTER_LO && acc_mag_filt < ENTER_HI && fabs(_v.wx) < GYRO_THR && fabs(_v.wy) < GYRO_THR){


                accel_valid = true;
            }

        }

        // comp filter now 
        float comp_roll, comp_pitch;

        if (accel_valid) {
        // normal complementary filter
        comp_roll  = _alpha * roll_gyro  + (1.0f - _alpha) * roll_acc;
        comp_pitch = _alpha * pitch_gyro + (1.0f - _alpha) * pitch_acc;
        } else {
            // only gyro while accel unreliable
            comp_roll  = roll_gyro;
            comp_pitch = pitch_gyro;
        }


    prev_roll = comp_roll; //for next iteration
    prev_pitch = comp_pitch;

    roll_pitch output = { comp_roll, comp_pitch};
    return output;

}
