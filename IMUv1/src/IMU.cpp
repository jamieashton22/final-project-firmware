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
