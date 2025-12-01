/* CODE just for the IMU for now to get it working, write a class and test it

FOR TESTING:
1 - Can read raw IMU vals
2 - can read compfiltered IMU vals
3 - tweak and test accel range, gyro range and bandwidth in setup method 
4 - Can get raw offsets before calibration (should be all zero)
5 - Can get raw offsets after calibration 

NOTE 
// dt needs to be in seconds when passed into the compfilter
// filter output is currently in radians, convert before passing/printing

*/

#define COMP_FILTER_ALPHA 0.96