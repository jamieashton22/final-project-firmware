/*

FIRMWARE FOR SELF STABILISING ROBOTIC ARM - VERSION 1  
    CREATED 12/11/25
    JAMIE ASHTON 

VERSION 1 - VERIFY BASIC SERIAL CONNECTION, AND THAT SERVOS ARE WORKING
- CONTAINS SUPER SIMPLE RECEIVER WITH NO COMMUNICATIONS PROTOCOL IMPLEMENTED
- CONTAINS SIMPLE END-EFFECTOR CLASS WITH COMPLEMENTARY FILTER AND PID 
- ALL WITHOUT USING SERVO DRIVER - JUST STANDARD SERVO LIBRARY 

FILES & INFO:
Receiver.h & Receiver.cpp - files for the arduino serial receiver

TO DO
 - Test serial connection - DONE
 - Create end-effector class - DONE
 - Add complementary filter - DONE
 - Add PID 
 - Add function to get analogue servo values 
 - Create receiver  class - DONE
 - Add main servo control
 - Add function to get dt time - DONE 

 FUTURE VERSIONS
 - Create Advanced Parser
*/

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <math.h>
#include <Servo.h>
#include "Receiver.h"
#include "EndEffector.h"
#include "RobotArm.h"


// -------------------------- MACROS --------------------------------------------------

#define DT_CAP 0.02

#define ROLL_PIN 9
#define PITCH_PIN 10

#define COMP_FILTER_ALPHA 0.94

#define BASE_PIN 11
#define SHOULDER_PIN 12
#define ELBOW_PIN 13

// -------------------------- GLOBAL --------------------------------------------------

Receiver Arduino_receiver(9600);    // declares receiver object 

EndEffector SSPlatform(ROLL_PIN, PITCH_PIN);    // declares EE object

RobotArm Arm(BASE_PIN, SHOULDER_PIN, ELBOW_PIN);

unsigned long prev_time = 0;
float dt = 0.0;                 // for loop timing


// -------------------------- FUNCTION PROTOTYPES --------------------------------------------------

void UpdateDT();        // TO GET LOOP TIME AND UPDATE DT WITH IT

// -------------------------- SET-UP ----------------------------------------------------------------

void setup() {

    Arduino_receiver.StartSerialConnection();   // initate serial connectiom
    SSPlatform.EndEffectorSetup();              // setup end effector 

    // onboard LED for testing serial
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);

    prev_time = micros();
}

// -------------------------- MAIN ----------------------------------------------------------------


void loop() {

    // update loop-timing
    UpdateDT();
    Serial.println(dt);

    // get IMU reading
    IMU_values raw_IMU_reading = SSPlatform.getRawIMU();
    // filter it
    roll_pitch fused_IMU_reading = SSPlatform.CompFilter(raw_IMU_reading, COMP_FILTER_ALPHA, dt);

    // recieve input message from python
    int q0 = 90; int q1 = 90; int q2 = 90;  // maybe safe positions?
    char j = ' '; 
    int q = 0;
    String anglesInput = Arduino_receiver.ReceiveSerialInput();

    // // write Multiple joint angles  // TEMP -- FOR DEBUGGING
    // sscanf(anglesInput.c_str(), "%d,%d,%d", &q0, &q1, &q2);
    // Arm.UpdatePosition(q0,q1,q2);

    // // write single joint angle     // TEMP -- FOR DEBUGGING
    // sscanf(anglesInput.c_str(), "%c,%d", &j, &q);
    // Arm.UpdateSinglePosition(j,q);

    sscanf(anglesInput.c_str(), "%c,%d,%d,%d,%d", &j, &q, &q0, &q1, &q2);
    Arm.UpdatePositionGeneral(j,q,q0,q1,q2);

    delay(100);

}


// -------------------------- FUNCTION DEFINITIONS ----------------------------------------------------------------

void UpdateDT(){       // update dt 

    unsigned long curr_time = micros();
    dt = (curr_time - prev_time) / 1000000.0;  // updates global variable dt
    if (dt > 0.05) dt = DT_CAP;   // cap dt
    prev_time = curr_time;      // update global prev_time

}
