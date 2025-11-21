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
    EndEffector.cpp & EndEffector.h - files for end effector (servos and IMU)

    TO DO
    - Test serial connection - DONE
    - Create end-effector class - DONE
    - Add complementary filter - DONE
    - Add PID 
    - Add function to calibrate analogue feedback servos - DONE
    - Add function to get analogue servo values  - DONE 
    - Create receiver  class - DONE
    - Add some servo testing - DONE
    - Create servo arm class - DONE
    - Add function to get dt time - DONE 
    - Convert roll and pitch to degrees - DONE 
    - add function to write analogue servo values -- DONE 

    FUTURE VERSIONS
    - Create Advanced Parser
    - Make sure bidirectional communication working
    - implement servo driver
    - MAYBE put analogue feedback servo calibration in end effector setup function 
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
    #define ROLL_FEEDBACK_PIN A0
    #define PITCH_FEEDBACK_PIN A1

    #define COMP_FILTER_ALPHA 0.94

    #define BASE_PIN 2
    #define SHOULDER_PIN 3
    #define ELBOW_PIN 4

    // -------------------------- GLOBAL --------------------------------------------------

    Receiver Arduino_receiver(9600);    // declares receiver object 

    EndEffector SSPlatform(ROLL_PIN, PITCH_PIN, ROLL_FEEDBACK_PIN, PITCH_FEEDBACK_PIN);    // declares EE object

    RobotArm Arm(BASE_PIN, SHOULDER_PIN, ELBOW_PIN);

    unsigned long prev_time = 0;
    float dt = 0.0;                 // for loop timing

    // to recieve joint angles from python
    int q0 = 90; int q1 = 90; int q2 = 90;  // maybe safe positions?
    char j = ' '; 
    int q = 0;


    // -------------------------- FUNCTION PROTOTYPES --------------------------------------------------

    void UpdateDT();        // TO GET LOOP TIME AND UPDATE DT WITH IT

    // -------------------------- SET-UP ----------------------------------------------------------------

    void setup() {

        Arduino_receiver.StartSerialConnection();   // initate serial connectiom
        SSPlatform.EndEffectorSetup();              // setup end effector 
        Arm.ArmSetup();
        SSPlatform.CalibrateFeedback();             // calibrate analogue feedback servos 

        // onboard LED for testing serial
        pinMode(LED_BUILTIN, OUTPUT);
        digitalWrite(LED_BUILTIN, LOW);

        prev_time = micros();
    }

    // -------------------------- MAIN ----------------------------------------------------------------


    void loop() {

        // update loop-timing
        UpdateDT();
        
        String anglesInput = Arduino_receiver.ReceiveSerialInput();

        if(anglesInput.length() > 0){

            // Parse joint commands
            int t0, t1, t2;
            sscanf(anglesInput.c_str(), "%d,%d,%d", &t0, &t1, &t2);
            q0 = t0; q1 = t1; q2 = t2;
            Arm.UpdatePosition(q0,q1,q2);

            // get IMU reading 
            String currIMUReading = SSPlatform.GetIMUMessage(COMP_FILTER_ALPHA, dt);

            // send IMU reading
            Arduino_receiver.SendSerialOutput(currIMUReading);

            //



        }

        delay(100);

    }

    // -------------------------- FUNCTION DEFINITIONS ----------------------------------------------------------------

    void UpdateDT(){       // update dt 

        unsigned long curr_time = micros();
        dt = (curr_time - prev_time) / 1000000.0;  // updates global variable dt
        if (dt > 0.05) dt = DT_CAP;   // cap dt
        prev_time = curr_time;      // update global prev_time

    }
