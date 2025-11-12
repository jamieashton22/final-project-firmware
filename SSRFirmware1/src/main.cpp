/*

FIRMWARE FOR SELF STABILISING ROBOTIC ARM - VERSION 1  
    CREATED 12/11/25
    JAMIE ASHTON 


 TO DO
 - Test serial connection - DONE
 - Create end-effector class
 - Add complementary filter
 - Create receiver  class files 
 - Create PID class 
*/

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <math.h>
#include <Servo.h>
#include "Receiver.h"


// -------------------------- MACROS --------------------------------------------------


// -------------------------- GLOBAL --------------------------------------------------

Receiver Arduino_receiver(9600);    // declares receiver object 

// -------------------------- FUNCTION DEFINITIONS --------------------------------------------------

// -------------------------- SET-UP ----------------------------------------------------------------

void setup() {

    Arduino_receiver.StartSerialConnection();

    // onboard LED for testing serial
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);

}

void loop() {

    String input_message = Arduino_receiver.ReceiveSerialInput();
    Arduino_receiver.TestSerialConnection(input_message);
    delay(100);

}

