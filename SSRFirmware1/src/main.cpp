/*

FIRMWARE FOR SELF STABILISING ROBOTIC ARM - VERSION 1  
    CREATED 12/11/25
    JAMIE ASHTON 


 TO DO
 - Create end-effector class
 - Add complementary filter
 - Create receiver function
 - Create PID class 
*/

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <math.h>
#include <Servo.h>


// -------------------------- MACROS --------------------------------------------------


// -------------------------- GLOBAL --------------------------------------------------


// -------------------------- FUNCTION DEFINITIONS --------------------------------------------------

String ReceiveSerialInput();   // to recieve input message from python script
void SendSerialOutput(String output_message);      // to output a message to python
void TestSerialConnection(String input_message);                // function to test the serial connection


// -------------------------- SET-UP ----------------------------------------------------------------

void setup() {

    // start serial
    Serial.begin(9600);

    // onboard LED for testing serial
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);

}

void loop() {

    String input_message = ReceiveSerialInput();
    TestSerialConnection(input_message);
    delay(100);

}

// -------------------------- SERIAL COMMUNICATIONS FUNCTIONS  ----------------------------------------------------------------

String ReceiveSerialInput(){

    String serialInput = "";
    if (Serial.available() > 0) {

        serialInput = Serial.readStringUntil('\n');
        serialInput.trim();

    }

    return(serialInput);
}

void SendSerialOutput(String output_message){

    Serial.print(output_message);

}

void TestSerialConnection(String input_message){

    int x = -1;

    sscanf(input_message.c_str(), "%d", &x);
    
    if(x == 1){

        digitalWrite(LED_BUILTIN, HIGH);

    }
    
    else{

        digitalWrite(LED_BUILTIN, LOW);

    }

}