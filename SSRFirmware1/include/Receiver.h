
// HEADER FILE FOR RECEIVER CLASS

#ifndef RECEIVER_H
#define RECEIVER_H

#include <Arduino.h>

class Receiver{

    private:

        int baud_rate;

    public:

        Receiver(int _baud_rate);
        void StartSerialConnection();
        String ReceiveSerialInput();   // to recieve input message from python script
        void SendSerialOutput(String _output_message);      // to output a message to python
        void TestSerialConnection(String _input_message);   

};


#endif