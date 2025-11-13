#include "Receiver.h"

Receiver::Receiver(int _baud_rate){

    baud_rate = _baud_rate;

}

void Receiver::StartSerialConnection(){     // to begin serial connection

    Serial.begin(baud_rate);
}

String Receiver::ReceiveSerialInput(){  // to receive message from front end

    String serialInput = "";
    if (Serial.available() > 0) {

        serialInput = Serial.readStringUntil('\n');
        serialInput.trim();

    }

    return(serialInput);

}

void Receiver::SendSerialOutput(String _output_message){    // to send message to front end

    Serial.println(_output_message);

}

void Receiver::TestSerialConnection(String _input_message){ // to test serial connection working

    int x = -1;

    sscanf(_input_message.c_str(), "%d", &x);
    
    if(x == 1){

        digitalWrite(LED_BUILTIN, HIGH);
        SendSerialOutput("1");

    }
    
    else{

        digitalWrite(LED_BUILTIN, LOW);
        SendSerialOutput("0");

    }

}