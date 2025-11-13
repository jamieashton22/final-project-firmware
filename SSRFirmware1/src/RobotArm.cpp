#include "RobotArm.h"

RobotArm::RobotArm(int _base_pin, int _shoulder_pin, int _elbow_pin){

    base_pin = _base_pin;
    shoulder_pin = _shoulder_pin;
    elbow_pin = _elbow_pin;
}

void RobotArm::ArmSetup(){

    // setup servos
    base.attach(base_pin);
    shoulder.attach(shoulder_pin);
    elbow.attach(elbow_pin);

    base.write(90);
    shoulder.write(90);     // SAFE STARTING POSITION
    elbow.write(90);


}

void RobotArm::UpdatePosition(int _q0, int _q1, int _q2){

    base.write(_q0);
    shoulder.write(_q1);
    elbow.write(_q2);

}

void RobotArm::UpdateSinglePosition(char _j, int _q){        // input from front end e.g "0,90"
                                                            // would be write base 90 
    switch (_j)
    {
    case 'b':
        base.write(_q);
        break;

    case 's':
        shoulder.write(_q);
        break;
    
    case 'e':
        elbow.write(_q);
        break;
    
    default:
        break;
    }

}

void RobotArm::UpdatePositionGeneral(char _j, int _q, int _q0, int _q1, int _q2){

    switch (_j)
    {
    case 'a':

        base.write(_q0);
        shoulder.write(_q1);
        elbow.write(_q2);
        
        break;
    
    case 'b':
        
        base.write(_q);

        break;

    case 's':

        shoulder.write(_q);

        break;

    case 'e':

        elbow.write(_q);

        break;

    default:
        break;
    }


}

