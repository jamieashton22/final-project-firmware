#ifndef CONTROLLER_H
#define CONTROLLER_H

class Controller {

    private:
        float kp;
        float ki;
        float kd;
        float dt;           // pass dt from loop timing?
        float alpha;        // alpha term possibly not needed

    public:
        Controller(float _kp, float _ki, float _kd, float _dt, float _alpha);
        float PController(float _measured);     // input measured value return corrected
        float PIController(float _measured);    
        float PIDController(float _measured);


};





#endif 