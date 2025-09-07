#ifndef _PID_CONTROLLER_
#define _PID_CONTROLLER_

/// @brief PID Controller class
class PIDcontroller
{
private:
    // Gains
    float _Kp;
    float _Kd;
    float _Ki;
    float _dt;

    // Calculated Gains
    float q0;
    float q1;
    float q2;

    // Error history
    float e;
    float e1;
    float e2;

    // Control action
    float last_action;

    void computeCoefficients() {
        q0 = _Kp + _Ki * _dt + (_Kd/_dt);
        q1 = -_Kp - 2*(_Kd/_dt);
        q2 = _Kd/_dt;
    }

public:
    PIDcontroller(float Kp, float Kd, float Ki, float dt);
    void updateGains(float Kp, float Kd, float Ki);
    float action(float error);
};

#endif