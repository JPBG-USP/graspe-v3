#ifndef _PID_CONTROLLER_
#define _PID_CONTROLLER_

/// @brief PID Controller class
class PIDcontroller
{
private:
    float _Kp, _Ki, _Kd, _dt;

    // States
    float e;       // current error (unused but kept for structure)
    float e1;      // previous error
    float integral; // accumulated error

public:
    PIDcontroller(float Kp, float Kd, float Ki, float dt);
    void updateGains(float Kp, float Kd, float Ki);
    float action(float error);
    void reset();
};

#endif