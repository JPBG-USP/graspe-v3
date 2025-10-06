#include <PIDcontroller.h>
#include <Arduino.h>

PIDcontroller::PIDcontroller(float Kp, float Kd, float Ki, float dt)
    : _Kp(Kp), _Kd(Kd), _Ki(Ki), _dt(dt), e(0), e1(0), e2(0), last_action(0)
{
    computeCoefficients();
}

/// @brief Update gains in running time
/// @param Kp Proporcional gain
/// @param Kd Derivative gain
/// @param Ki Integrate gain
void PIDcontroller::updateGains(float Kp, float Kd, float Ki) {
    _Kp = Kp;
    _Kd = Kd;
    _Ki = Ki;
    computeCoefficients();
}

/// @brief Returns the control action
/// @param error System error
/// @return Action
float PIDcontroller::action(float error) {
    // shift errors
    e2 = e1;
    e1 = e;
    e = error;

    // compute new action (incremental form)
    float u = last_action + q0 * e + q1 * e1 + q2 * e2;
    u = constrain(u, -1, 1);

    // update last action
    last_action = u;

    return u;
}