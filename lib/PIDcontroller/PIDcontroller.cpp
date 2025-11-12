#include <PIDcontroller.h>
#include <Arduino.h>

PIDcontroller::PIDcontroller(float Kp, float Kd, float Ki, float dt)
    : _Kp(Kp), _Kd(Kd), _Ki(Ki), _dt(dt),
      e(0.0f), e1(0.0f), integral(0.0f)
{
}

/// @brief Update gains in running time
/// @param Kp Proportional gain
/// @param Kd Derivative gain
/// @param Ki Integral gain
void PIDcontroller::updateGains(float Kp, float Kd, float Ki)
{
    _Kp = Kp;
    _Kd = Kd;
    _Ki = Ki;
}

/// @brief Returns the control action (positional PID form)
/// @param error System error
/// @return Control action
float PIDcontroller::action(float error)
{
    if (0.05 > abs(error))
    {
        return 0.0f;
    }
    
    // Compute derivative and integral
    float derivative = (error - e1) / _dt;
    integral += error * _dt;

    // Compute PID output
    float u = _Kp * error + _Ki * integral + _Kd * derivative;

    // Constrain output
    u = constrain(u, -1.0f, 1.0f);

    // Save previous error
    e1 = error;

    return u;
}

/// @brief Reset all internal states
void PIDcontroller::reset()
{
    e = 0.0f;
    e1 = 0.0f;
    integral = 0.0f;
}
