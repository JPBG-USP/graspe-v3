#ifndef _MOTOR_ENCODER_H_
#define _MOTOR_ENCODER_H_

#include <Arduino.h>

class MotorEncoder
{
private:

    // Potentiometer config
    uint8_t _potPin;
    uint16_t _pos_init;
    uint16_t _pos_end;

    // Angle configs
    float _angle_init;
    float _angle_end;
    float _angle_difference;

    // Kalman filter variables
    float _std; 
    float _r; 
    float _x; 
    float _p;
    float _q;

    // Generic
    bool _initialized;

public:
    MotorEncoder(uint8_t potPin, uint16_t pos_init, uint16_t pos_end, float angle_init, float angle_end, float std, float Q, float P_init);
    uint16_t readPot() const;
    float getAngle() const;
    float getFilteredAngle();
};

#endif  // _MOTOR_ENCODER_H_