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

    // Kalman filter variables
    float _std; 
    float _r; 
    float _x; 
    float _p;
    float _q;
    void updateKalmanVariables(){
        _r = _std * _std;
    }

public:
    MotorEncoder(uint8_t potPin, uint16_t pos_init, uint16_t pos_end, float std, float Q, float x_init, float P_init);
    uint16_t readPot();
    float getAngle();
    float getFilteredAngle();
};

#endif  // _MOTOR_ENCODER_H_