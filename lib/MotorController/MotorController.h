#ifndef _MOTOR_CONTROLLER_
#define _MOTOR_CONTROLLER_

#include <Arduino.h>

class MotorController
{
private:
    uint8_t _intA;
    uint8_t _intB;

public:
    MotorController(uint8_t intA, uint8_t intB);
    void action(int action);
};

#endif