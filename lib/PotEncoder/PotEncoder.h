#ifndef _POT_ENCONDER_
#define _POT_ENCODER_

#include <Arduino.h>
#include <math.h>

class PotEncoder
{
private:
    uint8_t _potPin;
    uint16_t _pos_init;
    uint16_t _pos_end;

public:
    PotEncoder(uint8_t potPin, uint16_t pos_init, uint16_t pos_end);
    uint16_t read();
    float getAngle();
};

#endif